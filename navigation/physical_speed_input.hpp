#pragma once
// Pure velocity-source arbitration. No motor, joystick, shared-memory or socket I/O.
#include "velocity_contract.hpp"
#include <cstddef>

namespace p2_nav {
struct GaitFeedback {
  std::uint64_t monotonic_ns=0, controller_instance=0, policy_generation=0;
  bool supported_policy=false, walking_ready=false, operator_connected=false;
  bool manual_activity=false, estop=false, healthy=false, calibrated=false;
  bool ground_ok=false, localized=false;
};
struct VelocitySelection {
  double vx=0., vy=0., wz=0.;
  bool navigation_owns_velocity=false, cancel_navigation_goal=false;
  const char* reason="navigation_disarmed";
};

class PhysicalSpeedInput {
  static constexpr std::uint64_t feedback_timeout=100000000ULL;
  static constexpr std::uint64_t command_timeout=300000000ULL;
  static constexpr std::uint64_t future_tolerance=5000000ULL;
  bool armed_=false, bound_=false, cancel_pending_=false;
  std::uint64_t controller_=0, generation_=0, source_=0, sequence_=0;
  std::uint64_t command_stamp_=0, armed_at_=0, last_now_=0;
  double vx_=0., wz_=0.;
  const char* reason_="navigation_disarmed";
  static bool fresh(std::uint64_t stamp,std::uint64_t now,std::uint64_t maximum){
    return stamp && (stamp>now ? stamp-now<=future_tolerance : now-stamp<=maximum);
  }
  const char* feedback_fault(const GaitFeedback& f,std::uint64_t now) const {
    if(f.estop)return "estop";
    if(f.manual_activity)return "manual_override";
    if(!fresh(f.monotonic_ns,now,feedback_timeout))return "controller_feedback_expired";
    if(!f.operator_connected)return "operator_disconnected";
    if(!f.supported_policy||!f.walking_ready)return "policy_not_ready";
    if(!f.controller_instance||!f.policy_generation)return "missing_controller_identity";
    if(!f.healthy||!f.calibrated||!f.ground_ok||!f.localized)return "navigation_health_not_ready";
    if(armed_&&(controller_!=f.controller_instance||generation_!=f.policy_generation))return "controller_or_policy_changed";
    return nullptr;
  }
  void revoke(const char* why){
    if(armed_)cancel_pending_=true;
    armed_=bound_=false;vx_=wz_=0.;reason_=why;
  }
  bool check(const GaitFeedback& f,std::uint64_t now){
    if(last_now_&&now<last_now_){last_now_=now;revoke("local_clock_rollback");return false;}
    last_now_=now;
    if(const char* why=feedback_fault(f,now)){revoke(why);return false;}
    return true;
  }
 public:
  // Call only for an explicit operator start action, with feedback from the gait
  // loop, after the caller has acknowledged cancellation of the previous goal.
  bool arm(const GaitFeedback& f,std::uint64_t now){
    if(armed_||cancel_pending_)return false;
    if(!check(f,now))return false;
    controller_=f.controller_instance;generation_=f.policy_generation;
    source_=sequence_=command_stamp_=0;armed_at_=now;bound_=false;armed_=true;
    reason_="awaiting_zero_velocity_handshake";return true;
  }
  void disarm(){revoke("operator_stopped_navigation");}
  bool armed() const{return armed_;}
  bool accept(const Packet& p,std::size_t bytes,const GaitFeedback& f,std::uint64_t now){
    if(!check(f,now)||!armed_)return false;
    // Expiry cannot be hidden by a packet arriving just before the next poll.
    const auto previous_stamp=bound_?command_stamp_:armed_at_;
    if(now>=previous_stamp && now-previous_stamp>command_timeout){
      revoke("navigation_command_expired");return false;
    }
    if(bytes!=sizeof(Packet)||p.magic!=0x504e4156||p.version!=1||p.flags!=1||p.reserved||!p.instance||!p.sequence){revoke("invalid_navigation_packet");return false;}
    if(!std::isfinite(p.vx)||!std::isfinite(p.vy)||!std::isfinite(p.wz)||p.vx<0||p.vx>.15f||p.vy!=0.f||std::abs(p.wz)>.24f){revoke("forward_velocity_contract");return false;}
    if(!fresh(p.monotonic_ns,now,command_timeout)){revoke("navigation_packet_expired");return false;}
    if(p.monotonic_ns<armed_at_){revoke("navigation_packet_predates_arm");return false;}
    if(bound_&&p.instance!=source_){revoke("navigation_source_changed");return false;}
    if(p.sequence<=sequence_){revoke("navigation_packet_replayed");return false;}
    if(!bound_&&(p.vx!=0.f||p.wz!=0.f)){revoke("zero_velocity_handshake_required");return false;}
    if(bound_&&p.monotonic_ns<command_stamp_){revoke("navigation_source_clock_rollback");return false;}
    source_=p.instance;sequence_=p.sequence;command_stamp_=p.monotonic_ns;
    vx_=p.vx;wz_=p.wz;bound_=true;reason_="navigation_active";return true;
  }
  // Call each gait tick, even when no new packet arrives. A healthy neutral
  // joystick never re-arms a lease revoked by manual intervention or a fault.
  VelocitySelection poll(const GaitFeedback& f,std::uint64_t now){
    check(f,now);
    if(armed_){
      const auto stamp=bound_?command_stamp_:armed_at_;
      if(now>=stamp&&now-stamp>command_timeout)revoke("navigation_command_expired");
    }
    VelocitySelection d;
    if(armed_&&bound_){d.vx=vx_;d.wz=wz_;d.navigation_owns_velocity=true;}
    d.reason=reason_;d.cancel_navigation_goal=cancel_pending_;cancel_pending_=false;
    return d;
  }
};
} // namespace p2_nav
