#pragma once
// User-requested D-policy manual-equivalent input. Full forward0.8 times
// upstream half-speed0.5 gives policy input0.4. This is not measured body speed.
// Actual velocity remains separately reported; no model curve or PI gain is used.
// Pure arithmetic: no hardware, socket, allocation or filesystem operations.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "input_s_curve.hpp"

namespace p2_nav {
struct SpeedTrackingState {
  double target_x=0., target_yaw=0., measured_x=0., measured_yaw=0.;
  double policy_x=0., policy_yaw=0.;
  std::uint64_t source_ns=0;
  std::uint32_t valid=0, reason=0;
};
enum SpeedTrackingReason : std::uint32_t {
  speed_idle=0, speed_tracking=1, speed_feedback_expired=2,
  speed_invalid_input=3, speed_overspeed=4
};
class SpeedResponse {
  SmoothInputAxis axes_[2];
  SpeedTrackingState state_{};
 public:
  void reset(){
    for(auto& axis:axes_)axis.reset();
    state_={};
  }
  const SpeedTrackingState& state()const{return state_;}
  SpeedTrackingState step(double vx,double wz,double measured_x,double measured_yaw,
                          double dt,std::uint64_t source_ns,std::uint64_t now_ns,bool owned){
    if(!owned || (vx==0. && wz==0.)){reset();return state_;}
    if(!std::isfinite(vx)||!std::isfinite(wz)||!std::isfinite(measured_x)||
       !std::isfinite(measured_yaw)||!std::isfinite(dt)||dt<=0.||dt>.1||
       vx<0.||vx>.800001||std::abs(wz)>1.000001||
       std::abs(measured_x)>2.||std::abs(measured_yaw)>4.){
      reset();state_.reason=speed_invalid_input;return state_;
    }
    // The same original 500ms source expiry as the navigation sensor contract.
    // A transport packet or controller tick must never refresh this timestamp.
    if(!source_ns || (source_ns>now_ns ? source_ns-now_ns>5000000ULL : now_ns-source_ns>500000000ULL)){
      reset();state_.reason=speed_feedback_expired;return state_;
    }
    state_.target_x=vx;state_.target_yaw=wz;
    state_.measured_x=measured_x;state_.measured_yaw=measured_yaw;
    state_.source_ns=source_ns;state_.valid=1;state_.reason=speed_tracking;
    // One finite jerk-constrained input trajectory at the original native tick.
    // Nonzero target changes retain the current derivative. Yaw keeps the
    // original .5 input/s slope, with a short finite slope transition: a slow
    // steering filter adds closed-loop phase lag and destabilizes pursuit.
    // An axis explicitly withdrawn by the controller is zero immediately,
    // including translation withdrawn for a collision-checked pivot. Such a
    // zero must not acquire a hidden coast-down tail.
    const double target[2]={vx,wz}, acceleration[2]={.30,.50}, jerk[2]={.40,6.};
    if(target[0]==0.)axes_[0].reset();
    state_.policy_x=axes_[0].step(target[0],dt,acceleration[0],jerk[0]);
    if(target[1]==0.)axes_[1].reset();
    state_.policy_yaw=axes_[1].step(target[1],dt,acceleration[1],jerk[1]);
    return state_;
  }
};
} // namespace p2_nav
