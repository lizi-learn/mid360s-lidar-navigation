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
  SmoothInputAxis forward_;
  double yaw_=0.;
  SpeedTrackingState state_{};
 public:
  void reset(){
    forward_.reset();yaw_=0.;
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
    // Forward starts and nonzero speed changes use the finite S trajectory.
    // Keep the original yaw slew exactly: replacing that closed-loop steering
    // response with a slower filter changes both pursuit and pivot pulse area.
    // Explicit axis withdrawal keeps the original immediate-zero contract.
    if(vx==0.)forward_.reset();
    state_.policy_x=forward_.step(vx,dt,.30,.40);
    if(wz==0.)yaw_=0.;
    else yaw_=std::clamp(wz,yaw_-.5*dt,yaw_+.5*dt);
    state_.policy_yaw=yaw_;
    return state_;
  }
};
} // namespace p2_nav
