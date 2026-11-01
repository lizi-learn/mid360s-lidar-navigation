#pragma once
// User-requested D-policy manual-equivalent input. Full forward0.8 times
// upstream half-speed0.5 gives policy input0.4. This is not measured body speed.
// Actual velocity remains separately reported; no model curve or PI gain is used.
// Pure arithmetic: no hardware, socket, allocation or filesystem operations.
#include <algorithm>
#include <cmath>
#include <cstdint>

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
  double integral_[2]{}, output_[2]{}, filtered_[2]{};
  double moving_time_[2]{};
  bool started_[2]{};
  bool initialized_=false;
  SpeedTrackingState state_{};
 public:
  void reset(){
    for(unsigned i=0;i<2;++i){integral_[i]=output_[i]=filtered_[i]=moving_time_[i]=0.;started_[i]=false;}
    initialized_=false;state_={};
  }
  const SpeedTrackingState& state()const{return state_;}
  SpeedTrackingState step(double vx,double wz,double measured_x,double measured_yaw,
                          double dt,std::uint64_t source_ns,std::uint64_t now_ns,bool owned){
    if(!owned || (vx==0. && wz==0.)){reset();return state_;}
    if(!std::isfinite(vx)||!std::isfinite(wz)||!std::isfinite(measured_x)||
       !std::isfinite(measured_yaw)||!std::isfinite(dt)||dt<=0.||dt>.1||
       vx<0.||vx>.800001||std::abs(wz)>.240001||
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
    // A zero on takeover, collision or cancellation clears immediately above.
    // Other input changes are slewed; measured velocity never rescales this
    // explicit manual-equivalent command or silently multiplies half twice.
    const double target[2]={vx,wz}, slew[2]={.4,.30};
    for(unsigned i=0;i<2;++i){
      if(target[i]==0.)output_[i]=0.;
      else {
        const double rate=i==0 && target[i]<output_[i]?.8:slew[i];
        output_[i]=std::clamp(target[i],output_[i]-rate*dt,output_[i]+rate*dt);
      }
    }
    initialized_=true;
    state_.policy_x=output_[0];state_.policy_yaw=output_[1];return state_;
  }
};
} // namespace p2_nav
