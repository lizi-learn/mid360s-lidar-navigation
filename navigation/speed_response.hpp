#pragma once
// Physical speed targets and learned-policy observation inputs are different
// quantities. This controller uses measured velocity, not a model inverse curve.
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
       vx<0.||vx>.150001||std::abs(wz)>.240001||
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
    const double target[2]={vx,wz}, measured[2]={measured_x,measured_yaw};
    // Candidate actor-input bounds/rates are controller tuning, not physical
    // speed limits or a claim that a model response curve calibrates hardware.
    const double maximum[2]={.30,.50}, slew[2]={.15,.30};
    const double proportional[2]={.10,.12}, integration[2]={.15,.35};
    for(unsigned i=0;i<2;++i){
      if(!initialized_)filtered_[i]=measured[i];
      else filtered_[i]+=dt/(.15+dt)*(measured[i]-filtered_[i]);
      if(target[i]==0.){integral_[i]=output_[i]=moving_time_[i]=0.;started_[i]=false;continue;}
      const double sign=target[i]>0.?1.:-1.;
      if(output_[i]*sign<0.){integral_[i]=output_[i]=moving_time_[i]=0.;started_[i]=false;continue;}
      const double error=sign*(target[i]-filtered_[i]);
      // Search gradually only while actual measured motion remains below a
      // fifth of the target. Once moving, ordinary PI avoids the bias produced
      // by sign-normalizing oscillatory biped feedback.
      const double ki=i==0&&std::abs(target[i])<.05?.60:integration[i];
      double rate=ki*error;
      // A sustained measured onset ends the search. A short pose oscillation
      // is not an onset; later gait phases must not add a search bias to PI.
      moving_time_[i]=std::max(0.,moving_time_[i]+
        (sign*filtered_[i]>=.5*std::abs(target[i])?dt:-dt));
      if(moving_time_[i]>=1.)started_[i]=true;
      if(!started_[i] && sign*filtered_[i]<.5*std::abs(target[i]) && error>0.)
        rate=std::max(rate,i==0?.04:.06);
      const double delta=rate*dt;
      double next=std::clamp(integral_[i]+delta,0.,maximum[i]);
      const double kp=proportional[i];
      const double requested=std::clamp(next+kp*error,0.,maximum[i]);
      const double applied=std::clamp(sign*requested,output_[i]-slew[i]*dt,output_[i]+slew[i]*dt);
      // Freeze only extra positive windup at an upper actuator slew limit.
      // Negative error must still lower the integral while braking is slewed.
      if(error>0. && (sign*requested-applied)*sign>0.)next=integral_[i];
      integral_[i]=next;
      output_[i]=std::clamp(sign*std::clamp(next+kp*error,0.,maximum[i]),
                            output_[i]-slew[i]*dt,output_[i]+slew[i]*dt);
    }
    initialized_=true;
    state_.policy_x=output_[0];state_.policy_yaw=output_[1];return state_;
  }
};
} // namespace p2_nav
