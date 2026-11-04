#pragma once
// Fixed-size scalar input trajectory. Value is a policy input, derivative is
// input change per second. No hardware or source/ownership decisions here.
#include <algorithm>
#include <array>
#include <cmath>
namespace p2_nav {
class SmoothInputAxis {
 struct Phase {double duration=0.,jerk=0.;};
 std::array<Phase,3> phases_{};
 double value_=0.,acceleration_=0.,target_=0.,left_=0.;
 unsigned phase_=3;
 public:
 void reset(){value_=acceleration_=target_=left_=0.;phase_=3;}
 double value()const{return value_;}
 double acceleration()const{return acceleration_;}
 double step(double target,double dt,double limit,double jerk){
  if(target!=target_){
   target_=target;phase_=0;
   const double delta=target-value_;
   const double stop_delta=acceleration_*std::abs(acceleration_)/(2.*jerk);
   const double direction=delta>=stop_delta?1.:-1.;
   const double peak=std::sqrt(std::max(0.,direction*jerk*delta+.5*acceleration_*acceleration_));
   const double top=std::min(peak,limit);
   const double rise=std::max(0.,(direction*top-acceleration_)/(direction*jerk));
   const double ramp_delta=(2.*top*top-acceleration_*acceleration_)/(2.*direction*jerk);
   const double hold=top>0.?std::max(0.,(delta-ramp_delta)/(direction*top)):0.;
   phases_={Phase{rise,direction*jerk},Phase{hold,0.},Phase{top/jerk,-direction*jerk}};
   left_=phases_[0].duration;
  }
  while(dt>0. && phase_<3){
   const double t=std::min(dt,left_),j=phases_[phase_].jerk;
   value_+=acceleration_*t+.5*j*t*t;acceleration_+=j*t;
   dt-=t;left_-=t;
   if(left_<=1e-12){++phase_;if(phase_<3)left_=phases_[phase_].duration;}
  }
  if(phase_==3){value_=target_;acceleration_=0.;}
  return value_;
 }
};

} // namespace p2_nav
