#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace p2_nav_controller {
// Within twice the original XY tolerance, approach the position facing forward;
// then align the requested final yaw. Every request still needs collision checks.
inline std::array<double,2> terminal_velocity(double distance,double bearing_error,
                                            double heading_error,bool rotate_final,double max_w) {
  if (!std::isfinite(distance)||distance<0||!std::isfinite(bearing_error)||
      !std::isfinite(heading_error)||!std::isfinite(max_w)||max_w<=0)
    throw std::invalid_argument("Invalid terminal velocity input");
  const double error=std::remainder(rotate_final?heading_error:bearing_error,2.*std::acos(-1.));
  const double w=std::copysign(std::min({max_w,.8*std::abs(error),std::sqrt(std::abs(error))}),error);
  const double v=!rotate_final && std::abs(error)<.15?std::min(.1,.5*distance):0.;
  return {v,w};
}

// Once forward acquisition starts, small pose fluctuations must not repeatedly
// erase the downstream startup ramp. Leave it only for a material bearing
// change; final heading and success tolerances remain the goal checker's job.
class TerminalApproach {
  bool advancing_=false;
  double last_=-1.;
 public:
  void reset(){advancing_=false;last_=-1.;}
  std::array<double,2> step(double distance,double bearing,double heading,
                          bool rotate_final,double max_w,double now){
    auto command=terminal_velocity(distance,bearing,heading,rotate_final,max_w);
    if(!std::isfinite(now))throw std::invalid_argument("Invalid terminal approach time");
    if(last_>=0.&&(now<last_||now-last_>.5))reset();
    last_=now;
    const double angle=std::abs(std::remainder(bearing,2.*std::acos(-1.)));
    if(rotate_final||angle>=.35)advancing_=false;
    else if(angle<.15)advancing_=true;
    if(!rotate_final&&advancing_)command[0]=std::min(.1,.5*distance);
    return command;
  }
};
}
