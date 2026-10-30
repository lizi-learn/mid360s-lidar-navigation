#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace p2_nav_controller {
// Persist the terminal rotation across replans of the same goal. The original
// goal checker still decides success. Large drift and control gaps reacquire XY.
class TerminalHeading {
 public:
  void reset() { latched_=false; last_time_=-1.; }
  void set_goal(double x,double y,double yaw,const std::string &frame) {
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||frame.empty()) {
      reset(); have_goal_=false; throw std::invalid_argument("Invalid terminal goal");
    }
    if (!have_goal_ || frame!=frame_ || std::hypot(x-x_,y-y_)>1e-6 ||
        std::abs(std::remainder(yaw-yaw_,2.*std::acos(-1.)))>1e-6) reset();
    x_=x; y_=y; yaw_=yaw; frame_=frame; have_goal_=true;
  }
  bool step(double distance,double tolerance,double now,bool heading_aligned=false) {
    if (!have_goal_ || !std::isfinite(distance) || distance<0 ||
        !std::isfinite(tolerance) || tolerance<=0 || !std::isfinite(now)) {
      reset(); throw std::invalid_argument("Invalid terminal heading observation");
    }
    if (last_time_>=0 && (now<last_time_ || now-last_time_>.5)) latched_=false;
    last_time_=now;
    if (distance>2.*tolerance) latched_=false;
    if (distance>=tolerance && heading_aligned) latched_=false;
    if (distance<tolerance) latched_=true;
    return latched_;
  }
 private:
  bool have_goal_=false,latched_=false;
  double x_=0,y_=0,yaw_=0,last_time_=-1.;
  std::string frame_;
};
}
