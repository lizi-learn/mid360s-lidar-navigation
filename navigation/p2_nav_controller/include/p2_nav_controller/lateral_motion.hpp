#pragma once
#include <cmath>
#include <stdexcept>

namespace p2_nav_controller {
// Estimate sustained sideways motion, rather than projecting one alternating
// gait/head-sway sample unchanged over the entire collision horizon. Original
// poses, maps, source-time checks and the filled footprint remain untouched.
class LateralMotion {
  bool valid_=false;
  double time_=0.,value_=0.;
 public:
  void reset(){valid_=false;time_=value_=0.;}
  double step(double sample,double now){
    if(!std::isfinite(sample)||!std::isfinite(now))
      throw std::invalid_argument("Invalid lateral motion sample");
    if(!valid_||now<time_||now-time_>.5){value_=sample;valid_=true;}
    else value_+=(sample-value_)*(-std::expm1(-(now-time_)/.3));
    time_=now;return value_;
  }
};
}
