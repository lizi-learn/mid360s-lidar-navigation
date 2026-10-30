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
}
