#pragma once
#include <algorithm>
#include <cmath>
namespace p2_nav_controller {
// Manual-equivalent input, not measured m/s. The ordinary downstream operator
// multiplier and input ramp are retained. Zero/cancel can never become motion.
inline double curved_forward(double forward,double yaw,double curved_limit) {
 if(curved_limit<=0.||forward<=0.)return forward;
 const double t=std::clamp((std::abs(yaw)-.15)/.25,0.,1.);
 const double blend=t*t*(3.-2.*t);
 return forward+(std::min(forward,curved_limit)-forward)*blend;
}
}
