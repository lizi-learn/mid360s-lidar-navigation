#pragma once
#include <cmath>
namespace p2_nav_controller {
// Internal goal completion, never an operator authorization or document gate.
// Position-only transit goals retain their separate unchanged goal checker.
class SettledArrival {
 double last_=-1.,quiet_since_=-1.;
 public:
 void reset(){last_=quiet_since_=-1.;}
 bool step(double distance,double error,double vx,double vy,double wz,double now,double xy,double yaw) {
  if(!std::isfinite(distance)||!std::isfinite(error)||!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(wz)||!std::isfinite(now)) {reset();return false;}
  if(last_>=0.&&(now<last_||now-last_>.3))quiet_since_=-1.;
  last_=now;
  if(distance>xy||std::abs(error)>yaw||std::hypot(vx,vy)>=.03||std::abs(wz)>=.03){quiet_since_=-1.;return false;}
  if(quiet_since_<0.)quiet_since_=now;
  return now-quiet_since_>=2.;
 }
};
}
