#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace p2_nav_controller {
// Near a directed stop, a small policy input may produce no steps. Observe
// actual position progress before increasing that input, within the existing
// controller speed limit. The caller checks the resulting full-body arc.
class TerminalInput {
  double stamp_=0.,progress_stamp_=0.,anchor_=0.,floor_=0.;
  bool active_=false;
 public:
  void reset(){stamp_=progress_stamp_=anchor_=floor_=0.;active_=false;}
  double step(double requested,double distance,double now,double limit){
    if(!std::isfinite(requested)||requested<0.||!std::isfinite(distance)||distance<0.||
       !std::isfinite(now)||!std::isfinite(limit)||limit<0.)
      throw std::invalid_argument("Invalid terminal input observation");
    if(requested==0.||limit==0.){reset();return 0.;}
    if(!active_||now<stamp_||now-stamp_>1.){
      active_=true;stamp_=progress_stamp_=now;anchor_=distance;floor_=0.;
    }
    const double dt=std::clamp(now-stamp_,0.,.1);stamp_=now;
    if(anchor_-distance>=.02){anchor_=distance;progress_stamp_=now;}
    if(now-progress_stamp_>=2.)floor_=std::min(limit,std::max(requested,floor_)+.4*dt);
    floor_=std::min(floor_,limit);
    return std::min(limit,std::max(requested,floor_));
  }
};
}
