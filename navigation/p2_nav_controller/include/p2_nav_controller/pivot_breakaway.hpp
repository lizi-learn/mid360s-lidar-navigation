#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace p2_nav_controller {
// Only assist a stalled pure pivot. Moving arcs keep their existing response;
// the ordinary downstream command ramp and collision projection still apply.
class PivotBreakaway {
 double since_=-1.,last_=-1.,boost_until_=-1.,retry_after_=-1.;
 public:
 void reset(){since_=last_=boost_until_=retry_after_=-1.;}
 double step(double request,double error,double rate,double tolerance,double minimum,double maximum,double now) {
  if(!std::isfinite(request)||!std::isfinite(error)||!std::isfinite(rate)||!std::isfinite(now)||minimum<0||minimum>maximum)
   throw std::invalid_argument("Invalid pivot response input");
  if(last_>=0&&(now<last_||now-last_>.3)){reset();}
  last_=now;
  if(std::abs(error)<=tolerance){reset();return 0.;}
  if(request==0.||minimum==0.||std::abs(request)>=minimum){since_=-1.;boost_until_=-1.;return request;}
  if(now<boost_until_)return std::copysign(minimum,request);
  if(now<retry_after_||std::abs(rate)>.06){since_=-1.;return request;}
  if(since_<0)since_=now;
  if(now-since_>=.5){boost_until_=now+.55;retry_after_=boost_until_+1.5;since_=-1.;return std::copysign(minimum,request);}
  return request;
 }
};
}
