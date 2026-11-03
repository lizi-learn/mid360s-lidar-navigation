#pragma once
#include <chrono>
#include "p2_nav_controller/terminal_path.hpp"
namespace p2_nav_controller {
// Rejoin beyond the controller's normal lookahead, so small tracking errors do
// not create a short sideways leg followed immediately by the opposite turn.
// The horizon bounds BOTH anchor search and connection construction. Previously
// extending the search alone still left terminal_path capped at one metre.
// Every proposal includes the remaining suffix and needs a current-map sweep.
template<class Sweep>
std::vector<std::array<double,3>> route_rejoin(
    const std::array<double,3> &start,
    const std::vector<std::array<double,3>> &route,double horizon,Sweep clear,
    std::chrono::steady_clock::time_point deadline) {
  if(route.empty()||!std::isfinite(horizon)||horizon<=0.)return {};
  size_t nearest=0;double best=1e30;
  for(size_t i=0;i<route.size();++i){double d=std::hypot(route[i][0]-start[0],route[i][1]-start[1]);if(d<best){best=d;nearest=i;}}
  if(best>=.5)return {};
  size_t end=nearest;
  while(end+1<route.size()&&std::hypot(route[end+1][0]-start[0],route[end+1][1]-start[1])<horizon)++end;
  for(size_t n=end+1;n>nearest;--n) {
    if(std::chrono::steady_clock::now()>deadline)break;
    const size_t i=n-1;
    auto joined=terminal_path(start,route[i],horizon);
    if(joined.empty())continue;
    joined.insert(joined.end(),route.begin()+i+1,route.end());
    if(clear(joined))return joined;
  }
  return {};
}
}
