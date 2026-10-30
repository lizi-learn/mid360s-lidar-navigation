#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace p2_nav_controller {
// A short forward connection avoids lattice start/goal discretizing to the same
// node. The caller must collision-check every sample using the planning footprint.
inline std::vector<std::array<double,3>> terminal_path(
    const std::array<double,3> &start,const std::array<double,3> &goal) {
  for (double v:start)if(!std::isfinite(v))throw std::invalid_argument("Invalid terminal start");
  for (double v:goal)if(!std::isfinite(v))throw std::invalid_argument("Invalid terminal goal");
  const double distance=std::hypot(goal[0]-start[0],goal[1]-start[1]);
  if(distance>.30)return {};
  std::vector<std::array<double,3>> samples{start};
  auto rotate=[&](double yaw) {
    const auto from=samples.back();
    const double delta=std::remainder(yaw-from[2],2.*std::acos(-1.));
    const int n=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/.02)));
    for(int i=1;i<=n;++i)samples.push_back({from[0],from[1],from[2]+delta*i/n});
  };
  if(distance>1e-6) {
    const double bearing=std::atan2(goal[1]-start[1],goal[0]-start[0]);
    rotate(bearing);
    const int n=std::max(1,static_cast<int>(std::ceil(distance/.02)));
    for(int i=1;i<=n;++i)samples.push_back({start[0]+(goal[0]-start[0])*i/n,start[1]+(goal[1]-start[1])*i/n,bearing});
  }
  rotate(goal[2]);
  return samples;
}
}
