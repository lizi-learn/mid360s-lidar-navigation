#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "geometry_msgs/msg/point.hpp"
namespace p2_nav_controller {
// SAT intersection of the filled robot polygon with whole occupied/unknown
// grid cells. Outline-only checks miss interior cells and cell-edge contacts.
inline bool filled_footprint_clear(nav2_costmap_2d::Costmap2D *map,
    const std::vector<geometry_msgs::msg::Point> &body,double x,double y,double yaw) {
  if(body.size()<3)return false;
  std::vector<std::array<double,2>> polygon;
  double lx=x,ux=x,ly=y,uy=y;const double c=std::cos(yaw),s=std::sin(yaw);
  for(const auto &p:body){double px=x+c*p.x-s*p.y,py=y+s*p.x+c*p.y;polygon.push_back({px,py});lx=std::min(lx,px);ux=std::max(ux,px);ly=std::min(ly,py);uy=std::max(uy,py);}
  unsigned ax,ay,bx,by;
  if(!map->worldToMap(lx,ly,ax,ay)||!map->worldToMap(ux,uy,bx,by))return false;
  const double half=map->getResolution()/2.;
  for(int iy=std::max(0,static_cast<int>(ay)-1);iy<=std::min(static_cast<int>(map->getSizeInCellsY())-1,static_cast<int>(by)+1);++iy)
  for(int ix=std::max(0,static_cast<int>(ax)-1);ix<=std::min(static_cast<int>(map->getSizeInCellsX())-1,static_cast<int>(bx)+1);++ix) {
    if(map->getCost(ix,iy)<254)continue;
    double px,py;map->mapToWorld(ix,iy,px,py);
    if(px+half<lx||px-half>ux||py+half<ly||py-half>uy)continue;
    bool separated=false;
    for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
      const auto &a=polygon[i];const auto &b=polygon[j];const double nx=b[1]-a[1],ny=a[0]-b[0];
      double low=1e30,high=-1e30;for(const auto &v:polygon){double q=nx*v[0]+ny*v[1];low=std::min(low,q);high=std::max(high,q);}
      double centre=nx*px+ny*py,extent=half*(std::abs(nx)+std::abs(ny));
      if(centre+extent<low||centre-extent>high){separated=true;break;}
    }
    if(!separated)return false;
  }
  return true;
}
}
