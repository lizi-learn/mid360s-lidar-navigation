#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <vector>
#include "p2_nav_controller/filled_footprint.hpp"

namespace p2_nav_controller {
namespace boundary_geometry {
using Point=std::array<double,2>;
using Polygon=std::vector<Point>;
inline double cross(const Point &a,const Point &b,const Point &c) {
  return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
}
inline Polygon hull(Polygon p) {
  std::sort(p.begin(),p.end());p.erase(std::unique(p.begin(),p.end()),p.end());
  if(p.size()<3)return {};
  Polygon out;
  for(const auto &v:p){while(out.size()>1&&cross(out[out.size()-2],out.back(),v)<=0.)out.pop_back();out.push_back(v);}
  const auto lower=out.size();
  for(auto i=p.rbegin()+1;i!=p.rend();++i){while(out.size()>lower&&cross(out[out.size()-2],out.back(),*i)<=0.)out.pop_back();out.push_back(*i);}
  out.pop_back();return out;
}
inline Polygon clip(const Polygon &p,int axis,double bound,int sign) {
  Polygon out;if(p.empty())return out;
  for(size_t i=0;i<p.size();++i){const auto &a=p[i],&b=p[(i+1)%p.size()];
    const bool ia=sign*(a[axis]-bound)>=0.,ib=sign*(b[axis]-bound)>=0.;
    if(ia)out.push_back(a);
    if(ia!=ib){const double f=(bound-a[axis])/(b[axis]-a[axis]);out.push_back({a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])});}}
  return out;
}
inline double area(const Polygon &p) {
  if(p.size()<3)return 0.;
  // Subtract a nearby origin to avoid cancellation on large map coordinates.
  double sum=0.;for(size_t i=1;i+1<p.size();++i)sum+=cross(p[0],p[i],p[i+1]);
  return std::abs(sum)*.5;
}
inline Polygon cell_clip(Polygon p,double x,double y,double r) {
  p=clip(p,0,x,1);p=clip(p,0,x+r,-1);p=clip(p,1,y,1);return clip(p,1,y+r,-1);
}
inline bool contains(const Polygon &p,double x,double y) {
  for(size_t i=0;i<p.size();++i)if(cross(p[i],p[(i+1)%p.size()],{x,y})<-1e-12)return false;
  return !p.empty();
}
inline Polygon at(const std::vector<geometry_msgs::msg::Point> &body,double x,double y,double a) {
  Polygon p;for(const auto &v:body)p.push_back({x+std::cos(a)*v.x-std::sin(a)*v.y,y+std::sin(a)*v.x+std::cos(a)*v.y});return hull(p);
}
}

// A whole-cell collision can start with a very small boundary overlap. A
// forward translation may REMOVE that overlap without sweeping any additional
// occupied space. Check the exact convex swept polygon, not just cell centres,
// an outline, a smaller body, or a sum of decreasing overlap areas.
// Costmap locking and freshness remain the caller's responsibility.
inline bool boundary_forward_escape(nav2_costmap_2d::Costmap2D *map,
    const std::vector<geometry_msgs::msg::Point> &body,double x,double y,double yaw,double distance,
    const std::array<double,3> *origin=nullptr) {
  using namespace boundary_geometry;
  if(!map||body.size()<3||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
      !std::isfinite(distance)||distance<=0.||distance>.6)return false;
  const auto current=at(body,x,y,yaw);
  const auto start=origin?at(body,(*origin)[0],(*origin)[1],(*origin)[2]):current;
  // Only convex footprints use this translation proof; never replace a concave
  // body with its hull and pretend its original collision check was satisfied.
  if(start.size()!=body.size())return false;
  const double ex=x+distance*std::cos(yaw),ey=y+distance*std::sin(yaw);
  if(filled_footprint_clear(map,body,x,y,yaw)||!filled_footprint_clear(map,body,ex,ey,yaw))return false;
  auto swept=current;auto end=at(body,ex,ey,yaw);swept.insert(swept.end(),end.begin(),end.end());swept=hull(swept);
  double lx=swept[0][0],ux=lx,ly=swept[0][1],uy=ly;
  for(const auto &p:swept){lx=std::min(lx,p[0]);ux=std::max(ux,p[0]);ly=std::min(ly,p[1]);uy=std::max(uy,p[1]);}
  unsigned ax,ay,bx,by;if(!map->worldToMap(lx,ly,ax,ay)||!map->worldToMap(ux,uy,bx,by))return false;
  const double r=map->getResolution();double initial_area=0.;bool have_overlap=false;
  for(unsigned iy=ay;iy<=by;++iy)for(unsigned ix=ax;ix<=bx;++ix) {
    const auto cost=map->getCost(ix,iy);if(cost<254)continue;
    double cx,cy;map->mapToWorld(ix,iy,cx,cy);const double a=area(cell_clip(start,cx-r/2.,cy-r/2.,r));
    const auto intersection=cell_clip(swept,cx-r/2.,cy-r/2.,r);
    if(intersection.empty())continue;
    // A touched unknown cell never becomes an escape allowance. A known
    // occupied centre inside the actual body is not a boundary-cell artefact.
    if(cost==255||contains(start,cx,cy))return false;
    for(const auto &p:intersection)if(!contains(start,p[0],p[1]))return false;
    const double b=area(intersection);
    if(a<=1e-12||b>a+1e-10)return false;
    have_overlap=true;initial_area+=a;
  }
  // Keep this mode restricted to a grid-edge sliver (at most 1/4 of one cell),
  // not a general motion-through-collision recovery. Occupancy is never edited.
  return have_overlap&&initial_area<=r*r*.25;
}
}
