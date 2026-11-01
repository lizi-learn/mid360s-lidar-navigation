#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>
namespace p2_nav_controller {
// A point-route supplies search headings ONLY. It never authorizes a body path;
// the lattice and all added pivots must still check the actual full footprint.
inline std::vector<std::array<double,2>> position_guide(
    const unsigned char *cells, unsigned width, unsigned height, double resolution,
    double ox, double oy, const std::array<double,2> &start,
    const std::array<double,2> &goal) {
  if (!cells || !width || !height || !std::isfinite(resolution) || resolution<=0 || !std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(start[0]) || !std::isfinite(start[1]) || !std::isfinite(goal[0]) || !std::isfinite(goal[1]))throw std::invalid_argument("Invalid guide map");
  auto index=[&](const std::array<double,2> &p) {
    const int x=static_cast<int>(std::floor((p[0]-ox)/resolution));
    const int y=static_cast<int>(std::floor((p[1]-oy)/resolution));
    if(x<0 || y<0 || x>=static_cast<int>(width) || y>=static_cast<int>(height))
      throw std::invalid_argument("Guide endpoint outside map");
    return y*static_cast<int>(width)+x;
  };
  const int from=index(start),to=index(goal),size=width*height,w=static_cast<int>(width);
  if(cells[from]>=254 || cells[to]>=254)return {};
  std::vector<double> cost(size,std::numeric_limits<double>::infinity());
  std::vector<int> parent(size,-1);
  using Item=std::pair<double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
  auto heuristic=[&](int p){return std::hypot(p%w-to%w,p/w-to/w);};
  cost[from]=0.;open.push({heuristic(from),from});
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(250);
  int iterations=0;
  while(!open.empty()) {
    const auto [f,p]=open.top();open.pop();
    if(f>cost[p]+heuristic(p)+1e-8)continue;
    if(p==to)break;
    if((++iterations%1024)==0 && std::chrono::steady_clock::now()>deadline)return {};
    const int x=p%width,y=p/width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      if((dx==0&&dy==0)||x+dx<0||y+dy<0||x+dx>=static_cast<int>(width)||y+dy>=static_cast<int>(height))continue;
      const int q=(y+dy)*width+x+dx;
      if(cells[q]>=253)continue;
      if(dx&&dy&&(cells[y*width+x+dx]>=253||cells[(y+dy)*width+x]>=253))continue;
      const double next=cost[p]+std::hypot(dx,dy)*(1.+2.*cells[q]/252.);
      if(next<cost[q]){cost[q]=next;parent[q]=p;open.push({next+heuristic(q),q});}
    }
  }
  if(from!=to && parent[to]<0)return {};
  std::vector<std::array<double,2>> path;
  for(int p=to;;p=parent[p]) {
    path.push_back({ox+(p%width+.5)*resolution,oy+(p/width+.5)*resolution});
    if(p==from)break;
  }
  std::reverse(path.begin(),path.end());path.front()=start;path.back()=goal;return path;
}
inline double guide_heading(const std::vector<std::array<double,2>> &p,bool arrival) {
  if(p.size()<2)return 0.;
  const auto end=arrival?p.back():p.front();auto other=end;
  for(size_t n=1;n<p.size();++n){other=arrival?p[p.size()-1-n]:p[n];if(std::hypot(other[0]-end[0],other[1]-end[1])>=.75)break;}
  return arrival?std::atan2(end[1]-other[1],end[0]-other[0]):std::atan2(other[1]-end[1],other[0]-end[0]);
}
}
