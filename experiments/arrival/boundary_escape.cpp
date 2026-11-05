#include <fstream>
#include <iostream>
#include <stdexcept>
#include "p2_nav_controller/boundary_escape.hpp"
int main(int argc,char **argv) {
 try{
  nav2_costmap_2d::Costmap2D map(100,100,.05,-1.02,-1.02,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:{std::array<double,2>{.275,.275},{.275,-.275},{-.275,-.275},{-.275,.275}}){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];body.push_back(p);}
  double x=0.,y=0.,yaw=0.;unsigned ix,iy;
  auto mark=[&](double px,double py,int cost){if(!map.worldToMap(px,py,ix,iy))throw std::runtime_error("Probe off map");map.setCost(ix,iy,cost);};
  auto escape=[&](){return p2_nav_controller::boundary_forward_escape(&map,body,x,y,yaw,.6);};
  auto expect=[&](bool value,const char *name){if(escape()!=value)throw std::runtime_error(name);};
  expect(false,"Clear start must not select recovery");
  mark(-.24,-.295,254);expect(true,"Existing trailing grid sliver should clear forward");
  const std::array<double,3> origin{0.,0.,0.};
  if(p2_nav_controller::boundary_forward_escape(&map,body,.01,-.01,0.,.6,&origin))throw std::runtime_error("Measured drift increased original contact");
  if(!p2_nav_controller::boundary_forward_escape(&map,body,.01,0.,0.,.6,&origin))throw std::runtime_error("Straight measured progress lost escape");
  mark(.55,0.,254);expect(false,"New forward obstacle must be rejected");mark(.55,0.,0);
  mark(-.24,-.295,255);expect(false,"Unknown start overlap must be rejected");
  mark(-.24,-.295,254);mark(.55,0.,255);expect(false,"New unknown must be rejected");mark(.55,0.,0);
  mark(0.,0.,254);expect(false,"Interior occupied cell must be rejected");mark(0.,0.,0);
  mark(-.24,-.295,0);mark(.29,0.,254);expect(false,"Forward motion into boundary obstacle must be rejected");mark(.29,0.,0);
  mark(-.24,-.295,254);yaw=.1;expect(false,"Increasing same-cell contact must be rejected");yaw=0.;
  mark(-.24,-.245,254);expect(false,"Larger actual overlap must be rejected");mark(-.24,-.245,0);
  if(argc>1){
   std::ifstream in(argv[1]);unsigned w,h;double r,ox,oy;
   if(!(in>>w>>h>>r>>ox>>oy>>x>>y>>yaw))throw std::runtime_error("Missing frozen map geometry");
   map.resizeMap(w,h,r,ox,oy);for(unsigned i=0;i<w*h;++i){int c;if(!(in>>c))throw std::runtime_error("Incomplete frozen map");map.getCharMap()[i]=c;}
   expect(true,"Frozen actual forward escape rejected");
  }
  std::cout<<"{\"passed\":true,\"negative_cases\":9,\"synthetic_clear_escape\":true,\"frozen_actual_map\":"<<(argc>1?"true":"false")<<",\"body_side_m\":0.55,\"forward_bound_m\":0.6,\"occupied_cells_edited\":false}\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
