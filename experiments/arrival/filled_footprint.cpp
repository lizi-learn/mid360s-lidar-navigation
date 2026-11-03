#include <iostream>
#include <stdexcept>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "p2_nav_controller/filled_footprint.hpp"
int main() {
 try {
  nav2_costmap_2d::Costmap2D map(44,44,.05,-1.1,-1.1,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:{std::array<double,2>{.275,.275},{.275,-.275},{-.275,-.275},{-.275,.275}}){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];body.push_back(p);}
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> outline(&map);
  unsigned x,y;map.worldToMap(.01,.01,x,y);map.setCost(x,y,254);
  if(outline.footprintCostAtPose(0.,0.,.37,body)>=254)throw std::runtime_error("Outline baseline did not miss interior cell");
  if(p2_nav_controller::filled_footprint_clear(&map,body,0.,0.,.37))throw std::runtime_error("Filled check missed interior obstacle");
  map.setCost(x,y,255);
  if(p2_nav_controller::filled_footprint_clear(&map,body,0.,0.,.37))throw std::runtime_error("Unknown cell became free");
  map.setCost(x,y,0);
  for(int i=0;i<72;++i)if(!p2_nav_controller::filled_footprint_clear(&map,body,0.,0.,i*2*std::acos(-1.)/72))throw std::runtime_error("Empty room rotation rejected");
  if(p2_nav_controller::filled_footprint_clear(&map,body,1.05,0.,0.))throw std::runtime_error("Off-map footprint accepted");
  std::cout<<"{\"passed\":true,\"outline_interior_miss_reproduced\":true,\"filled_obstacle_and_unknown_rejected\":true,\"empty_rotation_passes\":72,\"map_boundary_preserved\":true,\"footprint_side_m\":0.55}\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
