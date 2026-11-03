#include <cstdlib>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
int main(int argc,char **argv){
 if(argc<10)return 2;
 rclcpp::init(argc,argv);int rc=0;
 try{
  std::ifstream in(argv[1]);unsigned width,height;double r,ox,oy;in>>width>>height>>r>>ox>>oy;
  auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("arrival_planner");
  auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("arrival_costmap");
  cm->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[0.275,0.275],[0.275,-0.275],[-0.275,-0.275],[-0.275,0.275]]"),rclcpp::Parameter("footprint_padding",0.)});
  if(cm->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)throw std::runtime_error("Costmap setup failed");
  auto map=cm->getCostmap();map->resizeMap(width,height,r,ox,oy);
  for(unsigned i=0;i<width*height;++i){int c;if(!(in>>c))throw std::runtime_error("Incomplete replay map");map->getCharMap()[i]=c;}
  auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  auto planner=loader.createSharedInstance(argv[3]);planner->configure(parent,"GridBased",tf,cm);planner->activate();
  std::ofstream actual(std::string(argv[2])+".actual.json");actual<<"{\"smooth_path\":"<<(parent->get_parameter("GridBased.smooth_path").as_bool()?"true":"false")<<",\"radius\":"<<(cm->getUseRadius()?"true":"false")<<",\"tolerance\":"<<parent->get_parameter("GridBased.tolerance").as_double()<<",\"footprint\":[";
  bool first=true;for(const auto &p:cm->getRobotFootprint()){if(!first)actual<<",";first=false;actual<<"["<<p.x<<","<<p.y<<"]";}actual<<"]}";actual.close();
  geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="fixture_world";start.header.stamp=parent->now();goal.header.stamp=parent->now()-rclcpp::Duration::from_seconds(5.);
  start.pose.position.x=std::stod(argv[4]);start.pose.position.y=std::stod(argv[5]);start.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(std::stod(argv[6]));
  goal.pose.position.x=std::stod(argv[7]);goal.pose.position.y=std::stod(argv[8]);goal.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(std::stod(argv[9]));
  if(std::getenv("P2_ARRIVAL_FINAL_REQUEST")) {
    const double fx=parent->get_parameter("GridBased.final_x").as_double(),fy=parent->get_parameter("GridBased.final_y").as_double();
    // A real request header is used, rather than synthesizing a pose timestamp.
    if(std::hypot(goal.pose.position.x-fx,goal.pose.position.y-fy)<1e-6)
      parent->set_parameter(rclcpp::Parameter("GridBased.final_request_ns",rclcpp::Time(goal.header.stamp).nanoseconds()));
  }
  const auto began=std::chrono::steady_clock::now();
  try{
   auto path=planner->createPlan(start,goal);std::ofstream out(std::string(argv[2])+".csv");out<<"x,y,yaw\n";
   for(const auto &p:path.poses)out<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';
   std::ofstream(std::string(argv[2])+".json")<<std::setprecision(17)<<"{\"planned\":"<<(!path.poses.empty()?"true":"false")<<",\"points\":"<<path.poses.size()<<",\"input_goal_stamp_s\":"<<rclcpp::Time(goal.header.stamp).seconds()<<",\"path_stamp_s\":"<<rclcpp::Time(path.header.stamp).seconds()<<",\"path_age_at_result_s\":"<<(parent->now()-rclcpp::Time(path.header.stamp)).seconds()<<",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<"}";
  }catch(const std::exception &e){std::string err=e.what();for(auto &c:err)if(c=='\"'||c=='\\'||c=='\n')c=' ';std::ofstream(std::string(argv[2])+".json")<<std::setprecision(17)<<"{\"planned\":false,\"error\":\""<<err<<"\",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<"}";}
  if(std::getenv("P2_ARRIVAL_REUSE_NARROW")) {
    auto from=start;from.pose.position.x+=.2;from.pose.position.y+=.005;
    auto reused=planner->createPlan(from,goal);
    if(reused.poses.empty())throw std::runtime_error("Original footprint route was not reused");
    std::ofstream narrow(std::string(argv[2])+"-reused.csv");narrow<<"x,y,yaw\n";
    for(const auto &p:reused.poses)narrow<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';
    narrow.close();unsigned marked=0;
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){double wx,wy;map->mapToWorld(x,y,wx,wy);if(std::abs(wx-4.)<.08&&wy>1.2&&wy<1.8){map->setCost(x,y,254);++marked;}}
    bool rejected=false;
    try{auto blocked=planner->createPlan(from,goal);rejected=blocked.poses.empty();}catch(const nav2_core::PlannerException &){rejected=true;}
    if(!marked||!rejected)throw std::runtime_error("New corridor obstacle was not rejected");
    std::ofstream(std::string(argv[2])+"-blocked.json")<<"{\"rejected\":"<<(rejected?"true":"false")<<",\"marked_cells\":"<<marked<<"}";
  }
  if(std::getenv("P2_ARRIVAL_REPLAN_SEQUENCE")) {
    auto save_path=[&](const std::string &label,const geometry_msgs::msg::PoseStamped &from,const geometry_msgs::msg::PoseStamped &to){
      auto path=planner->createPlan(from,to);if(path.poses.empty())throw std::runtime_error("Replan empty: "+label);
      std::ofstream out(std::string(argv[2])+"-"+label+".csv");out<<"x,y,yaw\n";
      for(const auto &p:path.poses)out<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';
      if((parent->now()-rclcpp::Time(path.header.stamp)).seconds()>.5)throw std::runtime_error("Replan timestamp stale");return path;};
    auto initial=save_path("initial",start,goal);
    start.pose.position.x=2.2;start.pose.position.y=2.48;start.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(-.1);
    save_path("perturbed",start,goal);
    bool lower=false;for(const auto &p:initial.poses)if(std::abs(p.pose.position.x-3.5)<.1&&p.pose.position.y<2.)lower=true;
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){double wx,wy;map->mapToWorld(x,y,wx,wy);if(wx>=3.1&&wx<=4.3&&(lower?wy<2.:wy>3.))map->setCost(x,y,254);}
    save_path("blocked",start,goal);
    std::fill(map->getCharMap(),map->getCharMap()+width*height,0);
    save_path("opened",start,goal);
    goal.pose.position.x=2.2;goal.pose.position.y=5.5;
    save_path("newgoal",start,goal);
  }
  if(std::getenv("P2_ARRIVAL_ALTERNATING_SEQUENCE")) {
    const std::vector<unsigned char> original(map->getCharMap(),map->getCharMap()+width*height);
    auto save=[&](const std::string &name,const geometry_msgs::msg::PoseStamped &from,const geometry_msgs::msg::PoseStamped &to){
      auto path=planner->createPlan(from,to);if(path.poses.empty())throw std::runtime_error("Alternating replan empty");
      std::ofstream out(std::string(argv[2])+"-"+name+".csv");out<<"x,y,yaw\n";
      for(const auto &p:path.poses)out<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';};
    // First observe one side closed, then reopen it while the existing upper
    // detour remains valid. Another route leg must not evict that detour.
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){double wx,wy;map->mapToWorld(x,y,wx,wy);if(wx>=3.1&&wx<=4.3&&wy<2.)map->setCost(x,y,254);}
    save("alternating-initial",start,goal);
    std::copy(original.begin(),original.end(),map->getCharMap());
    auto other=goal;other.pose.position.x=2.2;other.pose.position.y=5.5;save("alternating-other",start,other);
    auto from=start;from.pose.position.x=2.2;from.pose.position.y=2.48;save("alternating-reused",from,goal);
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){double wx,wy;map->mapToWorld(x,y,wx,wy);if(wx>=3.1&&wx<=4.3&&wy>3.)map->setCost(x,y,254);}
    save("alternating-blocked",from,goal);
  }
  if(std::getenv("P2_ARRIVAL_CORRIDOR_SEQUENCE")) {
    const std::vector<unsigned char> original(map->getCharMap(),map->getCharMap()+width*height);
    auto save=[&](const std::string &label,const geometry_msgs::msg::PoseStamped &from,const geometry_msgs::msg::PoseStamped &to) {
      auto path=planner->createPlan(from,to);if(path.poses.empty())throw std::runtime_error("Empty reference area plan");
      std::ofstream out(std::string(argv[2])+"-"+label+".csv");out<<"x,y,yaw\n";
      for(const auto &p:path.poses)out<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';
      return path;
    };
    auto chosen=save("area-initial",start,goal);
    auto other=goal;other.pose.position.x=1.;other.pose.position.y=3.;save("area-other",start,other);
    auto from=start;from.pose.position.x+=.1;from.pose.position.y+=.01;
    auto reused=save("area-reused",from,goal);
    if(std::hypot(chosen.poses.back().pose.position.x-reused.poses.back().pose.position.x,
                  chosen.poses.back().pose.position.y-reused.poses.back().pose.position.y)>1e-6)
      throw std::runtime_error("Selected reference area flapped between route legs");
    const auto centre=reused.poses.back().pose.position;unsigned ix,iy;
    if(!map->worldToMap(centre.x,centre.y,ix,iy))throw std::runtime_error("Selected area outside map");
    map->setCost(ix,iy,254);save("area-new-obstacle",from,goal);
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){double wx,wy;map->mapToWorld(x,y,wx,wy);if(std::hypot(wx-goal.pose.position.x,wy-goal.pose.position.y)<1.)map->setCost(x,y,254);}
    bool rejected=false;try{auto path=planner->createPlan(from,goal);rejected=path.poses.empty();}catch(const nav2_core::PlannerException &){rejected=true;}
    if(!rejected)throw std::runtime_error("Completely occupied reference area accepted");
    std::copy(original.begin(),original.end(),map->getCharMap());
    // Same coordinates, a genuine new final request: now they must remain exact.
    auto final=goal;final.header.stamp=parent->now();
    parent->set_parameters({rclcpp::Parameter("GridBased.final_x",goal.pose.position.x),rclcpp::Parameter("GridBased.final_y",goal.pose.position.y),
      rclcpp::Parameter("GridBased.final_request_ns",rclcpp::Time(final.header.stamp).nanoseconds())});
    bool final_rejected=false;try{auto path=planner->createPlan(from,final);final_rejected=path.poses.empty();}catch(const nav2_core::PlannerException &){final_rejected=true;}
    if(!final_rejected)throw std::runtime_error("Occupied final visit was moved into a reference area");
    std::ofstream(std::string(argv[2])+"-area-receipts.json")<<"{\"blocked_area_rejected\":true,\"occupied_final_rejected\":true,\"actual_new_obstacle\":["<<centre.x<<","<<centre.y<<"]}";
  }
  planner->deactivate();planner->cleanup();planner.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();parent.reset();
 }catch(const std::exception &e){std::cerr<<e.what();rc=1;}
 rclcpp::shutdown();return rc;
}
