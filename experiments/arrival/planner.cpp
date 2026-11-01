#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
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
  const auto began=std::chrono::steady_clock::now();
  try{
   auto path=planner->createPlan(start,goal);std::ofstream out(std::string(argv[2])+".csv");out<<"x,y,yaw\n";
   for(const auto &p:path.poses)out<<std::setprecision(12)<<p.pose.position.x<<','<<p.pose.position.y<<','<<tf2::getYaw(p.pose.orientation)<<'\n';
   std::ofstream(std::string(argv[2])+".json")<<std::setprecision(17)<<"{\"planned\":"<<(!path.poses.empty()?"true":"false")<<",\"points\":"<<path.poses.size()<<",\"input_goal_stamp_s\":"<<rclcpp::Time(goal.header.stamp).seconds()<<",\"path_stamp_s\":"<<rclcpp::Time(path.header.stamp).seconds()<<",\"path_age_at_result_s\":"<<(parent->now()-rclcpp::Time(path.header.stamp)).seconds()<<",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<"}";
  }catch(const std::exception &e){std::string err=e.what();for(auto &c:err)if(c=='\"'||c=='\\'||c=='\n')c=' ';std::ofstream(std::string(argv[2])+".json")<<std::setprecision(17)<<"{\"planned\":false,\"error\":\""<<err<<"\",\"seconds\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<"}";}
  planner->deactivate();planner->cleanup();planner.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();parent.reset();
 }catch(const std::exception &e){std::cerr<<e.what();rc=1;}
 rclcpp::shutdown();return rc;
}
