// Actual Nav2 controller, progress checker and collision projection. Synthetic
// planar yaw plant; no sockets, hardware, vendor model or physical gait claim.
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/geometry_utils.hpp"
#include "speed_response.hpp"
constexpr double pi=3.14159265358979323846,dt=.05,half=.275;
bool overlap(double yaw,double cx,double cy,double h){
 double c=std::cos(yaw),s=std::sin(yaw);
 return std::abs(cx)<=half*(std::abs(c)+std::abs(s))+h&&std::abs(cy)<=half*(std::abs(c)+std::abs(s))+h&&
 std::abs(c*cx+s*cy)<=half+h*(std::abs(c)+std::abs(s))&&std::abs(-s*cx+c*cy)<=half+h*(std::abs(c)+std::abs(s));
}
int main(int argc,char **argv){
 if(argc<7)return 2;
 std::string prefix=argv[1],kind=argv[6];double gain=std::stod(argv[2]),delay=std::stod(argv[3]),dead=std::stod(argv[4]),goal=std::stod(argv[5]);
 const double scale=std::getenv("P2_TEST_INPUT_SCALE")?std::stod(std::getenv("P2_TEST_INPUT_SCALE")):1.;
 if(!std::isfinite(scale)||scale<=0.||scale>1.)return 2;
 rclcpp::init(argc,argv);int rc=0;
 try{
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("preview_controller");
  if(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle())!=RCL_RET_OK)throw std::runtime_error("ROS clock override failed");
  if(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),1000000000LL)!=RCL_RET_OK)throw std::runtime_error("ROS clock set failed");
  auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("pivot_costmap");
  cm->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[0.275,0.275],[0.275,-0.275],[-0.275,-0.275],[-0.275,0.275]]"),rclcpp::Parameter("footprint_padding",0.)});
  if(cm->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)throw std::runtime_error("Costmap configure failed");
  auto map=cm->getCostmap();map->resizeMap(100,100,.05,-2.5,-2.5);std::fill(map->getCharMap(),map->getCharMap()+10000,0);
  bool verified=false;double ox=0,oy=0;
  if(kind=="blocked"||kind=="endpoint") {unsigned ix,iy;map->worldToMap(.39,.02,ix,iy);map->setCost(ix,iy,254);map->mapToWorld(ix,iy,ox,oy);verified=map->getCost(ix,iy)==254;}
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  auto control=loader.createSharedInstance("p2_nav_controller::SettledGoalRPP");control->configure(node,"FollowPath",tf,cm);control->activate();
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
  if(!node->has_parameter("goal_checker.plugin"))node->declare_parameter("goal_checker.plugin",std::string("nav2_controller::SimpleGoalChecker"));
  auto checker=goals.createSharedInstance(node->get_parameter("goal_checker.plugin").as_string());checker->initialize(node,"goal_checker",cm);
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_loader("nav2_core","nav2_core::ProgressChecker");
  if(!node->has_parameter("progress_checker.plugin"))node->declare_parameter("progress_checker.plugin",std::string("nav2_controller::SimpleProgressChecker"));
  auto progress=progress_loader.createSharedInstance(node->get_parameter("progress_checker.plugin").as_string());progress->initialize(node,"progress_checker");
  geometry_msgs::msg::PoseStamped target;target.header.frame_id="fixture_world";target.header.stamp=node->now();target.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(goal);
  nav_msgs::msg::Path path;path.header=target.header;path.poses.push_back(target);control->setPlan(path);
  p2_nav::SpeedResponse native;std::deque<double> latency(static_cast<int>(std::round(delay/dt))+1,0.);
  double yaw=0,rate=0,quiet=0,time=0,max_input=0;int blocked=0,intrusions=0;bool reached=false,accepted=false,progress_failed=false,cancelled=false,post_cancel_nonzero=false;
  std::ofstream trace(prefix+".csv");trace<<"t,yaw,rate,error,raw_z,input_z,applied,collision_exception,cancelled,progress\n";
  for(int k=0;k<900;++k){
   time=k*dt;if(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),static_cast<int64_t>((1.+time)*1e9))!=RCL_RET_OK)throw std::runtime_error("Fixture clock tick failed");
   geometry_msgs::msg::PoseStamped pose;pose.header=target.header;pose.header.stamp=node->now();pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw);
   geometry_msgs::msg::Twist measured;measured.angular.z=rate;
   bool progressing=accepted||progress->check(pose);if(!progressing){progress_failed=true;break;}
   if(kind=="cancel"&&time>=3.&&!cancelled){cancelled=true;control->deactivate();}
   double raw=0;bool stopped=false;
   if(!cancelled&&!accepted)try{raw=control->computeVelocityCommands(pose,measured,checker.get()).twist.angular.z;}catch(const std::exception &e){++blocked;stopped=true;}
   auto stamp=static_cast<uint64_t>((time+1)*1e9);auto input=native.step(0,raw*scale,0,rate,dt,stamp,stamp,!cancelled&&!accepted);
   if(cancelled&&(raw!=0.||input.policy_yaw!=0.))post_cancel_nonzero=true;
   max_input=std::max(max_input,std::abs(input.policy_yaw));latency.push_back(input.policy_yaw);double applied=latency.front();latency.pop_front();
   double previous=yaw,target_rate=std::abs(applied)<dead?0.:gain*applied;
   rate+=(target_rate-rate)*(1-std::exp(-dt/.15));yaw=std::remainder(yaw+rate*dt,2*pi);
   if(verified)for(int sub=0;sub<=5;++sub)if(overlap(previous+std::remainder(yaw-previous,2*pi)*sub/5.,ox,oy,.025)){++intrusions;break;}
   const double error=std::abs(std::remainder(goal-yaw,2*pi));
   pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw);measured.angular.z=rate;
   bool goal_ok=checker->isGoalReached(pose.pose,target.pose,measured);if(goal_ok)accepted=true;
   if(accepted&&error<.15&&std::abs(rate)<.03&&raw==0.)quiet+=dt;else quiet=0.;
   trace<<std::setprecision(12)<<time<<','<<yaw<<','<<rate<<','<<error<<','<<raw<<','<<input.policy_yaw<<','<<applied<<','<<stopped<<','<<cancelled<<','<<progressing<<'\n';
   if(quiet>=2.){reached=true;break;}
   if((kind=="blocked"||kind=="cancel")&&time>=8.)break;
   if(intrusions)break;
  }
  std::ofstream out(prefix+".json");out<<std::boolalpha<<std::setprecision(12)<<"{\"goal_reached\":"<<reached<<",\"error_rad\":"<<std::abs(std::remainder(goal-yaw,2*pi))<<",\"duration_s\":"<<time<<",\"goal_accepted\":"<<accepted<<",\"goal_checker\":\""<<node->get_parameter("goal_checker.plugin").as_string()<<"\",\"progress_failed\":"<<progress_failed<<",\"body_intrusions\":"<<intrusions<<",\"obstacle_verified\":"<<verified<<",\"collision_stops\":"<<blocked<<",\"cancelled\":"<<cancelled<<",\"post_cancel_nonzero\":"<<post_cancel_nonzero<<",\"final_measured_rate\":"<<rate<<",\"operator_scale\":"<<scale<<",\"max_policy_input\":"<<max_input<<",\"settled_pivot_response\":"<<node->get_parameter("FollowPath.settled_pivot_response").as_bool()<<",\"movement_time_allowance\":"<<node->get_parameter("progress_checker.movement_time_allowance").as_double()<<",\"actual_footprint\":[";
  auto body=cm->getRobotFootprint();for(size_t i=0;i<body.size();++i){if(i)out<<',';out<<'['<<body[i].x<<','<<body[i].y<<']';}out<<"]}";
  if(!cancelled)control->deactivate();control->cleanup();control.reset();progress.reset();checker.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();node.reset();
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';rc=1;}
 rclcpp::shutdown();return rc;
}
