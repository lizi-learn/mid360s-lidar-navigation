// Real plugins, synthetic poses only. No action, device or motor interfaces.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"
int main(int argc,char **argv){
 rclcpp::init(argc,argv);int rc=0;
 try{
  auto n=std::make_shared<rclcpp_lifecycle::LifecycleNode>("preview_controller");
  auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("presentation_costmap");
  cm->set_parameters({rclcpp::Parameter("global_frame","fixture"),rclcpp::Parameter("robot_base_frame","base"),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]"),rclcpp::Parameter("footprint_padding",0.)});
  cm->on_configure(rclcpp_lifecycle::State());cm->getCostmap()->resizeMap(100,100,.05,-2.5,-2.5);std::fill(cm->getCostmap()->getCharMap(),cm->getCostmap()->getCharMap()+10000,0);
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");auto checker=goals.createSharedInstance("nav2_controller::SimpleGoalChecker");checker->initialize(n,"presentation_goal_checker",cm);
  geometry_msgs::msg::Pose tol;geometry_msgs::msg::Twist vt;checker->getTolerances(tol,vt);const double yaw_tol=tf2::getYaw(tol.orientation);
  if(std::abs(tol.position.x-.30)>1e-9||std::abs(yaw_tol-20.*std::acos(-1.)/180.)>1e-9)throw std::runtime_error("Actual presentation parameters differ");
  auto tf=std::make_shared<tf2_ros::Buffer>(n->get_clock());pluginlib::ClassLoader<nav2_core::Controller> controls("nav2_core","nav2_core::Controller");auto controller=controls.createSharedInstance("p2_nav_controller::SettledGoalRPP");controller->configure(n,"FollowPath",tf,cm);controller->activate();
  geometry_msgs::msg::PoseStamped target;target.header.frame_id="fixture";target.header.stamp=n->now();target.pose.orientation.w=1.;nav_msgs::msg::Path path;path.header=target.header;path.poses.push_back(target);
  geometry_msgs::msg::Twist speed;
  for(double degrees:{-19.9,-15.,0.,15.,19.9}){
   checker->reset();controller->setPlan(path);auto pose=target;pose.pose.position.x=.20;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(degrees*std::acos(-1.)/180.);
   auto command=controller->computeVelocityCommands(pose,speed,checker.get()).twist;
   if(std::abs(command.linear.x)+std::abs(command.linear.y)+std::abs(command.angular.z)>1e-9)throw std::runtime_error("Fine adjustment continued inside20deg");
   if(!checker->isGoalReached(pose.pose,target.pose,speed))throw std::runtime_error("Approximate arrival rejected");
  }
  auto pose=target;pose.pose.position.x=.2;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(20.1*std::acos(-1.)/180.);checker->reset();if(checker->isGoalReached(pose.pose,target.pose,speed))throw std::runtime_error("Outside20deg accepted");
  pose.pose=target.pose;pose.pose.position.x=.301;checker->reset();if(checker->isGoalReached(pose.pose,target.pose,speed))throw std::runtime_error("Outside30cm accepted before acquisition");
  auto changes=n->set_parameters({rclcpp::Parameter("presentation_goal_checker.yaw_goal_tolerance",10.*std::acos(-1.)/180.)});if(!changes[0].successful)throw std::runtime_error("Live parameter update rejected");checker->getTolerances(tol,vt);if(std::abs(tf2::getYaw(tol.orientation)-10.*std::acos(-1.)/180.)>1e-9)throw std::runtime_error("Live tolerance is only a stored label");
  pose.pose.position.x=.2;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(15.*std::acos(-1.)/180.);checker->reset();if(checker->isGoalReached(pose.pose,target.pose,speed))throw std::runtime_error("Updated10deg ignored");
  n->set_parameters({rclcpp::Parameter("presentation_goal_checker.yaw_goal_tolerance",yaw_tol)});checker->reset();if(!checker->isGoalReached(pose.pose,target.pose,speed))throw std::runtime_error("Restored20deg ignored");
  std::cout<<"{\"passed\":true,\"actual_goal_checker\":\"nav2_controller::SimpleGoalChecker\",\"actual_controller\":\"p2_nav_controller::SettledGoalRPP\",\"xy_tolerance_m\":0.3,\"yaw_tolerance_deg\":20,\"inside_cases_zero_adjustment\":5,\"outside_boundary_checks\":2,\"actual_live_parameter_checks\":2,\"physical_commands\":0}\n";
  controller->deactivate();controller->cleanup();controller.reset();checker.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();n.reset();
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';rc=1;}
 rclcpp::shutdown();return rc;
}
