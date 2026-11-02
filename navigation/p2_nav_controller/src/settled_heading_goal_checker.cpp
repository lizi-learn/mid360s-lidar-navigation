#include <cmath>
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"
#include "p2_nav_controller/settled_arrival.hpp"
namespace p2_nav_controller {
class SettledHeadingGoalChecker : public nav2_core::GoalChecker {
 double xy_=.15,yaw_=.15;SettledArrival arrival_;rclcpp::Clock::SharedPtr clock_;
 public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override {
  auto node=parent.lock();clock_=node->get_clock();
  nav2_util::declare_parameter_if_not_declared(node,name+".xy_goal_tolerance",rclcpp::ParameterValue(.15));
  nav2_util::declare_parameter_if_not_declared(node,name+".yaw_goal_tolerance",rclcpp::ParameterValue(.15));
  node->get_parameter(name+".xy_goal_tolerance",xy_);node->get_parameter(name+".yaw_goal_tolerance",yaw_);
  if(!std::isfinite(xy_)||!std::isfinite(yaw_)||xy_<=0.||yaw_<=0.)throw std::invalid_argument("Invalid heading goal tolerance");
 }
 void reset()override{arrival_.reset();}
 bool isGoalReached(const geometry_msgs::msg::Pose &a,const geometry_msgs::msg::Pose &b,const geometry_msgs::msg::Twist &v)override {
  return arrival_.step(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y),std::remainder(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation),2.*std::acos(-1.)),v.linear.x,v.linear.y,v.angular.z,clock_->now().seconds(),xy_,yaw_);
 }
 bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &v)override {
  p=geometry_msgs::msg::Pose();v=geometry_msgs::msg::Twist();p.position.x=p.position.y=xy_;p.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw_);v.linear.x=v.linear.y=v.angular.z=.03;return true;
 }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledHeadingGoalChecker,nav2_core::GoalChecker)
