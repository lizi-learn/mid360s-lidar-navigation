#include <cmath>
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"
#include "p2_nav_controller/settled_arrival.hpp"
#include "p2_nav_controller/reference_goal.hpp"
namespace p2_nav_controller {
class SettledHeadingGoalChecker : public nav2_core::GoalChecker, public ReferenceGoal {
 double xy_=.15,yaw_=.15;SettledArrival arrival_,before_reset_;rclcpp::Clock::SharedPtr clock_;
 geometry_msgs::msg::PoseStamped reference_;bool have_reference_=false,reset_pending_=false;
 public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override {
  auto node=parent.lock();clock_=node->get_clock();
  nav2_util::declare_parameter_if_not_declared(node,name+".xy_goal_tolerance",rclcpp::ParameterValue(.15));
  nav2_util::declare_parameter_if_not_declared(node,name+".yaw_goal_tolerance",rclcpp::ParameterValue(.15));
  node->get_parameter(name+".xy_goal_tolerance",xy_);node->get_parameter(name+".yaw_goal_tolerance",yaw_);
  if(!std::isfinite(xy_)||!std::isfinite(yaw_)||xy_<=0.||yaw_<=0.)throw std::invalid_argument("Invalid heading goal tolerance");
 }
 void reset()override {
  // Humble calls reset for each replacement path, including the same goal.
  // Retain a copy so our controller can identify that case before the next
  // observation. Other controllers keep the normal immediate reset behavior.
  if(!reset_pending_)before_reset_=arrival_;
  arrival_.reset();reset_pending_=true;
 }
 void reference_goal(const geometry_msgs::msg::PoseStamped &goal)override {
  const bool same=have_reference_&&goal.header.frame_id==reference_.header.frame_id&&
   std::hypot(goal.pose.position.x-reference_.pose.position.x,goal.pose.position.y-reference_.pose.position.y)<=1e-6&&
   std::abs(std::remainder(tf2::getYaw(goal.pose.orientation)-tf2::getYaw(reference_.pose.orientation),2.*std::acos(-1.)))<=1e-6;
  if(same&&reset_pending_)arrival_=before_reset_;
  else if(!same)arrival_.reset();
  reference_=goal;have_reference_=true;reset_pending_=false;
 }
 bool isGoalReached(const geometry_msgs::msg::Pose &a,const geometry_msgs::msg::Pose &b,const geometry_msgs::msg::Twist &v)override {
  return arrival_.step(std::hypot(a.position.x-b.position.x,a.position.y-b.position.y),std::remainder(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation),2.*std::acos(-1.)),v.linear.x,v.linear.y,v.angular.z,clock_->now().seconds(),xy_,yaw_);
 }
 bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &v)override {
  p=geometry_msgs::msg::Pose();v=geometry_msgs::msg::Twist();p.position.x=p.position.y=xy_;p.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw_);v.linear.x=v.linear.y=v.angular.z=.03;return true;
 }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledHeadingGoalChecker,nav2_core::GoalChecker)
