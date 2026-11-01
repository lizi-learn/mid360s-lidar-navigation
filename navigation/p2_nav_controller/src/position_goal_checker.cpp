#include <cmath>
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace p2_nav_controller {
class PositionGoalChecker : public nav2_core::GoalChecker {
 double tolerance_=.15;
 public:
 void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {
  auto node=parent.lock();nav2_util::declare_parameter_if_not_declared(node,name+".xy_goal_tolerance",rclcpp::ParameterValue(.15));node->get_parameter(name+".xy_goal_tolerance",tolerance_);
  if(!std::isfinite(tolerance_)||tolerance_<=0.)throw std::invalid_argument("Invalid position tolerance");
 }
 void reset() override {}
 bool isGoalReached(const geometry_msgs::msg::Pose &a,const geometry_msgs::msg::Pose &b,const geometry_msgs::msg::Twist &) override {return std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)<=tolerance_;}
 bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist &v) override {p=geometry_msgs::msg::Pose();v=geometry_msgs::msg::Twist();p.position.x=p.position.y=tolerance_;p.orientation=nav2_util::geometry_utils::orientationAroundZAxis(std::acos(-1.));return true;}
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::PositionGoalChecker,nav2_core::GoalChecker)
