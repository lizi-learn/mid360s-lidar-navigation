#include <cmath>
#include <stdexcept>
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"

namespace p2_nav_controller {
// Humble's translational checker otherwise times out a stationary-XY turn.
// Keep its movement radius, timeout, reset and dynamic-parameter behavior.
// Only an actual measured yaw displacement can refresh the rotation baseline.
class RotationProgressChecker : public nav2_controller::SimpleProgressChecker {
 public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  const std::string &name) override {
    nav2_controller::SimpleProgressChecker::initialize(parent,name);
    auto node=parent.lock();
    nav2_util::declare_parameter_if_not_declared(node,name+".required_movement_angle",rclcpp::ParameterValue(.1));
    node->get_parameter(name+".required_movement_angle",angle_);
    if(!std::isfinite(angle_)||angle_<=0.||angle_>std::acos(-1.))
      throw std::invalid_argument("Invalid angular progress threshold");
  }
  bool check(geometry_msgs::msg::PoseStamped &current) override {
    const double yaw=tf2::getYaw(current.pose.orientation);
    if(!std::isfinite(yaw))return false;
    if(baseline_pose_set_&&std::abs(std::remainder(yaw-baseline_pose_.theta,2*std::acos(-1.)))>=angle_) {
      geometry_msgs::msg::Pose2D p;
      p.x=current.pose.position.x;p.y=current.pose.position.y;p.theta=yaw;
      resetBaselinePose(p);
    }
    return nav2_controller::SimpleProgressChecker::check(current);
  }
 private:
  double angle_=.1;
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::RotationProgressChecker,nav2_core::ProgressChecker)
