#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace p2_nav_controller {
// The controller supplies the actual path goal in its original frame. Goal
// checker inputs alone have already been transformed into the local frame.
class ReferenceGoal {
 public:
  virtual ~ReferenceGoal() = default;
  virtual void reference_goal(const geometry_msgs::msg::PoseStamped &goal) = 0;
};
}
