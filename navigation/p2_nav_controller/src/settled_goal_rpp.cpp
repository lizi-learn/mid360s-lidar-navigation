#include <chrono>
#include <algorithm>
#include <cmath>
#include <mutex>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "p2_nav_controller/terminal_heading.hpp"
#include "p2_nav_controller/terminal_velocity.hpp"

namespace p2_nav_controller {
class SettledGoalRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using Base=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
 public:
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (path.poses.empty()) { phase_.reset(); throw nav2_core::PlannerException("Empty terminal path"); }
    auto goal=path.poses.back(); goal.header.frame_id=path.header.frame_id;
    const auto &q=goal.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (!std::isfinite(norm) || norm<.8 || norm>1.2) { phase_.reset(); throw nav2_core::PlannerException("Invalid terminal quaternion"); }
    phase_.set_goal(goal.pose.position.x,goal.pose.position.y,tf2::getYaw(q),goal.header.frame_id);
    goal_=goal; Base::setPlan(path);
  }
  void activate() override { phase_.reset(); Base::activate(); }
  void deactivate() override { phase_.reset(); Base::deactivate(); }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    std::unique_lock<std::mutex> base_lock(mutex_);
    geometry_msgs::msg::Pose tolerance; geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker || !checker->getTolerances(tolerance,velocity_tolerance))
      throw nav2_core::PlannerException("Terminal goal tolerance unavailable");
    auto goal=goal_; goal.header.stamp=pose.header.stamp;
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(pose.header.frame_id,goal,transformed))
      throw nav2_core::PlannerException("Terminal goal transform unavailable");
    const double distance=std::hypot(transformed.pose.position.x-pose.pose.position.x,
                                    transformed.pose.position.y-pose.pose.position.y);
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const double error=std::remainder(tf2::getYaw(transformed.pose.orientation)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
    const bool aligned=std::abs(error)<std::abs(tf2::getYaw(tolerance.orientation));
    const bool rotate_final=phase_.step(distance,tolerance.position.x,now,aligned);
    if (!use_rotate_to_heading_ || (!rotate_final && distance>2.*tolerance.position.x)) {
      base_lock.unlock(); return Base::computeVelocityCommands(pose,speed,checker);
    }
    // A new lattice plan can contain a tight arrival loop to impose final yaw.
    // In the bounded terminal region, acquire XY first and then pivot. No lateral
    // or reverse velocity is generated, and the original success tolerances stay.
    const double bearing=std::remainder(std::atan2(transformed.pose.position.y-pose.pose.position.y,
      transformed.pose.position.x-pose.pose.position.x)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
    const auto command=terminal_velocity(distance,bearing,error,rotate_final,rotate_to_heading_angular_vel_);
    const double v=command[0],w=command[1];
    auto *costmap=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*(costmap->getMutex()));
    if (use_collision_detection_ && isCollisionImminent(pose,v,w,distance))
      throw nav2_core::PlannerException("Terminal rotation collision ahead");
    geometry_msgs::msg::TwistStamped out; out.header=pose.header; out.twist.linear.x=v; out.twist.angular.z=w; return out;
  }
 private:
  std::mutex phase_mutex_; TerminalHeading phase_; geometry_msgs::msg::PoseStamped goal_;
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledGoalRPP,nav2_core::Controller)
