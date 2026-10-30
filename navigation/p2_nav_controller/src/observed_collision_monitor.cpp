// Copyright 2026 MID360S navigation contributors. SPDX-License-Identifier: MIT
// Observe the actual Humble collision decision after its unmodified process().
// Do not infer STOP from a zero command or duplicate the obstacle algorithm.
#include <memory>
#include "nav2_collision_monitor/collision_monitor_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"

class ObservedCollisionMonitor : public nav2_collision_monitor::CollisionMonitor {
  using State = nav2_msgs::msg::CollisionMonitorState;
  rclcpp_lifecycle::LifecyclePublisher<State>::SharedPtr state_;
public:
  explicit ObservedCollisionMonitor(const rclcpp::NodeOptions &options)
  : CollisionMonitor(options) {}
protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &s) override {
    const auto result=CollisionMonitor::on_configure(s);
    if(result!=nav2_util::CallbackReturn::SUCCESS)return result;
    nav2_util::declare_parameter_if_not_declared(shared_from_this(),"state_topic",
      rclcpp::ParameterValue("/p2_nav/collision_state"));
    state_=create_publisher<State>(get_parameter("state_topic").as_string(),1);
    cmd_vel_in_sub_=create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_in_topic").as_string(),1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg){
        if(!process_active_ || !nav2_util::validateTwist(*msg))return;
        process({msg->linear.x,msg->linear.y,msg->angular.z});
        State out;
        using namespace nav2_collision_monitor;
        switch(robot_action_prev_.action_type){
          case STOP:out.action_type=State::STOP;break;
          case SLOWDOWN:out.action_type=State::SLOWDOWN;break;
          case APPROACH:out.action_type=State::APPROACH;break;
          default:out.action_type=State::DO_NOTHING;
        }
        // Humble does not retain the chosen polygon in accessible state.
        // Leave its name empty instead of inventing an attribution.
        state_->publish(out);
      });
    return result;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &s) override {
    state_->on_activate();return CollisionMonitor::on_activate(s);
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &s) override {
    const auto result=CollisionMonitor::on_deactivate(s);state_->on_deactivate();return result;
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &s) override {
    const auto result=CollisionMonitor::on_cleanup(s);state_.reset();return result;
  }
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<ObservedCollisionMonitor>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();return 0;
}
