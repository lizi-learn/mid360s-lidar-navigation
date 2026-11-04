// Actual pluginlib and Humble comparison with an explicit test ROS clock.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl/time.h"
#include "nav2_core/progress_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_loader.hpp"

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);int rc=0;
  try {
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("progress_fixture");
    for(const std::string p:{"old","rotation"}) {
      node->declare_parameter(p+".required_movement_radius",.1);
      node->declare_parameter(p+".movement_time_allowance",40.);
    }
    pluginlib::ClassLoader<nav2_core::ProgressChecker> loader("nav2_core","nav2_core::ProgressChecker");
    auto old=loader.createSharedInstance("nav2_controller::SimpleProgressChecker");
    auto rotation=loader.createSharedInstance("p2_nav_controller::RotationProgressChecker");
    old->initialize(node,"old");rotation->initialize(node,"rotation");
    auto clock=node->get_clock()->get_clock_handle();
    if(rcl_enable_ros_time_override(clock)!=RCL_RET_OK)throw std::runtime_error("Test clock override failed");
    auto at=[&](double seconds,double x,double yaw){
      if(rcl_set_ros_time_override(clock,static_cast<int64_t>(seconds*1e9))!=RCL_RET_OK)throw std::runtime_error("Test clock advance failed");
      geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.header.stamp=node->now();
      pose.pose.position.x=x;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw);return pose;};
    bool baseline_failed=false;
    for(int i=0;i<=46;++i) {
      auto p=at(1.+i,0.,std::remainder(3.+i*.02,2*std::acos(-1.)));
      if(!old->check(p))baseline_failed=true;
      if(!rotation->check(p))throw std::runtime_error("Measured pivot falsely stalled");
    }
    if(!baseline_failed)throw std::runtime_error("Baseline timeout not reproduced");
    rotation->reset();auto p=at(100.,0.,0.);rotation->check(p);
    p=at(141.,0.,0.);if(rotation->check(p))throw std::runtime_error("No-motion timeout lost");
    rotation->reset();
    for(int i=0;i<=46;++i) {
      p=at(200.+i,0.,i%2?.025:-.025);
      bool ok=rotation->check(p);
      if(i==46&&ok)throw std::runtime_error("Small yaw jitter masked stall");
    }
    old->reset();rotation->reset();
    for(int i=0;i<=46;++i){p=at(300.+i,.02*i,0.);if(!old->check(p)||!rotation->check(p))throw std::runtime_error("Translation behavior changed");}
    rotation.reset();old.reset();node.reset();
    std::cout<<"{\"passed\":true,\"baseline_pivot_timeout_reproduced\":true,\"measured_pivot_passed\":true,\"stationary_timeout_preserved\":true,\"jitter_timeout_preserved\":true,\"translation_preserved\":true,\"timeout_s\":40,\"radius_m\":0.1,\"angle_rad\":0.1,\"clock\":\"explicit synthetic ROS test clock, not sensor stamps\",\"hardware_commands\":0}\n";
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';rc=1;}
  rclcpp::shutdown();return rc;
}
