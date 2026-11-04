#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "p2_nav_controller/reference_goal.hpp"

int main(int argc,char **argv) {
 const bool negative=argc>1&&std::string(argv[1])=="--expect-baseline-failure";
 rclcpp::init(argc,argv);int rc=0;
 try {
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("settled_reset_fixture");
  auto clock=node->get_clock();
  if(rcl_enable_ros_time_override(clock->get_clock_handle())!=RCL_RET_OK)throw std::runtime_error("Fixture clock setup failed");
  auto at=[&](double t){if(rcl_set_ros_time_override(clock->get_clock_handle(),static_cast<int64_t>((10.+t)*1e9))!=RCL_RET_OK)throw std::runtime_error("Fixture clock advance failed");};
  at(0.);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core","nav2_core::GoalChecker");
  auto checker=loader.createSharedInstance("p2_nav_controller::SettledHeadingGoalChecker");
  checker->initialize(node,"goal_checker",nullptr);
  auto *reference=dynamic_cast<p2_nav_controller::ReferenceGoal*>(checker.get());
  geometry_msgs::msg::PoseStamped requested;requested.header.frame_id="fixture_map";
  requested.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(0.);
  geometry_msgs::msg::Pose body=requested.pose,local_goal=requested.pose;
  geometry_msgs::msg::Twist stopped;
  bool reached=false;double accepted_at=-1.;
  for(int i=0;i<=55;++i) {
   at(i*.05);
   if(i%20==0)checker->reset();
   if(reference)reference->reference_goal(requested);
   if(checker->isGoalReached(body,local_goal,stopped)){reached=true;accepted_at=i*.05;break;}
  }
  if(negative) {
   if(reached)throw std::runtime_error("Original reset defect not reproduced");
   std::cout<<"{\"passed\":true,\"expected_original_failure\":true,\"accepted_with_1Hz_replans\":false,\"hardware_commands\":0}\n";
  } else {
   if(!reference||!reached||accepted_at<2.)throw std::runtime_error("Same goal replan did not retain two-second evidence");
   requested.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(.1);
   local_goal=requested.pose;
   bool changed_goal_early=false,changed_goal_reached=false;
   for(int i=0;i<=45;++i) {
    at(3.+i*.05);
    if(i%20==0)checker->reset();
    reference->reference_goal(requested);
    const bool yes=checker->isGoalReached(body,local_goal,stopped);
    if(i<40&&yes)changed_goal_early=true;
    changed_goal_reached=changed_goal_reached||yes;
   }
   if(changed_goal_early||!changed_goal_reached)throw std::runtime_error("Changed goal did not reset its quiet evidence");
   geometry_msgs::msg::Twist moving;moving.angular.z=.1;at(6.);
   if(checker->isGoalReached(body,local_goal,moving))throw std::runtime_error("Moving robot accepted");
   bool early=false,settled=false;
   for(int i=0;i<=45;++i){at(6.05+i*.05);if(i%20==0)checker->reset();reference->reference_goal(requested);const bool yes=checker->isGoalReached(body,local_goal,stopped);if(i<40&&yes)early=true;settled=settled||yes;}
   if(early||!settled)throw std::runtime_error("Motion did not reset the full quiet period");
   at(9.);checker->reset();reference->reference_goal(requested);
   if(checker->isGoalReached(body,local_goal,stopped))throw std::runtime_error("Missing observation gap accepted stale quiet evidence");
   std::cout<<"{\"passed\":true,\"actual_goal_checker_plugin\":true,\"same_goal_replan_accepted_at_s\":"<<accepted_at<<",\"changed_goal_resets\":true,\"motion_resets\":true,\"control_gap_resets\":true,\"original_hold_seconds\":2,\"hardware_commands\":0}\n";
  }
  checker.reset();node.reset();
 } catch(const std::exception &error){std::cerr<<error.what()<<'\n';rc=1;}
 rclcpp::shutdown();return rc;
}
