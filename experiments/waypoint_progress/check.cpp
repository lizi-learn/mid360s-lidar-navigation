#include <cmath>
#include <iostream>
#include <memory>
#include <vector>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("reference_waypoint_progress_fixture");
 auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
 tf2_ros::TransformListener listener(*tf,node,true);
 geometry_msgs::msg::TransformStamped transform;
 transform.header.frame_id="map";transform.child_frame_id="base_link";
 transform.header.stamp=node->now();transform.transform.rotation.w=1.;
 transform.transform.translation.x=.6;transform.transform.translation.y=.4;
 tf->setTransform(transform,"synthetic_fixture",true);
 using Goals=std::vector<geometry_msgs::msg::PoseStamped>;
 auto goal=[](double x,double y){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p;};
 BT::BehaviorTreeFactory factory;
 factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_remove_passed_goals_action_bt_node.so");
 auto evaluate=[&](double radius,Goals goals){
  auto board=BT::Blackboard::create();board->set("node",node);board->set("tf_buffer",tf);board->set("transform_tolerance",.1);board->set("goals",goals);
  auto tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'><RemovePassedGoals input_goals='{goals}' output_goals='{goals}' radius='"+std::to_string(radius)+"'/></BehaviorTree></root>",board);
  if(tree.tickRoot()!=BT::NodeStatus::SUCCESS)throw std::runtime_error("Native BT did not complete");
  return board->get<Goals>("goals");
 };
 const Goals route{goal(0.,0.),goal(4.,0.),goal(8.,0.)};
 auto old=evaluate(.30,route),corrected=evaluate(.75,route);
 auto final=evaluate(.75,{goal(.65,.4)});
 if(old.size()!=3||corrected.size()!=2||corrected.front().pose.position.x!=4.||corrected.back().pose.position.x!=8.||final.size()!=1)throw std::runtime_error("Unexpected native intermediate/final waypoint behavior");
 std::cout<<"{\"passed\":true,\"actual_nav2_bt_plugin\":true,\"hardware_io\":false,\"old_reference_count\":3,\"corrected_reference_count\":2,\"terminal_goal_retained\":true,\"reference_radius\":0.75,\"physical_goal_tolerance_changed\":false}\n";
 rclcpp::shutdown();return 0;
}
