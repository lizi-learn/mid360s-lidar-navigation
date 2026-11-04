#include <cmath>
#include <fstream>
#include <sstream>
#include <regex>
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SlowPlanner:public BT::ActionNodeBase {
 std::chrono::steady_clock::time_point started_;bool began_=false;
 public:
 SlowPlanner(const std::string &name,const BT::NodeConfiguration &conf):BT::ActionNodeBase(name,conf){}
 static BT::PortsList providedPorts(){return {};}
 BT::NodeStatus tick()override{if(!began_){began_=true;started_=std::chrono::steady_clock::now();}return std::chrono::duration<double>(std::chrono::steady_clock::now()-started_).count()<1.6?BT::NodeStatus::RUNNING:BT::NodeStatus::SUCCESS;}
 void halt()override{setStatus(BT::NodeStatus::IDLE);}
};
class KeepFollowing:public BT::ActionNodeBase {
 public:using BT::ActionNodeBase::ActionNodeBase;
 static BT::PortsList providedPorts(){return {};}
 BT::NodeStatus tick()override{return BT::NodeStatus::RUNNING;}
 void halt()override{setStatus(BT::NodeStatus::IDLE);}
};
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
 factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_pipeline_sequence_bt_node.so");
 factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_rate_controller_bt_node.so");
 factory.registerNodeType<SlowPlanner>("SlowPlanner");factory.registerNodeType<KeepFollowing>("KeepFollowing");
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
 // A1.6s planner spans the entire intermediate-point passage window.
 // Execute the shipped tree's actual pipeline/rate/removal plugins while TF
 // crosses that point, rather than inferring behavior from XML placement.
 node->declare_parameter("transform_tolerance",.1);
 if(argc<2)throw std::runtime_error("Production through-poses XML required");
 std::ifstream xml(argv[1]);std::stringstream content;content<<xml.rdbuf();auto production=content.str();
 auto begin=production.find("<PipelineSequence"),end=production.find("</PipelineSequence>");
 if(begin==std::string::npos||end==std::string::npos)throw std::runtime_error("Pipeline missing");
 auto pipeline=production.substr(begin,end+std::string("</PipelineSequence>").size()-begin);
 pipeline=std::regex_replace(pipeline,std::regex("<ComputeReferencePath[^>]*/>"),"<SlowPlanner/>");
 pipeline=std::regex_replace(pipeline,std::regex("<FollowPath[^>]*/>"),"<KeepFollowing/>");
 auto probe=[&](std::string body){
  auto board=BT::Blackboard::create();board->set("node",node);board->set("tf_buffer",tf);board->set("goals",Goals{goal(2.,0.),goal(4.,0.)});
  auto tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"+body+"</BehaviorTree></root>",board);
  auto start=std::chrono::steady_clock::now();
  while(true){double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();if(t>=1.8)break;
   transform.transform.translation.x=std::min(4.,t*2.5);transform.transform.translation.y=0.;transform.header.stamp=node->now();tf->setTransform(transform,"slow_planner_fixture",true);
   tree.tickRoot();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  tree.haltTree();return board->get<Goals>("goals").size();
 };
 const std::string remove="<RemovePassedGoals input_goals='{goals}' output_goals='{goals}' radius='0.75'/>";
 const auto old_count=probe("<PipelineSequence><RateController hz='1.0'><Sequence>"+remove+"<SlowPlanner/></Sequence></RateController><KeepFollowing/></PipelineSequence>");
 const auto new_count=probe(pipeline);
 if(old_count!=2||new_count!=1)throw std::runtime_error("Slow-planner passage regression: "+std::to_string(old_count)+" -> "+std::to_string(new_count));
 std::cout<<"{\"slow_planner_old_remaining\":2,\"slow_planner_new_remaining\":1,\"passed\":true,\"actual_nav2_bt_plugin\":true,\"hardware_io\":false,\"old_reference_count\":3,\"corrected_reference_count\":2,\"terminal_goal_retained\":true,\"reference_radius\":0.75,\"physical_goal_tolerance_changed\":false}\n";
 rclcpp::shutdown();return 0;
}
