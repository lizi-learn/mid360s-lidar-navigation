// Actual Nav2 pipeline/recovery plugins, scripted planner responses only.
#include <chrono>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
struct Counts{int plans=0,follows=0,halts=0;bool persistent=false;};
Counts counts;
class Planner:public BT::SyncActionNode{
 public:using BT::SyncActionNode::SyncActionNode;
 static BT::PortsList providedPorts(){return {};}
 BT::NodeStatus tick()override{++counts.plans;return counts.plans>1&&(counts.persistent||counts.plans==2)?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;}
};
class Following:public BT::ActionNodeBase{
 public:using BT::ActionNodeBase::ActionNodeBase;
 static BT::PortsList providedPorts(){return {};}
 BT::NodeStatus tick()override{++counts.follows;return BT::NodeStatus::RUNNING;}
 void halt()override{if(status()==BT::NodeStatus::RUNNING)++counts.halts;setStatus(BT::NodeStatus::IDLE);}
};
std::string pipeline(const char*file){
 std::ifstream input(file);std::stringstream text;text<<input.rdbuf();auto s=text.str();
 const auto a=s.find("<PipelineSequence"),b=s.find("</PipelineSequence>");
 if(a==std::string::npos||b==std::string::npos)throw std::runtime_error("actual generated pipeline missing");
 s=s.substr(a,b+std::string("</PipelineSequence>").size()-a);
 s=std::regex_replace(s,std::regex("<ComputePathToPose[^>]*/>"),"<ScriptedPlanner/>");
 return std::regex_replace(s,std::regex("<FollowPath[^>]*/>"),"<Following/>");
}
int main(int argc,char**argv){
 if(argc<3)return 2;rclcpp::init(argc,argv);int rc=0;
 try{
  auto node=std::make_shared<rclcpp::Node>("path_refresh_retry_fixture");
  BT::BehaviorTreeFactory f;
  for(const auto&lib:{"pipeline_sequence","rate_controller","recovery_node"})f.registerFromPlugin(std::string("/opt/ros/humble/lib/libnav2_")+lib+"_bt_node.so");
  f.registerNodeType<Planner>("ScriptedPlanner");f.registerNodeType<Following>("Following");
  auto run=[&](const char*file,bool persistent){
   counts=Counts{};counts.persistent=persistent;auto board=BT::Blackboard::create();board->set("node",node);
   auto tree=f.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"+pipeline(file)+"</BehaviorTree></root>",board);
   const auto start=std::chrono::steady_clock::now();
   while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<1.8){tree.tickRoot();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
   const auto result=counts;tree.haltTree();return result;
  };
  const auto baseline=run(argv[1],false),candidate=run(argv[2],false),persistent=run(argv[2],true);
  if(baseline.halts==0||candidate.halts!=0||candidate.plans<3||persistent.halts==0)throw std::runtime_error("Planner failure did not retain/withdraw the real pipeline as expected");
  std::cout<<"{\"passed\":true,\"baseline_controller_halts\":"<<baseline.halts<<",\"transient_candidate_controller_halts\":"<<candidate.halts<<",\"candidate_plan_attempts\":"<<candidate.plans<<",\"persistent_failure_controller_halts\":"<<persistent.halts<<",\"actual_nav2_pipeline_and_recovery\":true,\"rate_hz\":1,\"hardware_commands\":0}"<<std::endl;
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rc=1;}
 rclcpp::shutdown();return rc;
}
