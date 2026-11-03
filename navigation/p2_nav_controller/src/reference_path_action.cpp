#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
namespace p2_nav_controller {
// Passage may occur while planning is still running. A completed response to
// the old prefix must not publish a path that sends the robot back to it.
class ReferencePathAction:public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathThroughPoses> {
 using Base=nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathThroughPoses>;
 using Goals=std::vector<geometry_msgs::msg::PoseStamped>;
 bool progress_update_=false;
 public:
 ReferencePathAction(const std::string &name,const BT::NodeConfiguration &conf):Base(name,"compute_path_through_poses",conf){}
 static BT::PortsList providedPorts(){return providedBasicPorts({
   BT::InputPort<Goals>("goals"),BT::InputPort<std::string>("planner_id",""),
   BT::InputPort<geometry_msgs::msg::PoseStamped>("start"),BT::OutputPort<nav_msgs::msg::Path>("path")});}
 BT::NodeStatus tick()override {
   Goals current;
   if(!getInput("goals",current))throw BT::RuntimeError("Missing reference goals");
   if(status()==BT::NodeStatus::RUNNING&&current!=goal_.goals){
     progress_update_=true;Base::halt();progress_update_=false;
   }
   return Base::tick();
 }
 void on_tick()override {
   getInput("goals",goal_.goals);getInput("planner_id",goal_.planner_id);
   goal_.use_start=bool(getInput("start",goal_.start));
 }
 BT::NodeStatus on_success()override{setOutput("path",result_.result->path);return BT::NodeStatus::SUCCESS;}
 BT::NodeStatus on_aborted()override{setOutput("path",nav_msgs::msg::Path{});return BT::NodeStatus::FAILURE;}
 BT::NodeStatus on_cancelled()override {
   // Only prefix progress keeps the previously issued path while its remaining
   // references are recomputed. Ordinary cancel/halt still clears the output;
   // FollowPath, collision checks and physical manual priority remain intact.
   if(!progress_update_)setOutput("path",nav_msgs::msg::Path{});
   return BT::NodeStatus::SUCCESS;
 }
};
}
BT_REGISTER_NODES(factory){factory.registerNodeType<p2_nav_controller::ReferencePathAction>("ComputeReferencePath");}
