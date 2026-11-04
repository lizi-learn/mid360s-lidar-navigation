#include <atomic>
#include <fstream>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
using Action=nav2_msgs::action::ComputePathThroughPoses;
using Handle=rclcpp_action::ServerGoalHandle<Action>;
class Following:public BT::ActionNodeBase {
 public:using BT::ActionNodeBase::ActionNodeBase;
 static BT::PortsList providedPorts(){return {BT::InputPort<nav_msgs::msg::Path>("path")};}
 BT::NodeStatus tick()override{return BT::NodeStatus::RUNNING;}
 void halt()override{setStatus(BT::NodeStatus::IDLE);}
};
int main(int argc,char**argv){
 if(argc<3)return 2;rclcpp::init(argc,argv);int rc=0;
 auto server_node=rclcpp::Node::make_shared("reference_planner_server");
 std::mutex mutex;std::vector<size_t> requests;std::vector<std::thread> workers;std::atomic<int> cancelled{0};
 auto server=rclcpp_action::create_server<Action>(server_node,"compute_path_through_poses",
  [&](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal> g){std::lock_guard<std::mutex>lock(mutex);requests.push_back(g->goals.size());return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
  [](const std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},
  [&](const std::shared_ptr<Handle> h){workers.emplace_back([&,h]{
    auto began=std::chrono::steady_clock::now();auto result=std::make_shared<Action::Result>();
    while(rclcpp::ok()&&std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<1.2){
      if(h->is_canceling()){++cancelled;h->canceled(result);return;}std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    if(h->is_canceling()){++cancelled;h->canceled(result);return;}
    result->path.header.frame_id="map";result->path.header.stamp=server_node->now();result->path.poses=h->get_goal()->goals;h->succeed(result);
  });});
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(server_node);std::thread spinner([&]{executor.spin();});
 try{
 auto node=rclcpp::Node::make_shared("reference_action_fixture");node->declare_parameter("transform_tolerance",.1);auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());tf->setUsingDedicatedThread(true);
 using Goals=std::vector<geometry_msgs::msg::PoseStamped>;auto goal=[](double x){geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.orientation.w=1.;return p;};
 auto board=BT::Blackboard::create();board->set("node",node);board->set("tf_buffer",tf);board->set("goals",Goals{goal(2.),goal(4.)});board->set("path",nav_msgs::msg::Path{});board->set("bt_loop_duration",std::chrono::milliseconds(10));board->set("server_timeout",std::chrono::milliseconds(300));board->set("wait_for_service_timeout",std::chrono::milliseconds(2000));
 std::ifstream in(argv[2]);std::stringstream text;text<<in.rdbuf();auto s=text.str();auto begin=s.find("<PipelineSequence"),end=s.find("</PipelineSequence>");s=s.substr(begin,end+std::string("</PipelineSequence>").size()-begin);s=std::regex_replace(s,std::regex("<FollowPath[^>]*/>"),"<Following path='{path}'/>");
 BT::BehaviorTreeFactory factory;factory.registerFromPlugin(argv[1]);
 for(const auto &name:{"remove_passed_goals_action","pipeline_sequence","rate_controller"})factory.registerFromPlugin("/opt/ros/humble/lib/libnav2_"+std::string(name)+"_bt_node.so");factory.registerNodeType<Following>("Following");
 auto tree=factory.createTreeFromText("<root main_tree_to_execute='Main'><BehaviorTree ID='Main'>"+s+"</BehaviorTree></root>",board);
 auto began=std::chrono::steady_clock::now();bool stale=false,complete=false;
 while(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<4.){
  double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();geometry_msgs::msg::TransformStamped transform;transform.header.frame_id="map";transform.child_frame_id="base_link";transform.header.stamp=node->now();transform.transform.rotation.w=1.;transform.transform.translation.x=std::min(3.5,2.5*t);tf->setTransform(transform,"synthetic_fixture",true);tree.tickRoot();
  nav_msgs::msg::Path path;if(board->get("path",path)&&!path.poses.empty()){
   if(board->get<Goals>("goals").size()==1&&path.poses.front().pose.position.x==2.)stale=true;
   if(path.poses.size()==1&&path.poses.back().pose.position.x==4.){complete=true;break;}}
  std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 tree.haltTree();bool old=false,newer=false;{std::lock_guard<std::mutex>lock(mutex);for(size_t n:requests){old|=n==2;newer|=n==1;}}
 if(!complete||stale||!old||!newer||cancelled.load()<1)throw std::runtime_error("In-flight old-reference result was not discarded correctly");
 std::cout<<"{\"passed\":true,\"actual_ros_action_and_bt_plugins\":true,\"old_request_cancelled\":"<<cancelled.load()<<",\"new_prefix_applied\":true,\"stale_path_published\":false,\"physical_commands\":0}\n";
 }catch(const std::exception&e){std::cerr<<e.what();rc=1;}
 executor.cancel();spinner.join();for(auto &w:workers)w.join();rclcpp::shutdown();return rc;
}
