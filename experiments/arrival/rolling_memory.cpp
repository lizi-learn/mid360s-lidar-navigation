// Actual rolling obstacle layers and RPP, synthetic points/clock only.
// No robot endpoint, publishers, actions, motor commands or physical gait claim.
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/transform_listener.h"

void require(bool value,const char *why){if(!value)throw std::runtime_error(why);}
int main(int argc,char **argv){
 rclcpp::init(argc,argv);int rc=0;
 try{
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("preview_controller");
  auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("rolling_memory_fixture");
  cm->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("rolling_window",true),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[0.275,0.275],[0.275,-0.275],[-0.275,-0.275],[-0.275,0.275]]"),rclcpp::Parameter("footprint_padding",0.)});
  require(cm->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"costmap configure");
  auto layered=cm->getLayeredCostmap();layered->resizeMap(100,100,.05,-2.5,-2.5);
  auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener=std::make_shared<tf2_ros::TransformListener>(*tf);
  geometry_msgs::msg::TransformStamped tr;tr.header.frame_id="fixture_world";tr.child_frame_id="fixture_base";tr.transform.rotation.w=1.;require(tf->setTransform(tr,"synthetic_rolling_fixture",true),"fixture transform");
  auto group=cm->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> layers("nav2_costmap_2d","nav2_costmap_2d::Layer");
  pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core","nav2_core::Controller");
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
  auto control=controllers.createSharedInstance("p2_nav_controller::SettledGoalRPP");control->configure(node,"FollowPath",tf,cm);control->activate();
  auto goal=goals.createSharedInstance("p2_nav_controller::PositionGoalChecker");goal->initialize(node,"position_goal_checker",cm);
  nav_msgs::msg::Path path;path.header.frame_id="fixture_world";
  for(int i=0;i<=250;++i){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=i*.02;p.pose.orientation.w=1.;path.poses.push_back(p);}
  geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.pose.orientation.w=1.;
  auto command=[&](){control->setPlan(path);try{return control->computeVelocityCommands(pose,geometry_msgs::msg::Twist(),goal.get()).twist.linear.x;}catch(const std::exception&){return 0.;}};
  const auto clock=cm->get_clock()->get_clock_handle();require(rcl_enable_ros_time_override(clock)==RCL_RET_OK,"isolated source clock");
  auto at=[&](int64_t ns){require(rcl_set_ros_time_override(clock,ns)==RCL_RET_OK,"fixture time");};
  bool baseline_blocked=false,candidate_forward=false;int checks=0;
  for(const auto &type:{"nav2_costmap_2d::ObstacleLayer","p2_nav_controller::TemporalObstacleLayer"}){
   const bool candidate=std::string(type).find("Temporal")!=std::string::npos;
   const std::string name=candidate?"candidate":"baseline";cm->declare_parameter(name+".combination_method",0);
   auto plugin=layers.createSharedInstance(type);plugin->initialize(layered,name,tf.get(),cm,group);
   auto layer=std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);require(bool(layer),"real obstacle plugin");
   auto observe=[&](int64_t stamp,bool hit){
    layer->clearStaticObservations(true,true);nav2_costmap_2d::Observation obs;
    obs.cloud_->header.frame_id="fixture_world";obs.cloud_->header.stamp=rclcpp::Time(stamp);
    sensor_msgs::PointCloud2Modifier mod(*obs.cloud_);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(*obs.cloud_,"x"),y(*obs.cloud_,"y"),z(*obs.cloud_,"z");*x=hit?.8f:2.f;*y=hit?.05f:1.f;*z=.4f;
    obs.obstacle_min_range_=.12;obs.obstacle_max_range_=11.;layer->addStaticObservation(obs,true,false);
   };
   auto update=[&](double x=0.,double y=0.){double ax=1e30,ay=1e30,bx=-1e30,by=-1e30;plugin->updateBounds(x,y,0.,&ax,&ay,&bx,&by);};
   auto cell=[&](){unsigned x,y;require(layer->worldToMap(static_cast<double>(.8f),static_cast<double>(.05f),x,y),"hit inside rolling grid");return layer->getCost(x,y);};
   auto master=cm->getCostmap();auto merge=[&](unsigned char background){std::fill(master->getCharMap(),master->getCharMap()+10000,background);plugin->updateCosts(*master,0,0,100,100);};
   at(100000000000LL);observe(100000000000LL,true);update();require(cell()==254,"real endpoint marked");++checks;
   merge(0);require(command()==0.,"fresh real obstacle rejects forward command");++checks;
   at(104000000000LL);update();require(cell()==254,"offline sensor retains obstacle");++checks;
   observe(104000000000LL,false);update();
   if(!candidate){require(cell()==254,"stock local memory reproduces phantom wall");merge(0);baseline_blocked=command()==0.;require(baseline_blocked,"stock controller reproduces zero input");checks+=2;}
   else{
    require(cell()==255,"expired dynamic mark reveals static context, not invented free space");++checks;
    merge(0);candidate_forward=command()>0.;require(candidate_forward,"actual controller returns forward after old mark expires");++checks;
    merge(254);require(command()==0.,"static wall still rejects command");++checks;
    merge(255);require(command()==0.,"static unknown still rejects command");++checks;
    // Preserve source times at their world cell through positive, negative,
    // diagonal and sub-cell rolling shifts. Replay of an old cloud cannot
    // refresh its timestamp. A current actual hit always remains lethal.
    for(const auto &shift:std::vector<std::pair<double,double>>{{.2,0.},{-.2,0.},{0.,.2},{0.,-.2},{.2,-.2},{.01,.01}}){
     layer->reset();at(110000000000LL);observe(110000000000LL,true);update();
     at(111000000000LL);observe(111000000000LL,false);update(shift.first,shift.second);require(cell()==254,"world hit/time survives rolling shift");++checks;
     at(114000000000LL);observe(114000000000LL,false);update(shift.first,shift.second);require(cell()==255,"shifted old source expires at original lifetime");++checks;
    }
    layer->reset();at(120000000000LL);observe(120000000000LL,true);update();
    at(124000000000LL);observe(123600000000LL,false);update();require(cell()==254,"late unrelated scan cannot authorize expiration");++checks;
    observe(124000000000LL,true);update();require(cell()==254,"fresh obstacle remains despite elapsed old lifetime");++checks;
    at(126900000000LL);observe(126900000000LL,false);update();require(cell()==254,"fresh hit gets full lifetime");++checks;
    at(127100000000LL);observe(127100000000LL,false);update();require(cell()==255,"fresh hit expires from refreshed original source time");++checks;
    layer->reset();at(130000000000LL);observe(130000000000LL,true);update();
    observe(131000000000LL,false);at(131000000000LL);update(20.,20.);
    observe(135000000000LL,false);at(135000000000LL);update();require(cell()!=254,"large rolling jump cannot attach old time to new cell");++checks;
   }
   plugin.reset();layer.reset();
  }
  std::cout<<"{\"passed\":true,\"checks\":"<<checks<<",\"stock_controller_blocked\":"<<baseline_blocked<<",\"candidate_controller_forward\":"<<candidate_forward<<",\"source_timed\":true,\"full_footprint_side\":0.55,\"physical_commands\":0,\"model\":\"actual Nav2 layers and controller; synthetic static floor and partial point observations\"}"<<std::endl;
  control->deactivate();control->cleanup();control.reset();goal.reset();cm->on_cleanup(rclcpp_lifecycle::State());
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;rc=1;}
 rclcpp::shutdown();return rc;
}
