#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "pluginlib/class_loader.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "nav2_costmap_2d/costmap_filters/keepout_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
int main(int argc,char **argv){
 rclcpp::init(argc,argv);int result=0;
 try{
  auto node=std::make_shared<nav2_costmap_2d::Costmap2DROS>("dynamic_map_fixture");
  node->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("plugins",std::vector<std::string>{})});
  require(node->on_configure(rclcpp_lifecycle::State())==nav2_util::CallbackReturn::SUCCESS,"costmap configure");
  auto layered=node->getLayeredCostmap();layered->resizeMap(80,80,.1,-4.,-4.);
  tf2_ros::Buffer tf(node->get_clock());auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
  auto plugin=loader.createSharedInstance("p2_nav_controller::TemporalObstacleLayer");
  plugin->initialize(layered,"dynamic",&tf,node,group);
  auto layer=std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);require(bool(layer),"actual obstacle plugin");
  auto clock=node->get_clock()->get_clock_handle();
  require(rcl_enable_ros_time_override(clock)==RCL_RET_OK,"enable isolated fixture time");
  auto at=[&](int64_t ns){require(rcl_set_ros_time_override(clock,ns)==RCL_RET_OK,"set isolated fixture time");};
  auto observe=[&](int64_t stamp,bool mark){
   layer->clearStaticObservations(true,true);nav2_costmap_2d::Observation obs;
   obs.cloud_->header.frame_id="fixture_world";obs.cloud_->header.stamp=rclcpp::Time(stamp);
   sensor_msgs::PointCloud2Modifier modifier(*obs.cloud_);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(mark?1:0);
   if(mark){sensor_msgs::PointCloud2Iterator<float> x(*obs.cloud_,"x"),y(*obs.cloud_,"y"),z(*obs.cloud_,"z");*x=1.f;*y=0.f;*z=.4f;}
   obs.obstacle_min_range_=.1;obs.obstacle_max_range_=11.;layer->addStaticObservation(obs,true,false);
  };
  auto update=[&](){double ax=1e30,ay=1e30,bx=-1e30,by=-1e30;plugin->updateBounds(0.,0.,0.,&ax,&ay,&bx,&by);return bx>0.;};
  auto cell=[&](){unsigned x,y;layer->worldToMap(1.,0.,x,y);return layer->getCost(x,y);};
  at(100000000000LL);observe(100000000000LL,true);update();require(cell()==254,"fresh obstacle retained");
  at(104000000000LL);update();require(cell()==254,"stale sensor cannot authorize expiry");
  observe(104000000000LL,false);require(update(),"expired cell update bounds");require(cell()==255,"old dynamic mark expires to unknown, not free");
  nav2_costmap_2d::Costmap2D master(80,80,.1,-4.,-4.,0);unsigned mx,my;master.worldToMap(1.,0.,mx,my);
  master.setCost(mx,my,254);plugin->updateCosts(master,0,0,80,80);require(master.getCost(mx,my)==254,"static wall survives expiration");
  master.setCost(mx,my,0);plugin->updateCosts(master,0,0,80,80);require(master.getCost(mx,my)==0,"static floor revealed");
  observe(104000000000LL,true);update();at(106900000000LL);observe(106900000000LL,true);update();require(cell()==254,"fresh refresh retained");
  at(109800000000LL);observe(109800000000LL,false);update();require(cell()==254,"refreshed obstacle retains full lifetime");
  at(110000000000LL);observe(110000000000LL,false);update();require(cell()==255,"refreshed obstacle expires from actual source time");
  // Exercise the stock filter after dynamic free-space overwrite.
  auto filter_plugin=loader.createSharedInstance("nav2_costmap_2d::KeepoutFilter");
  node->declare_parameter("boundary.filter_info_topic","/fixture/boundary_info");
  filter_plugin->initialize(layered,"boundary",&tf,node,group);
  filter_plugin->activate();
  auto filter=std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(filter_plugin);require(bool(filter),"actual standard keepout filter");
  auto pubnode=std::make_shared<rclcpp::Node>("dynamic_map_fixture_publisher");auto qos=rclcpp::QoS(1).transient_local().reliable();
  auto info_pub=pubnode->create_publisher<nav2_msgs::msg::CostmapFilterInfo>("/fixture/boundary_info",qos);
  auto mask_pub=pubnode->create_publisher<nav_msgs::msg::OccupancyGrid>("/fixture/boundary_mask",qos);
  nav2_msgs::msg::CostmapFilterInfo info;info.type=0;info.base=0.;info.multiplier=1.;info.filter_mask_topic="/fixture/boundary_mask";
  nav_msgs::msg::OccupancyGrid mask;mask.header.frame_id="fixture_world";mask.info.width=80;mask.info.height=80;mask.info.resolution=.1;mask.info.origin.position.x=-4.;mask.info.origin.position.y=-4.;mask.info.origin.orientation.w=1.;mask.data.assign(6400,-1);
  for(unsigned y=0;y<80;++y)for(unsigned x=0;x<20;++x)mask.data[y*80+x]=100;
  info_pub->publish(info);mask_pub->publish(mask);rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node->get_node_base_interface());executor.add_node(pubnode);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  while(!filter->isActive()&&std::chrono::steady_clock::now()<deadline){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));}
  require(filter->isActive(),"boundary mask received");
  std::fill(master.getCharMap(),master.getCharMap()+6400,0);
  filter->process(master,0,0,80,80,geometry_msgs::msg::Pose2D());
  require(master.getCost(10,40)==254&&master.getCost(40,40)==0,"outside remains forbidden after dynamic clearing");
  // An interior unknown hole is not made free by removing the extra keepout.
  master.setCost(mx,my,255);filter->process(master,0,0,80,80,geometry_msgs::msg::Pose2D());
  require(master.getCost(mx,my)==255,"unobserved interior hole remains unknown");
  // Only an actual ObstacleLayer clearing observation can reveal free floor.
  layer->clearStaticObservations(true,true);nav2_costmap_2d::Observation ray;
  ray.cloud_->header.frame_id="fixture_world";ray.cloud_->header.stamp=rclcpp::Time(110000000000LL);
  sensor_msgs::PointCloud2Modifier modifier(*ray.cloud_);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> rx(*ray.cloud_,"x"),ry(*ray.cloud_,"y"),rz(*ray.cloud_,"z");*rx=3.f;*ry=0.f;*rz=.4f;
  ray.origin_.x=0.;ray.origin_.y=0.;ray.origin_.z=.4;ray.raytrace_min_range_=.1;ray.raytrace_max_range_=12.;
  layer->addStaticObservation(ray,false,true);update();plugin->updateCosts(master,0,0,80,80);
  filter->process(master,0,0,80,80,geometry_msgs::msg::Pose2D());
  require(master.getCost(mx,my)==0&&master.getCost(10,40)==254,"observed interior floor clears while exterior stays protected");
  std::cout<<"{\"passed\":true,\"cases\":12,\"synthetic_source_time\":true,\"actual_plugins\":true,\"physical_commands\":0}"<<std::endl;
  executor.remove_node(node->get_node_base_interface());executor.remove_node(pubnode);filter_plugin->deactivate();filter_plugin.reset();filter.reset();layer.reset();plugin.reset();node->on_cleanup(rclcpp_lifecycle::State());
 }catch(const std::exception &error){std::cerr<<error.what()<<std::endl;result=1;}
 rclcpp::shutdown();return result;
}
