// Copyright 2026 MID360S navigation contributors. SPDX-License-Identifier: MIT
// Reuse Humble source projection and native polygon decisions, with an explicit
// unknown-source stop before publishing any velocity. Never infer obstacles
// from a zero command or substitute a duplicate polygon implementation.
#include <memory>
#include <vector>
#include "nav2_collision_monitor/collision_monitor_node.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/range.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"
#include "std_msgs/msg/bool.hpp"

// This height-sliced sensor contract requires actual finite obstacle endpoints.
// An empty source is unknown: missing/stale data, failed TF and an all-unknown
// scan must not be interpreted as clear space. Keep native projection and the
// native polygon algorithms; only observe whether projection produced data.
struct ObservationValidity {
  mutable bool observed{false};
  virtual ~ObservationValidity() = default;
};
template<class Base> class CheckedSource : public Base, public ObservationValidity {
public:
  using Base::Base;
  void getData(const rclcpp::Time &now,
    std::vector<nav2_collision_monitor::Point> &points) const override {
    this->observed=false;
    if(!this->data_)return;
    const double age=(now-rclcpp::Time(this->data_->header.stamp,now.get_clock_type())).seconds();
    if(age<-.05 || age>this->source_timeout_.seconds())return;
    const auto before=points.size();
    Base::getData(now,points);
    this->observed=points.size()>before;
  }
};

class ObservedCollisionMonitor : public nav2_collision_monitor::CollisionMonitor {
  using State = nav2_msgs::msg::CollisionMonitorState;
  rclcpp_lifecycle::LifecyclePublisher<State>::SharedPtr state_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr source_health_;
  std::vector<std::shared_ptr<ObservationValidity>> checks_;
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
    nav2_util::declare_parameter_if_not_declared(shared_from_this(),"source_valid_topic",
      rclcpp::ParameterValue("/p2_nav/collision_source_valid"));
    source_health_=create_publisher<std_msgs::msg::Bool>(get_parameter("source_valid_topic").as_string(),1);
    // Replace source readers while inactive; no duplicate live subscribers.
    // The installed Humble Source::getData returns void, so its failure would
    // otherwise leave an empty point array indistinguishable from clear space.
    sources_.clear();checks_.clear();
    const auto base=get_parameter("base_frame_id").as_string();
    const auto odom=get_parameter("odom_frame_id").as_string();
    const auto tolerance=tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
    const auto timeout=rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
    const auto shift=get_parameter("base_shift_correction").as_bool();
    const auto names=get_parameter("observation_sources").as_string_array();
    for(const auto &name:names) {
      const auto type=get_parameter(name+".type").as_string();
      if(type=="scan")add_source<nav2_collision_monitor::Scan>(name,base,odom,tolerance,timeout,shift);
      else if(type=="pointcloud")add_source<nav2_collision_monitor::PointCloud>(name,base,odom,tolerance,timeout,shift);
      else if(type=="range")add_source<nav2_collision_monitor::Range>(name,base,odom,tolerance,timeout,shift);
      else return nav2_util::CallbackReturn::FAILURE;
    }
    cmd_vel_in_sub_=create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_in_topic").as_string(),1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg){
        if(!process_active_ || !nav2_util::validateTwist(*msg))return;
        verified_process({msg->linear.x,msg->linear.y,msg->angular.z});
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
  template<class Source> void add_source(const std::string &name,const std::string &base,
    const std::string &odom,const tf2::Duration &tolerance,const rclcpp::Duration &timeout,bool shift) {
    auto source=std::make_shared<CheckedSource<Source>>(shared_from_this(),name,tf_buffer_,base,odom,tolerance,timeout,shift);
    source->configure();sources_.push_back(source);checks_.push_back(source);
  }
  void verified_process(const nav2_collision_monitor::Velocity &requested) {
    using namespace nav2_collision_monitor;
    std::vector<Point> points;bool valid=true;std::size_t enabled=0;
    const auto now=this->now();
    for(std::size_t i=0;i<sources_.size();++i)if(sources_[i]->getEnabled()) {
      ++enabled;sources_[i]->getData(now,points);valid=valid && checks_[i]->observed;
    }
    valid=valid && enabled>0;
    std_msgs::msg::Bool health;health.data=valid;source_health_->publish(health);
    Action action{DO_NOTHING,requested};std::shared_ptr<Polygon> cause;
    if(!valid) {
      // No nonzero velocity is published before checking every active source.
      action={STOP,{0.,0.,0.}};
      cmd_vel_out_pub_->publish(geometry_msgs::msg::Twist());
      robot_action_prev_=action;publishPolygons();return;
    }
    for(const auto &polygon:polygons_) {
      if(!polygon->getEnabled() || action.action_type==STOP)continue;
      const auto type=polygon->getActionType();
      const bool selected=(type==STOP || type==SLOWDOWN)
        ? processStopSlowdown(polygon,points,requested,action)
        : type==APPROACH && processApproach(polygon,points,requested,action);
      if(selected)cause=polygon;
    }
    if(action.action_type!=robot_action_prev_.action_type)printAction(action,cause);
    publishVelocity(action);publishPolygons();robot_action_prev_=action;
  }
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &s) override {
    state_->on_activate();source_health_->on_activate();return CollisionMonitor::on_activate(s);
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &s) override {
    const auto result=CollisionMonitor::on_deactivate(s);state_->on_deactivate();source_health_->on_deactivate();return result;
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &s) override {
    checks_.clear();const auto result=CollisionMonitor::on_cleanup(s);state_.reset();source_health_.reset();return result;
  }
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<ObservedCollisionMonitor>(rclcpp::NodeOptions{});
  rclcpp::spin(node->get_node_base_interface());rclcpp::shutdown();return 0;
}
