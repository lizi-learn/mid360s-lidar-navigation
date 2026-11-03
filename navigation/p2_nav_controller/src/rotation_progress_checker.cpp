#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>
#include <string>
#include "nav2_core/progress_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "pluginlib/class_list_macros.hpp"
namespace p2_nav_controller {
// Independent plugin: linking another plugin's implementation as a base can
// register its factory under the wrong class_loader when RPP loads first.
class RotationProgressChecker : public nav2_core::ProgressChecker {
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
  geometry_msgs::msg::PoseStamped baseline_;
  rclcpp::Time baseline_time_{0,0,RCL_ROS_TIME};
  double radius_=.1,angle_=.1,allowance_=40.;bool have_baseline_=false;
  std::mutex mutex_;
 public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,const std::string &name) override {
    auto node=parent.lock();if(!node)throw std::invalid_argument("Progress parent expired");clock_=node->get_clock();
    for(const auto &item:{std::pair<std::string,double>{"required_movement_radius",.1},{"required_movement_angle",.1},{"movement_time_allowance",40.}})
      nav2_util::declare_parameter_if_not_declared(node,name+"."+item.first,rclcpp::ParameterValue(item.second));
    node->get_parameter(name+".required_movement_radius",radius_);node->get_parameter(name+".required_movement_angle",angle_);node->get_parameter(name+".movement_time_allowance",allowance_);
    auto valid=[](double radius,double angle,double allowance){return std::isfinite(radius)&&radius>0&&std::isfinite(angle)&&angle>0&&angle<=std::acos(-1.)&&std::isfinite(allowance)&&allowance>0;};
    if(!valid(radius_,angle_,allowance_))throw std::invalid_argument("Invalid measured progress parameters");
    parameters_=node->add_on_set_parameters_callback([this,name,valid](const std::vector<rclcpp::Parameter> &values){
      std::lock_guard<std::mutex> lock(mutex_);double radius=radius_,angle=angle_,allowance=allowance_;
      for(const auto &p:values){
        const auto &key=p.get_name();
        if(key!=name+".required_movement_radius"&&key!=name+".required_movement_angle"&&key!=name+".movement_time_allowance")continue;
        if(p.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE){rcl_interfaces::msg::SetParametersResult result;result.successful=false;result.reason="Progress values must be doubles";return result;}
        if(key==name+".required_movement_radius")radius=p.as_double();else if(key==name+".required_movement_angle")angle=p.as_double();else allowance=p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult result;result.successful=valid(radius,angle,allowance);
      if(result.successful){radius_=radius;angle_=angle;allowance_=allowance;}else result.reason="Invalid measured progress parameters";
      return result;
    });
  }
  bool check(geometry_msgs::msg::PoseStamped &current) override {
    std::lock_guard<std::mutex> lock(mutex_);
    const double yaw=tf2::getYaw(current.pose.orientation);
    if(!std::isfinite(current.pose.position.x)||!std::isfinite(current.pose.position.y)||!std::isfinite(yaw))return false;
    const auto now=clock_->now();
    if(!have_baseline_||std::hypot(current.pose.position.x-baseline_.pose.position.x,current.pose.position.y-baseline_.pose.position.y)>radius_||
      std::abs(std::remainder(yaw-tf2::getYaw(baseline_.pose.orientation),2*std::acos(-1.)))>=angle_) {
      baseline_=current;baseline_time_=now;have_baseline_=true;return true;
    }
    return (now-baseline_time_).seconds()<=allowance_;
  }
  void reset() override {std::lock_guard<std::mutex> lock(mutex_);have_baseline_=false;}
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::RotationProgressChecker,nav2_core::ProgressChecker)
