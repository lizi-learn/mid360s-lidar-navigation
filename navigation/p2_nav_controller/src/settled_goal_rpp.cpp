#include <chrono>
#include <algorithm>
#include <cmath>
#include <mutex>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "p2_nav_controller/terminal_heading.hpp"
#include "p2_nav_controller/terminal_velocity.hpp"
#include "p2_nav_controller/pivot_breakaway.hpp"

namespace p2_nav_controller {
class SettledGoalRPP : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
  using Base=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cm) override {
    Base::configure(parent,name,tf,cm);auto node=parent.lock();
    nav2_util::declare_parameter_if_not_declared(node,name+".minimum_pivot_input",rclcpp::ParameterValue(0.));
    node->get_parameter(name+".minimum_pivot_input",minimum_pivot_);
    nav2_util::declare_parameter_if_not_declared(node,name+".collision_lookahead_floor",rclcpp::ParameterValue(0.));
    node->get_parameter(name+".collision_lookahead_floor",collision_lookahead_floor_);
    if(!std::isfinite(collision_lookahead_floor_)||collision_lookahead_floor_<0||collision_lookahead_floor_>min_lookahead_dist_)
      throw nav2_core::PlannerException("Invalid collision lookahead floor");
    if(!std::isfinite(minimum_pivot_)||minimum_pivot_<0||minimum_pivot_>rotate_to_heading_angular_vel_)
      throw nav2_core::PlannerException("Invalid minimum pivot input");
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (path.poses.empty()) { phase_.reset(); throw nav2_core::PlannerException("Empty terminal path"); }
    auto goal=path.poses.back(); goal.header.frame_id=path.header.frame_id;
    const auto &q=goal.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (!std::isfinite(norm) || norm<.8 || norm>1.2) { phase_.reset(); throw nav2_core::PlannerException("Invalid terminal quaternion"); }
    if(!have_goal_||goal.header.frame_id!=goal_.header.frame_id||
      std::hypot(goal.pose.position.x-goal_.pose.position.x,goal.pose.position.y-goal_.pose.position.y)>1e-4||
      std::abs(std::remainder(tf2::getYaw(q)-tf2::getYaw(goal_.pose.orientation),2*std::acos(-1.)))>1e-4)breakaway_.reset();
    have_goal_=true;
    phase_.set_goal(goal.pose.position.x,goal.pose.position.y,tf2::getYaw(q),goal.header.frame_id);
    goal_=goal; Base::setPlan(path);
  }
  void activate() override { phase_.reset(); breakaway_.reset(); Base::activate(); }
  void deactivate() override { phase_.reset(); breakaway_.reset(); Base::deactivate(); }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    std::unique_lock<std::mutex> base_lock(mutex_);
    geometry_msgs::msg::Pose tolerance; geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker || !checker->getTolerances(tolerance,velocity_tolerance))
      throw nav2_core::PlannerException("Terminal goal tolerance unavailable");
    auto goal=goal_; goal.header.stamp=pose.header.stamp;
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(pose.header.frame_id,goal,transformed))
      throw nav2_core::PlannerException("Terminal goal transform unavailable");
    const double distance=std::hypot(transformed.pose.position.x-pose.pose.position.x,
                                    transformed.pose.position.y-pose.pose.position.y);
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const double error=std::remainder(tf2::getYaw(transformed.pose.orientation)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
    const bool aligned=std::abs(error)<std::abs(tf2::getYaw(tolerance.orientation));
    const bool rotate_final=phase_.step(distance,tolerance.position.x,now,aligned);
    const bool position_only=std::abs(tf2::getYaw(tolerance.orientation))>3.14;
    if (position_only || !use_rotate_to_heading_ || (!rotate_final && distance>2.*tolerance.position.x)) {
      breakaway_.reset();
      return collision_aware_base(pose,speed,checker,distance,base_lock);
    }
    // A new lattice plan can contain a tight arrival loop to impose final yaw.
    // In the bounded terminal region, acquire XY first and then pivot. No lateral
    // or reverse velocity is generated, and the original success tolerances stay.
    const double bearing=std::remainder(std::atan2(transformed.pose.position.y-pose.pose.position.y,
      transformed.pose.position.x-pose.pose.position.x)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
    const auto command=terminal_velocity(distance,bearing,error,rotate_final,rotate_to_heading_angular_vel_);
    const double v=command[0];
    const double w=v==0.?breakaway_.step(command[1],rotate_final?error:bearing,speed.angular.z,
      rotate_final?std::abs(tf2::getYaw(tolerance.orientation)):.15,minimum_pivot_,rotate_to_heading_angular_vel_,now):command[1];
    auto *costmap=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*(costmap->getMutex()));
    if (use_collision_detection_ && isCollisionImminent(pose,v,w,distance))
      throw nav2_core::PlannerException("Terminal rotation collision ahead");
    geometry_msgs::msg::TwistStamped out; out.header=pose.header; out.twist.linear.x=v; out.twist.angular.z=w; return out;
  }
 private:
  geometry_msgs::msg::TwistStamped collision_aware_base(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker,double distance,std::unique_lock<std::mutex> &lock) {
    const double lo=min_lookahead_dist_,hi=max_lookahead_dist_;
    struct Restore {
      SettledGoalRPP *self;std::unique_lock<std::mutex> &lock;double lo,hi;
      ~Restore(){if(!lock.owns_lock())lock.lock();self->min_lookahead_dist_=lo;self->max_lookahead_dist_=hi;}
    } restore{this,lock,lo,hi};
    std::vector<double> previews{lo};
    if(collision_lookahead_floor_>0.)for(double value:{.9,.7,collision_lookahead_floor_})
      if(value<previews.back()-1e-6&&value>=collision_lookahead_floor_)previews.push_back(value);
    std::string last="No collision-free tracking command";
    for(size_t i=0;i<previews.size();++i) {
      if(i){min_lookahead_dist_=previews[i];max_lookahead_dist_=previews[i];}
      try {
        lock.unlock();auto out=Base::computeVelocityCommands(pose,speed,checker);lock.lock();
        const double bounded=std::clamp(out.twist.angular.z,-rotate_to_heading_angular_vel_,rotate_to_heading_angular_vel_);
        // A shorter carrot must not shorten the original time horizon: check
        // its actual bounded arc out to the unchanged goal/time limit as well.
        {auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
          if(use_collision_detection_&&isCollisionImminent(pose,out.twist.linear.x,bounded,distance))
            throw nav2_core::PlannerException("Forward tracking arc collision ahead");}
        out.twist.angular.z=bounded;
        return out;
      }catch(const nav2_core::PlannerException &e){
        if(!lock.owns_lock())lock.lock();last=e.what();
        if(last.find("collision")==std::string::npos&&last.find("Collision")==std::string::npos)throw;
      }
    }
    // A clear pivot can acquire a tight reference corner without a lateral
    // command. Only try it after every forward arc failed its original check.
    if(collision_lookahead_floor_>0.) {
      auto plan=transformGlobalPlan(pose);auto carrot=getLookAheadPoint(lo,plan);
      const double angle=std::atan2(carrot.pose.position.y,carrot.pose.position.x);
      auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
      double best=1e30,best_v=0.,best_w=0.;bool found=false;
      // A small velocity window is a fallback after pursuit arcs collide.
      // Every sampled command keeps the original full prediction horizon.
      for(double v:{std::min(desired_linear_vel_,.6),0.})for(double fraction:{-1.,-.75,-.5,-.25,0.,.25,.5,.75,1.}) {
        const double w=fraction*rotate_to_heading_angular_vel_;
        if(v==0.&&(std::abs(angle)<=.15||std::abs(w)<minimum_pivot_||w*angle<=0.))continue;
        if(use_collision_detection_&&isCollisionImminent(pose,v,w,distance))continue;
        const double px=std::abs(w)<1e-6?v:v*std::sin(w)/w;
        const double py=std::abs(w)<1e-6?0.:v*(1.-std::cos(w))/w;
        const double heading=std::remainder(angle-w,2*std::acos(-1.));
        const double score=2.*(std::pow(carrot.pose.position.x-px,2)+std::pow(carrot.pose.position.y-py,2))+.3*heading*heading+.05*w*w;
        if(score<best){best=score;best_v=v;best_w=w;found=true;}
      }
      if(found){geometry_msgs::msg::TwistStamped out;out.header=pose.header;out.twist.linear.x=best_v;out.twist.angular.z=best_w;return out;}
    }

    throw nav2_core::PlannerException(last);
  }
  std::mutex phase_mutex_; TerminalHeading phase_; geometry_msgs::msg::PoseStamped goal_;
  PivotBreakaway breakaway_;double minimum_pivot_=0.,collision_lookahead_floor_=0.;bool have_goal_=false;
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledGoalRPP,nav2_core::Controller)
