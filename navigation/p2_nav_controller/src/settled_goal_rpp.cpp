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
#include "p2_nav_controller/settled_pivot.hpp"
#include "p2_nav_controller/reference_goal.hpp"
#include "p2_nav_controller/curved_forward.hpp"

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
    nav2_util::declare_parameter_if_not_declared(node,name+".corner_preview",rclcpp::ParameterValue(false));
    node->get_parameter(name+".corner_preview",corner_preview_);
    nav2_util::declare_parameter_if_not_declared(node,name+".settled_pivot_response",rclcpp::ParameterValue(false));
    node->get_parameter(name+".settled_pivot_response",settled_pivot_response_);
    nav2_util::declare_parameter_if_not_declared(node,name+".curved_forward_input",rclcpp::ParameterValue(0.));
    node->get_parameter(name+".curved_forward_input",curved_forward_input_);
    if(!std::isfinite(curved_forward_input_)||curved_forward_input_<0.)throw nav2_core::PlannerException("Invalid curved forward input");
    if(!std::isfinite(collision_lookahead_floor_)||collision_lookahead_floor_<0||collision_lookahead_floor_>min_lookahead_dist_)
      throw nav2_core::PlannerException("Invalid collision lookahead floor");
    if(!std::isfinite(minimum_pivot_)||minimum_pivot_<0||minimum_pivot_>rotate_to_heading_angular_vel_)
      throw nav2_core::PlannerException("Invalid minimum pivot input");
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    if (path.poses.empty()) { reference_pivot_=false; phase_.reset(); settled_pivot_.reset(); throw nav2_core::PlannerException("Empty terminal path"); }
    auto goal=path.poses.back(); goal.header.frame_id=path.header.frame_id;
    const auto &q=goal.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (!std::isfinite(norm) || norm<.8 || norm>1.2) { reference_pivot_=false; phase_.reset(); settled_pivot_.reset(); throw nav2_core::PlannerException("Invalid terminal quaternion"); }
    if(!have_goal_||goal.header.frame_id!=goal_.header.frame_id||
      std::hypot(goal.pose.position.x-goal_.pose.position.x,goal.pose.position.y-goal_.pose.position.y)>1e-4||
      (!position_only_goal_&&std::abs(std::remainder(tf2::getYaw(q)-tf2::getYaw(goal_.pose.orientation),2*std::acos(-1.)))>1e-4)){reference_pivot_=false;response_braking_=false;breakaway_.reset();settled_pivot_.reset();}
    have_goal_=true;
    phase_.set_goal(goal.pose.position.x,goal.pose.position.y,tf2::getYaw(q),goal.header.frame_id);
    // Pure-pursuit skips repeated-XY poses and can therefore drive forward
    // through an explicit initial pivot in the reference. Honor that reference
    // phase before tracking its straight leg; ordinary curved paths are unchanged.
    size_t pivot_end=0;
    while(pivot_end+1<path.poses.size()&&std::hypot(path.poses[pivot_end+1].pose.position.x-path.poses.front().pose.position.x,
      path.poses[pivot_end+1].pose.position.y-path.poses.front().pose.position.y)<.005)++pivot_end;
    const double reference_turn=std::abs(std::remainder(tf2::getYaw(path.poses[pivot_end].pose.orientation)-tf2::getYaw(path.poses.front().pose.orientation),2.*std::acos(-1.)));
    if(pivot_end>0&&(reference_turn>.55||(reference_pivot_&&reference_turn>.15))){reference_pivot_=true;reference_target_=path.poses[pivot_end];reference_target_.header.frame_id=path.header.frame_id;}
    else if(pivot_end==0)reference_pivot_=false;
    goal_=goal; plan_source_ns_=rclcpp::Time(path.header.stamp).nanoseconds(); Base::setPlan(path);
  }
  void activate() override { response_braking_=false; reference_pivot_=false; phase_.reset(); breakaway_.reset(); settled_pivot_.reset(); Base::activate(); }
  void deactivate() override { response_braking_=false; reference_pivot_=false; phase_.reset(); breakaway_.reset(); settled_pivot_.reset(); Base::deactivate(); }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    std::unique_lock<std::mutex> base_lock(mutex_);
    geometry_msgs::msg::Pose tolerance; geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker || !checker->getTolerances(tolerance,velocity_tolerance))
      throw nav2_core::PlannerException("Terminal goal tolerance unavailable");
    if(auto *reference=dynamic_cast<ReferenceGoal*>(checker))reference->reference_goal(goal_);
    auto goal=goal_; goal.header.stamp=pose.header.stamp;
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(pose.header.frame_id,goal,transformed))
      throw nav2_core::PlannerException("Terminal goal transform unavailable");
    const double distance=std::hypot(transformed.pose.position.x-pose.pose.position.x,
                                    transformed.pose.position.y-pose.pose.position.y);
    const double now=clock_->now().seconds();
    const double error=std::remainder(tf2::getYaw(transformed.pose.orientation)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
    const bool aligned=std::abs(error)<std::abs(tf2::getYaw(tolerance.orientation));
    const bool rotate_final=phase_.step(distance,tolerance.position.x,now,aligned);
    const bool position_only=std::abs(tf2::getYaw(tolerance.orientation))>3.14;position_only_goal_=position_only;
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
    const double pivot_error=rotate_final?error:bearing;
    const double requested_w=v==0.?(settled_pivot_response_?settled_pivot_.step(pivot_error,tf2::getYaw(pose.pose.orientation),speed.angular.z,now,rotate_final?std::abs(tf2::getYaw(tolerance.orientation)):.15):breakaway_.step(command[1],pivot_error,speed.angular.z,
      rotate_final?std::abs(tf2::getYaw(tolerance.orientation)):.15,minimum_pivot_,rotate_to_heading_angular_vel_,now)):command[1];
    const double w=std::clamp(requested_w,-rotate_to_heading_angular_vel_,rotate_to_heading_angular_vel_);
    auto *costmap=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*(costmap->getMutex()));
    if (use_collision_detection_ && isCollisionImminent(pose,v,w,distance))
      {settled_pivot_.reset();throw nav2_core::PlannerException("Terminal rotation collision ahead");}
    geometry_msgs::msg::TwistStamped out; out.header=pose.header; out.twist.linear.x=v; out.twist.angular.z=w; return out;
  }
 private:
  geometry_msgs::msg::TwistStamped collision_aware_base(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker,double distance,std::unique_lock<std::mutex> &lock) {
    if(reference_pivot_&&settled_pivot_response_) {
      auto reference=reference_target_;reference.header.stamp=pose.header.stamp;
      geometry_msgs::msg::PoseStamped target;
      if(!transformPose(pose.header.frame_id,reference,target))throw nav2_core::PlannerException("Reference pivot transform unavailable");
      const double error=std::remainder(tf2::getYaw(target.pose.orientation)-tf2::getYaw(pose.pose.orientation),2.*std::acos(-1.));
      const double w=settled_pivot_.step(error,tf2::getYaw(pose.pose.orientation),speed.angular.z,clock_->now().seconds(),.15);
      {auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
        if(use_collision_detection_&&isCollisionImminent(pose,0.,w,distance))throw nav2_core::PlannerException("Reference pivot collision ahead");}
      if(std::abs(error)<.15&&!settled_pivot_.pending())reference_pivot_=false;
      geometry_msgs::msg::TwistStamped out;out.header=pose.header;out.twist.angular.z=w;return out;
    }
    const double lo=min_lookahead_dist_,hi=max_lookahead_dist_;
    struct Restore {
      SettledGoalRPP *self;std::unique_lock<std::mutex> &lock;double lo,hi;
      ~Restore(){if(!lock.owns_lock())lock.lock();self->min_lookahead_dist_=lo;self->max_lookahead_dist_=hi;}
    } restore{this,lock,lo,hi};
    double preview=lo;
    if(corner_preview_) {
      auto local=transformGlobalPlan(pose);
      std::vector<geometry_msgs::msg::Point> points;
      for(const auto &p:local.poses)if(points.empty()||std::hypot(p.pose.position.x-points.back().x,p.pose.position.y-points.back().y)>.005)points.push_back(p.pose.position);
      for(size_t i=1;i+1<points.size();++i) {
        const auto &a=points[i-1];const auto &b=points[i];const auto &c=points[i+1];
        const double turn=std::abs(std::remainder(std::atan2(c.y-b.y,c.x-b.x)-std::atan2(b.y-a.y,b.x-a.x),2*std::acos(-1.)));
        const double distance=std::hypot(b.x,b.y);
        if(turn>.5&&distance>.10&&distance<preview)preview=std::max(.20,distance);
      }
      min_lookahead_dist_=std::min(lo,preview);max_lookahead_dist_=std::min(hi,preview);
    }
    std::vector<double> previews{preview};
    if(collision_lookahead_floor_>0.)for(double value:{.9,.7,collision_lookahead_floor_})
      if(value<previews.back()-1e-6&&value>=collision_lookahead_floor_)previews.push_back(value);
    std::string last="No collision-free tracking command";
    for(size_t i=0;i<previews.size();++i) {
      if(i){min_lookahead_dist_=previews[i];max_lookahead_dist_=previews[i];}
      try {
        lock.unlock();auto out=Base::computeVelocityCommands(pose,speed,checker);lock.lock();
        if(settled_pivot_response_&&((out.twist.linear.x==0.&&out.twist.angular.z!=0.)||settled_pivot_.pending())) {
          auto reference=transformGlobalPlan(pose);auto carrot=getLookAheadPoint(lo,reference);
          const double angle=std::atan2(carrot.pose.position.y,carrot.pose.position.x);
          out.twist.linear.x=0.;out.twist.angular.z=settled_pivot_.step(angle,tf2::getYaw(pose.pose.orientation),speed.angular.z,clock_->now().seconds());
        }else if(out.twist.linear.x>0.)settled_pivot_.reset();
        const double bounded=std::clamp(out.twist.angular.z,-rotate_to_heading_angular_vel_,rotate_to_heading_angular_vel_);
        const double previous_forward=out.twist.linear.x;
        out.twist.linear.x=curved_forward(previous_forward,bounded,curved_forward_input_);
        if(out.twist.linear.x<previous_forward-1e-6)RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,2000,"Curved forward input blended: requested=%.6f applied=%.6f yaw=%.6f",previous_forward,out.twist.linear.x,bounded);
        // A shorter carrot must not shorten the original time horizon: check
        // its actual bounded arc out to the unchanged goal/time limit as well.
        {auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
          if(use_collision_detection_&&isCollisionImminent(pose,out.twist.linear.x,bounded,distance))
            throw nav2_core::PlannerException("Forward tracking arc collision ahead");}
        out.twist.angular.z=bounded;
        follow_measured_turn(pose,speed,distance,out);
        if(std::abs(bounded)>.35)RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,500,
          "Pursuit evidence: source=%ld preview=%.3f attempt=%zu x=%.3f y=%.3f yaw=%.3f measured_yaw=%.3f forward=%.3f turn=%.3f",
          static_cast<long>(plan_source_ns_),i?previews[i]:preview,i,pose.pose.position.x,pose.pose.position.y,tf2::getYaw(pose.pose.orientation),speed.angular.z,out.twist.linear.x,bounded);
        return out;
      }catch(const nav2_core::PlannerException &e){
        if(!lock.owns_lock())lock.lock();settled_pivot_.reset();last=e.what();
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
      if(found){
        RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,500,
          "Velocity window evidence: source=%ld angle=%.3f forward=%.3f turn=%.3f measured_yaw=%.3f",
          static_cast<long>(plan_source_ns_),angle,best_v,best_w,speed.angular.z);
        geometry_msgs::msg::TwistStamped out;out.header=pose.header;out.twist.linear.x=best_v;out.twist.angular.z=best_w;
        follow_measured_turn(pose,speed,distance,out,false);return out;}
    }

    throw nav2_core::PlannerException(last);
  }
  void follow_measured_turn(const geometry_msgs::msg::PoseStamped &pose,
      const geometry_msgs::msg::Twist &speed,double distance,geometry_msgs::msg::TwistStamped &out,
      bool acquire_map=true) {
    // A changed yaw request does not instantly change the native input ramp
    // or body motion. Check the continuing MEASURED yaw as well as the
    // requested arc. Only withdraw translation when that ongoing arc intersects
    // the map; retain the corrective yaw and active task, without a plan retry.
    // Keep the already-requested forward projection: a low measured speed can
    // precede an acceleration that is still queued downstream. This uses the
    // existing controller projection, not a calibrated physical-speed model.
    // In open space the ordinary continuous forward correction is unchanged.
    if(!use_collision_detection_||out.twist.linear.x<=0.)return;
    if(!response_braking_&&std::abs(speed.angular.z-out.twist.angular.z)<=.06)return;
    auto *map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex(),std::defer_lock);
    if(acquire_map)map_lock.lock();
    const double forward=std::max(speed.linear.x,out.twist.linear.x);
    const bool continuing_arc_blocked=isCollisionImminent(pose,forward,speed.angular.z,distance);
    // Keep one correction phase. Releasing merely because measured translation
    // dipped near zero alternated forward/pivot on every acceleration and kept
    // resetting the downstream turn ramp. The normal handoff settles yaw before
    // resuming forward, so that zero-yaw continuation must also fit at release.
    const bool settling_arc_blocked=response_braking_&&isCollisionImminent(pose,forward,0.,distance);
    response_braking_=continuing_arc_blocked||settling_arc_blocked;
    if(response_braking_) {
      RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,1000,
        "Ongoing turn intersects map: measured_x=%.3f measured_yaw=%.3f corrective_yaw=%.3f; translation zero while correction remains active",
        speed.linear.x,speed.angular.z,out.twist.angular.z);
      out.twist.linear.x=0.;
    }
  }
  bool response_braking_=false;
  bool reference_pivot_=false;geometry_msgs::msg::PoseStamped reference_target_;
  int64_t plan_source_ns_=0;
  std::mutex phase_mutex_; TerminalHeading phase_; geometry_msgs::msg::PoseStamped goal_;
  SettledPivot settled_pivot_;bool settled_pivot_response_=false,position_only_goal_=false;
  PivotBreakaway breakaway_;double curved_forward_input_=0.,minimum_pivot_=0.,collision_lookahead_floor_=0.;bool have_goal_=false,corner_preview_=false;
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledGoalRPP,nav2_core::Controller)
