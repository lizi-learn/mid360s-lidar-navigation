#include <chrono>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
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
#include "p2_nav_controller/filled_footprint.hpp"
#include "p2_nav_controller/lateral_motion.hpp"

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
      (!position_only_goal_&&std::abs(std::remainder(tf2::getYaw(q)-tf2::getYaw(goal_.pose.orientation),2*std::acos(-1.)))>1e-4)){reference_pivot_=false;response_braking_=false;response_pivot_.reset();breakaway_.reset();settled_pivot_.reset();}
    have_goal_=true;
    phase_.set_goal(goal.pose.position.x,goal.pose.position.y,tf2::getYaw(q),goal.header.frame_id);
    // Pure-pursuit skips repeated-XY poses and can therefore drive forward
    // through an explicit initial pivot in the reference. Honor that reference
    // phase before tracking its straight leg; ordinary curved paths are unchanged.
    size_t pivot_end=0;
    while(pivot_end+1<path.poses.size()&&std::hypot(path.poses[pivot_end+1].pose.position.x-path.poses.front().pose.position.x,
      path.poses[pivot_end+1].pose.position.y-path.poses.front().pose.position.y)<.005)++pivot_end;
    const double reference_turn=std::abs(std::remainder(tf2::getYaw(path.poses[pivot_end].pose.orientation)-tf2::getYaw(path.poses.front().pose.orientation),2.*std::acos(-1.)));
    // The position planner represents each straight rejoin with a pivot. A
    // modest rejoin is still trackable as a checked forward curve; forcing
    // every >.55rad prefix to stop also restarted pivot pulses on replans.
    // Use the configured turn-in-place threshold already used by RPP. Tight
    // corners can still select a collision-checked pivot in the fallback.
    if(pivot_end>0&&(reference_turn>rotate_to_heading_min_angle_||(reference_pivot_&&reference_turn>.15))){reference_pivot_=true;reference_target_=path.poses[pivot_end];reference_target_.header.frame_id=path.header.frame_id;}
    else reference_pivot_=false;
    goal_=goal; plan_source_ns_=rclcpp::Time(path.header.stamp).nanoseconds(); Base::setPlan(path);
  }
  void activate() override { lateral_motion_.reset(); response_braking_=false; response_pivot_.reset(); reference_pivot_=false; phase_.reset(); breakaway_.reset(); settled_pivot_.reset(); Base::activate(); }
  void deactivate() override { lateral_motion_.reset(); response_braking_=false; response_pivot_.reset(); reference_pivot_=false; phase_.reset(); breakaway_.reset(); settled_pivot_.reset(); Base::deactivate(); }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
      nav2_core::GoalChecker *checker) override {
    std::lock_guard<std::mutex> lock(phase_mutex_);
    std::unique_lock<std::mutex> base_lock(mutex_);
    if(!std::isfinite(speed.linear.y))throw nav2_core::PlannerException("Invalid measured lateral velocity");
    lateral_velocity_=lateral_motion_.step(speed.linear.y,clock_->now().seconds());
    if(std::abs(speed.linear.y-lateral_velocity_)>.08)
      RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,1000,"Lateral motion evidence: raw=%.3f sustained=%.3f",speed.linear.y,lateral_velocity_);
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
    if (use_collision_detection_ && (v==0.?pivot_collision(pose,w,pivot_error,speed.angular.z,distance):body_arc_collision(pose,v,w,distance)))
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
      bool pivot_clear=true;
      {auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
        pivot_clear=!use_collision_detection_||!pivot_collision(pose,w,error,speed.angular.z,distance);}
      if(pivot_clear) {
        if(std::abs(error)<.15&&!settled_pivot_.pending())reference_pivot_=false;
        geometry_msgs::msg::TwistStamped out;out.header=pose.header;out.twist.angular.z=w;return out;
      }
      // A global-map pivot can disagree with newer/local obstacle cells at a
      // line endpoint. Try the existing fully checked forward/window tracking
      // before failing the action; moving past the corner can make the turn fit.
      reference_pivot_=false;settled_pivot_.reset();
      RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,1000,"Reference pivot unavailable locally; evaluating current-body forward alternatives");
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
    std::optional<geometry_msgs::msg::TwistStamped> close_tracking;
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
          if(use_collision_detection_&&body_arc_collision(pose,out.twist.linear.x,bounded,distance))
            throw nav2_core::PlannerException("Forward tracking arc collision ahead");}
        out.twist.angular.z=bounded;
        {auto *map=costmap_ros_->getCostmap();std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
          if(collision_lookahead_floor_>0.&&out.twist.linear.x>0.&&inflation_exposure(pose,out.twist.linear.x,bounded,distance)>.22){close_tracking=out;break;}}
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
      auto score_command=[&](double v,double w) {
        const double px=std::abs(w)<1e-6?v:v*std::sin(w)/w+lateral_velocity_*(std::cos(w)-1.)/w;
        const double py=std::abs(w)<1e-6?lateral_velocity_:v*(1.-std::cos(w))/w+lateral_velocity_*std::sin(w)/w;
        const double heading=std::remainder(angle-w,2*std::acos(-1.));
        return 2.*(std::pow(carrot.pose.position.x-px,2)+std::pow(carrot.pose.position.y-py,2))+.3*heading*heading+.05*w*w+3.*inflation_exposure(pose,v,w,distance);
      };
      if(close_tracking&&(!use_collision_detection_||!body_arc_collision(pose,close_tracking->twist.linear.x,close_tracking->twist.angular.z,distance))){best_v=close_tracking->twist.linear.x;best_w=close_tracking->twist.angular.z;best=score_command(best_v,best_w);found=true;}
      // A small velocity window is a fallback after pursuit arcs collide.
      // Every sampled command keeps the original full prediction horizon.
      for(double v:{std::min(desired_linear_vel_,.6),0.})for(double fraction:{-1.,-.875,-.75,-.625,-.5,-.375,-.25,-.125,0.,.125,.25,.375,.5,.625,.75,.875,1.}) {
        const double w=fraction*rotate_to_heading_angular_vel_;
        if(v==0.&&(std::abs(angle)<=.15||std::abs(w)<minimum_pivot_||w*angle<=0.))continue;
        if(use_collision_detection_&&body_arc_collision(pose,v,w,distance))continue;
        const double score=score_command(v,w);
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
  double inflation_exposure(const geometry_msgs::msg::PoseStamped &pose,double v,double w,double distance) {
    auto *map=costmap_ros_->getCostmap();const double x=pose.pose.position.x,y=pose.pose.position.y,yaw=tf2::getYaw(pose.pose.orientation);
    double sum=0.,peak=0.;int count=0;
    for(int i=0;i<=15;++i) {
      const double t=max_allowed_time_to_collision_up_to_carrot_*i/15.,a=yaw+w*t;
      const double px=x+(std::abs(w)<1e-8?t*(v*std::cos(yaw)-lateral_velocity_*std::sin(yaw)):v/w*(std::sin(a)-std::sin(yaw))+lateral_velocity_/w*(std::cos(a)-std::cos(yaw)));
      const double py=y+(std::abs(w)<1e-8?t*(v*std::sin(yaw)+lateral_velocity_*std::cos(yaw)):v/w*(std::cos(yaw)-std::cos(a))+lateral_velocity_/w*(std::sin(a)-std::sin(yaw)));
      if(std::hypot(px-x,py-y)>distance)break;
      unsigned ix,iy;if(!map->worldToMap(px,py,ix,iy))return 1.;
      const double cost=std::min(253.,static_cast<double>(map->getCost(ix,iy)))/253.;
      sum+=cost*cost;peak=std::max(peak,cost*cost);++count;
    }
    // Existing inflation is a preference, never a larger footprint or a new
    // rejection condition. Retain the clear RPP candidate if no better arc fits.
    return count?.5*(sum/count+peak):1.;
  }
  bool body_arc_collision(const geometry_msgs::msg::PoseStamped &pose,double v,double w,double distance) {
    const double horizon=max_allowed_time_to_collision_up_to_carrot_;
    if(!std::isfinite(v)||!std::isfinite(w)||!std::isfinite(horizon)||horizon<0.)return true;
    auto *map=costmap_ros_->getCostmap();const auto body=costmap_ros_->getRobotFootprint();
    const double x=pose.pose.position.x,y=pose.pose.position.y,yaw=tf2::getYaw(pose.pose.orientation);
    const int steps=std::max({1,int(std::ceil(horizon/.01)),int(std::ceil(std::abs(v)*horizon/.005)),int(std::ceil(std::abs(w)*horizon/.005))});
    for(int i=0;i<=steps;++i) {
      const double t=horizon*static_cast<double>(i)/steps,a=yaw+w*t;
      // y commands remain zero. Observed lateral motion is nevertheless real:
      // a biped can drift toward the obstacle while following a left-side path.
      const double px=x+(std::abs(w)<1e-8?t*(v*std::cos(yaw)-lateral_velocity_*std::sin(yaw)):v/w*(std::sin(a)-std::sin(yaw))+lateral_velocity_/w*(std::cos(a)-std::cos(yaw)));
      const double py=y+(std::abs(w)<1e-8?t*(v*std::sin(yaw)+lateral_velocity_*std::cos(yaw)):v/w*(std::cos(yaw)-std::cos(a))+lateral_velocity_/w*(std::sin(a)-std::sin(yaw)));
      if(std::hypot(px-x,py-y)>distance)break;
      if(!filled_footprint_clear(map,body,px,py,a))return true;
    }
    return false;
  }
  bool pivot_collision(const geometry_msgs::msg::PoseStamped &pose,double requested,
      double remaining_error,double measured,double /*distance*/) {
    const double horizon=max_allowed_time_to_collision_up_to_carrot_;
    if(!std::isfinite(requested)||!std::isfinite(remaining_error)||!std::isfinite(measured))return true;
    const double requested_angle=requested*remaining_error>=0.?remaining_error:requested*horizon;
    auto *map=costmap_ros_->getCostmap();const auto body=costmap_ros_->getRobotFootprint();
    auto clear=[&](double angle) {
      const double yaw=tf2::getYaw(pose.pose.orientation);
      const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(angle)/.005)));
      for(int i=0;i<=steps;++i)if(!filled_footprint_clear(map,body,pose.pose.position.x,
          pose.pose.position.y,yaw+angle*static_cast<double>(i)/steps))return false;
      return true;
    };
    // Check the COMPLETE intended angle, irrespective of pulse magnitude;
    // do not add rotation beyond the requested yaw. Filled-cell intersection
    // also avoids outline raster gaps near the corner of the square body.
    // The measured continuing rotation keeps the original full time horizon.
    return !clear(requested_angle)||!clear(measured*horizon);
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
    if(!response_braking_&&!response_pivot_.pending()&&std::abs(speed.angular.z-out.twist.angular.z)<=.06&&std::abs(lateral_velocity_)<=.06)return;
    auto *map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex(),std::defer_lock);
    if(acquire_map)map_lock.lock();
    const double forward=std::max(speed.linear.x,out.twist.linear.x);
    const bool continuing_arc_blocked=body_arc_collision(pose,forward,speed.angular.z,distance);
    // Keep one correction phase. Releasing merely because measured translation
    // dipped near zero alternated forward/pivot on every acceleration and kept
    // resetting the downstream turn ramp. The normal handoff settles yaw before
    // resuming forward, so that zero-yaw continuation must also fit at release.
    const bool settling_arc_blocked=(response_braking_||response_pivot_.pending())&&body_arc_collision(pose,forward,0.,distance);
    response_braking_=continuing_arc_blocked||settling_arc_blocked;
    if(response_braking_) {
      RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,1000,
        "Ongoing turn intersects map: measured_x=%.3f measured_yaw=%.3f corrective_yaw=%.3f; translation zero while correction remains active",
        speed.linear.x,speed.angular.z,out.twist.angular.z);
      out.twist.linear.x=0.;
      if(settled_pivot_response_) {
        auto reference=transformGlobalPlan(pose);
        auto carrot=getLookAheadPoint(min_lookahead_dist_,reference);
        const double angle=std::atan2(carrot.pose.position.y,carrot.pose.position.x);
        // This pivot is created AFTER the ordinary RPP/pivot handling. Keep a
        // separate response phase so a forward RPP proposal cannot reset it on
        // every tick. Small raw steering values may not start the D policy.
        const double turn=response_pivot_.step(angle,tf2::getYaw(pose.pose.orientation),
          speed.angular.z,clock_->now().seconds(),.03);
        if(pivot_collision(pose,turn,angle,speed.angular.z,distance)) {
          response_pivot_.reset();throw nav2_core::PlannerException("Measured-turn correction pivot collision ahead");
        }
        if(turn==0.&&!response_pivot_.pending())
          throw nav2_core::PlannerException("Aligned reference has no forward collision-free arc");
        out.twist.angular.z=turn;
        RCLCPP_INFO_THROTTLE(costmap_ros_->get_logger(),*clock_,1000,
          "Measured-turn pivot response: error=%.3f requested=%.3f measured=%.3f pending=%d",
          angle,turn,speed.angular.z,response_pivot_.pending());
      }
    }else if(response_pivot_.pending()) {
      const double now=clock_->now().seconds();response_pivot_.settle(now);
      out.twist.angular.z=response_pivot_.step(0.,tf2::getYaw(pose.pose.orientation),speed.angular.z,now,.03);
      if(response_pivot_.pending())out.twist.linear.x=0.;
      else response_pivot_.reset();
    }
  }
  bool response_braking_=false;
  LateralMotion lateral_motion_;
  double lateral_velocity_=0.;
  bool reference_pivot_=false;geometry_msgs::msg::PoseStamped reference_target_;
  int64_t plan_source_ns_=0;
  std::mutex phase_mutex_; TerminalHeading phase_; geometry_msgs::msg::PoseStamped goal_;
  SettledPivot settled_pivot_,response_pivot_;bool settled_pivot_response_=false,position_only_goal_=false;
  PivotBreakaway breakaway_;double curved_forward_input_=0.,minimum_pivot_=0.,collision_lookahead_floor_=0.;bool have_goal_=false,corner_preview_=false;
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::SettledGoalRPP,nav2_core::Controller)
