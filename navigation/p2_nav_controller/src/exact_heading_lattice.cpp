#include <algorithm>
#include <cmath>
#include <mutex>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "p2_nav_controller/terminal_path.hpp"

namespace p2_nav_controller {
// Preserve the requested final yaw after the lattice's heading quantization.
// P2 can pivot; every interpolated terminal orientation must fit the planning
// clearance footprint. The lattice endpoint position and all prior path remain.
class ExactHeadingLattice : public nav2_smac_planner::SmacPlannerLattice {
 public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    if (start.header.frame_id==goal.header.frame_id) {
      const auto &a=start.pose.orientation;const auto &b=goal.pose.orientation;
      const double an=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w);
      const double bn=std::sqrt(b.x*b.x+b.y*b.y+b.z*b.z+b.w*b.w);
      if (!std::isfinite(an)||!std::isfinite(bn)||an<.8||an>1.2||bn<.8||bn>1.2)
        throw nav2_core::PlannerException("Invalid short-path quaternion");
      auto samples=terminal_path({start.pose.position.x,start.pose.position.y,tf2::getYaw(a)},
                                 {goal.pose.position.x,goal.pose.position.y,tf2::getYaw(b)});
      if (!samples.empty()) {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));
        nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(_costmap);
        bool clear=true;
        for(const auto &p:samples) {
          const double cost=checker.footprintCostAtPose(p[0],p[1],p[2],_costmap_ros->getRobotFootprint());
          if(!std::isfinite(cost)||cost>=nav2_costmap_2d::LETHAL_OBSTACLE){clear=false;break;}
        }
        if(clear) {
          nav_msgs::msg::Path short_path;short_path.header=goal.header;
          for(const auto &p:samples) {
            geometry_msgs::msg::PoseStamped pose;pose.header=short_path.header;
            pose.pose.position.x=p[0];pose.pose.position.y=p[1];
            pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(p[2]);
            short_path.poses.push_back(pose);
          }
          return short_path;
        }
      }
    }
    auto path=SmacPlannerLattice::createPlan(start,goal);
    if (path.poses.empty())return path;
    if (goal.header.frame_id!=path.header.frame_id)
      throw nav2_core::PlannerException("Exact terminal heading requires the map frame");
    const auto &q=goal.pose.orientation;
    const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
    if (!std::isfinite(norm) || norm<.8 || norm>1.2)
      throw nav2_core::PlannerException("Invalid requested terminal quaternion");
    const auto end=path.poses.back();
    const double yaw=tf2::getYaw(end.pose.orientation),requested=tf2::getYaw(q);
    const double delta=std::remainder(requested-yaw,2.*std::acos(-1.));
    const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/.02)));
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(_costmap);
    for (int i=0;i<=steps;++i) {
      const double angle=yaw+delta*static_cast<double>(i)/steps;
      const double cost=checker.footprintCostAtPose(end.pose.position.x,end.pose.position.y,angle,_costmap_ros->getRobotFootprint());
      if (!std::isfinite(cost) || cost>=nav2_costmap_2d::LETHAL_OBSTACLE)
        throw nav2_core::PlannerException("Requested terminal rotation lacks planning clearance");
    }
    for (int i=1;i<=steps;++i) {
      auto pose=end;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw+delta*static_cast<double>(i)/steps);path.poses.push_back(pose);
    }
    return path;
  }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::ExactHeadingLattice,nav2_core::GlobalPlanner)
