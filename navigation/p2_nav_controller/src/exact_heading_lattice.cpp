#include "p2_nav_controller/position_only_lattice.hpp"
#include "p2_nav_controller/filled_footprint.hpp"
namespace p2_nav_controller {
// Preserve explicit arrival yaw, with complete filled/swept-body verification.
// Reuse the existing position planner for an alternative approach when the
// lattice's sampled primitive fits but its continuous sweep does not.
class ExactHeadingLattice : public PositionOnlyLattice {
  bool clear(double x,double y,double yaw) {
    return filled_footprint_clear(_costmap,_costmap_ros->getRobotFootprint(),x,y,yaw);
  }
  nav_msgs::msg::Path completed(nav_msgs::msg::Path path) {
    const auto node=_node.lock();
    if(!node)throw nav2_core::PlannerException("Planner node expired");
    path.header.stamp=node->now();
    for(auto &pose:path.poses)pose.header=path.header;
    return path;
  }
  nav_msgs::msg::Path exact(nav_msgs::msg::Path path,const geometry_msgs::msg::PoseStamped &goal) {
    if(path.poses.empty())throw nav2_core::PlannerException("Empty directed approach");
    if(goal.header.frame_id!=path.header.frame_id)throw nav2_core::PlannerException("Exact terminal heading requires the map frame");
    const auto end=path.poses.back();
    const double yaw=tf2::getYaw(end.pose.orientation),requested=tf2::getYaw(goal.pose.orientation);
    const double delta=std::remainder(requested-yaw,2*std::acos(-1.));
    const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/.02)));
    for(int i=1;i<=steps;++i){auto pose=end;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw+delta*i/steps);path.poses.push_back(pose);}
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    for(size_t i=0;i<path.poses.size();++i) {
      const auto &a=path.poses[i?i-1:i].pose;const auto &b=path.poses[i].pose;
      const double da=std::remainder(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation),2*std::acos(-1.));
      const int n=std::max({1,static_cast<int>(std::ceil(std::hypot(b.position.x-a.position.x,b.position.y-a.position.y)/.01)),static_cast<int>(std::ceil(std::abs(da)/.01))});
      for(int k=0;k<=n;++k){double f=static_cast<double>(k)/n;if(!clear(a.position.x+f*(b.position.x-a.position.x),a.position.y+f*(b.position.y-a.position.y),tf2::getYaw(a.orientation)+f*da))throw nav2_core::PlannerException("Directed path has a filled-body sweep intersection");}
    }
    return completed(std::move(path));
  }
 public:
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) override {
    if(start.header.frame_id!=goal.header.frame_id)throw nav2_core::PlannerException("Directed goal frame mismatch");
    for(const auto &q:{start.pose.orientation,goal.pose.orientation}) {
      const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
      if(!std::isfinite(norm)||norm<.8||norm>1.2)throw nav2_core::PlannerException("Invalid directed quaternion");
    }
    // A directed visit uses the same roomier, forward-only route search as a
    // position visit. That search can include collision-checked zero-translation
    // pivots; it need not first fail a car-like approach or ask the operator to
    // turn the robot. Append and check the requested arrival yaw separately.
    // The former zero-margin straight shortcut bypassed the position planner's
    // room preference and route memory even when a wider route was available.
    try {return exact(PositionOnlyLattice::createPlan(start,goal),goal);}
    catch(const nav2_core::PlannerException &error) {
      RCLCPP_INFO(_logger,"Directed position/pivot route unavailable; checking lattice alternative: %s",error.what());
      // Keep a genuinely feasible lattice alternative. Both branches retain
      // the full swept body and exact arrival yaw; neither authorizes movement
      // through an occupied start, goal or pivot.
      return exact(SmacPlannerLattice::createPlan(start,goal),goal);
    }
  }
};
// A reception batch contains only action-free transit points before its final
// visit. Their yaw is a seed, never an instruction to turn at each waypoint.
class RouteDirectedLattice : public ExactHeadingLattice {
  std::string prefix_;
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    ExactHeadingLattice::configure(parent,name,tf,costmap);prefix_=name;
    auto node=parent.lock();
    for(const auto &key:{"final_x","final_y"})nav2_util::declare_parameter_if_not_declared(node,name+"."+key,rclcpp::ParameterValue(0.));
    nav2_util::declare_parameter_if_not_declared(node,name+".final_request_ns",rclcpp::ParameterValue(int64_t(0)));
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) override {
    auto node=_node.lock();if(!node)throw nav2_core::PlannerException("Route planner node expired");
    double x,y;node->get_parameter(prefix_+".final_x",x);node->get_parameter(prefix_+".final_y",y);
    int64_t stamp;node->get_parameter(prefix_+".final_request_ns",stamp);
    // Original request time identifies the final occurrence in a route that
    // may cross these coordinates earlier. No pose time is rewritten.
    if(std::hypot(goal.pose.position.x-x,goal.pose.position.y-y)<=1e-6&&rclcpp::Time(goal.header.stamp).nanoseconds()==stamp)return ExactHeadingLattice::createPlan(start,goal);
    return PositionOnlyLattice::createPlan(start,goal);
  }
};
// Separate planner IDs keep corridor guidance distinct from exact through-poses.
// The original final header identifies the final occurrence, even when a route
// crosses those same coordinates earlier. No sensor/goal stamp is rewritten.
class CorridorRouteLattice : public ExactHeadingLattice {
  std::string prefix_;
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    ExactHeadingLattice::configure(parent,name,tf,costmap);prefix_=name;auto node=parent.lock();
    for(const auto &key:{"final_x","final_y"})nav2_util::declare_parameter_if_not_declared(node,name+"."+key,rclcpp::ParameterValue(0.));
    nav2_util::declare_parameter_if_not_declared(node,name+".final_request_ns",rclcpp::ParameterValue(int64_t(0)));
    nav2_util::declare_parameter_if_not_declared(node,name+".reference_radius",rclcpp::ParameterValue(.6));
    nav2_util::declare_parameter_if_not_declared(node,name+".final_heading_required",rclcpp::ParameterValue(false));
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) override {
    auto node=_node.lock();if(!node)throw nav2_core::PlannerException("Corridor planner node expired");
    double x,y,radius;int64_t stamp;bool heading_required;
    node->get_parameter(prefix_+".final_x",x);node->get_parameter(prefix_+".final_y",y);
    node->get_parameter(prefix_+".final_request_ns",stamp);node->get_parameter(prefix_+".reference_radius",radius);
    node->get_parameter(prefix_+".final_heading_required",heading_required);
    if(stamp<=0)throw nav2_core::PlannerException("Missing actual final route request");
    if(std::hypot(goal.pose.position.x-x,goal.pose.position.y-y)<=1e-6&&rclcpp::Time(goal.header.stamp).nanoseconds()==stamp)
      return heading_required?ExactHeadingLattice::createPlan(start,goal):PositionOnlyLattice::createPlan(start,goal);
    return PositionOnlyLattice::createReferencePlan(start,goal,radius);
  }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::CorridorRouteLattice,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::ExactHeadingLattice,nav2_core::GlobalPlanner)

PLUGINLIB_EXPORT_CLASS(p2_nav_controller::RouteDirectedLattice,nav2_core::GlobalPlanner)
