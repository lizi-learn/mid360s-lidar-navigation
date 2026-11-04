#pragma once
#include <cmath>
#include <mutex>
#include <functional>
#include <chrono>
#include <deque>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "p2_nav_controller/terminal_path.hpp"
#include "p2_nav_controller/position_guide.hpp"
#include "p2_nav_controller/route_rejoin.hpp"
namespace p2_nav_controller {
class PositionOnlyLattice : public nav2_smac_planner::SmacPlannerLattice {
  std::vector<unsigned> occupied_;
  double route_rejoin_distance_=2.;
  // A route preference, never an enlarged accepted footprint. Keep full-body
  // narrow fallbacks below when no roomier complete route can be found.
  static constexpr double tracking_room_preference_=.30;
  std::mutex stable_mutex_;
  nav_msgs::msg::Path stable_path_;
  geometry_msgs::msg::PoseStamped stable_goal_;
  struct RouteMemory {nav_msgs::msg::Path path;geometry_msgs::msg::PoseStamped goal;};
  std::deque<RouteMemory> route_memory_;
  struct ReferenceMemory {geometry_msgs::msg::PoseStamped original,selected;};
  std::deque<ReferenceMemory> reference_memory_;
  bool same_goal(const geometry_msgs::msg::PoseStamped &a,const geometry_msgs::msg::PoseStamped &b) const {
    return a.header.frame_id==b.header.frame_id&&std::hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y)<1e-6;
  }
  void select_route(const geometry_msgs::msg::PoseStamped &goal) {
    stable_path_={};stable_goal_=goal;
    for(const auto &entry:route_memory_)if(same_goal(entry.goal,goal)){stable_path_=entry.path;stable_goal_=entry.goal;break;}
  }
  nav_msgs::msg::Path retain_route(nav_msgs::msg::Path path,const geometry_msgs::msg::PoseStamped &goal) {
    for(auto i=route_memory_.begin();i!=route_memory_.end();++i)if(same_goal(i->goal,goal)){route_memory_.erase(i);break;}
    // Bounded memory for alternating legs of ComputePathThroughPoses. Stored
    // paths are only proposals: plan_once rechecks their entire filled sweep
    // against the current map before returning them. Recovery anchors never
    // enter this cache until the complete requested leg has succeeded.
    if(!path.poses.empty()&&path.poses.size()<=16000){
      route_memory_.push_back({path,goal});size_t poses=0;for(const auto &entry:route_memory_)poses+=entry.path.poses.size();
      while(route_memory_.size()>8||poses>32000){poses-=route_memory_.front().path.poses.size();route_memory_.pop_front();}
    }
    return remember(std::move(path),goal);
  }
  nav_msgs::msg::Path remember(nav_msgs::msg::Path path,const geometry_msgs::msg::PoseStamped &goal) {
    stable_path_=path;stable_goal_=goal;return path;
  }
  void prepare_map() {
    const unsigned w=_costmap->getSizeInCellsX(),h=_costmap->getSizeInCellsY(),stride=w+1;
    occupied_.assign((w+1)*(h+1),0);
    for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x)
      occupied_[(y+1)*stride+x+1]=(_costmap->getCost(x,y)>=254)+occupied_[y*stride+x+1]+occupied_[(y+1)*stride+x]-occupied_[y*stride+x];
  }
  bool empty_box(unsigned ax,unsigned ay,unsigned bx,unsigned by) {
    const unsigned stride=_costmap->getSizeInCellsX()+1;
    return occupied_[(by+1)*stride+bx+1]-occupied_[ay*stride+bx+1]-occupied_[(by+1)*stride+ax]+occupied_[ay*stride+ax]==0;
  }
  bool clear(double x,double y,double yaw,double margin=0.) {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(_costmap);
    auto body=_costmap_ros->getRobotFootprint();
    if(margin>0.)nav2_costmap_2d::padFootprint(body,margin);
    // Outline checking alone can miss an occupied cell inside a large body.
    std::vector<std::array<double,2>> polygon;
    double lx=x,ux=x,ly=y,uy=y;const double c=std::cos(yaw),s=std::sin(yaw);
    for(const auto &p:body){double px=x+c*p.x-s*p.y,py=y+s*p.x+c*p.y;polygon.push_back({px,py});lx=std::min(lx,px);ux=std::max(ux,px);ly=std::min(ly,py);uy=std::max(uy,py);}
    unsigned ax,ay,bx,by;
    if(!_costmap->worldToMap(lx,ly,ax,ay)||!_costmap->worldToMap(ux,uy,bx,by))return false;
    // Integral image belongs to this locked planning snapshot only. In empty
    // boxes every interior/outline cell is known clear; mixed boxes retain the
    // exact original filled-polygon checks below. Never cache across updates.
    if(empty_box(ax?ax-1:0,ay?ay-1:0,std::min(bx+1,_costmap->getSizeInCellsX()-1),std::min(by+1,_costmap->getSizeInCellsY()-1)))return true;
    if(checker.footprintCostAtPose(x,y,yaw,body)>=254)return false;
    const double half=_costmap->getResolution()/2.;
    for(int iy=std::max(0,static_cast<int>(ay)-1);iy<=std::min(static_cast<int>(_costmap->getSizeInCellsY())-1,static_cast<int>(by)+1);++iy)
    for(int ix=std::max(0,static_cast<int>(ax)-1);ix<=std::min(static_cast<int>(_costmap->getSizeInCellsX())-1,static_cast<int>(bx)+1);++ix) {
      if(_costmap->getCost(ix,iy)<254)continue;
      double px,py;_costmap->mapToWorld(ix,iy,px,py);
      if(px+half<lx||px-half>ux||py+half<ly||py-half>uy)continue;
      bool separated=false;
      for(size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
        const auto &a=polygon[i];const auto &b=polygon[j];const double nx=b[1]-a[1],ny=a[0]-b[0];
        double low=1e30,high=-1e30;for(const auto &v:polygon){double q=nx*v[0]+ny*v[1];low=std::min(low,q);high=std::max(high,q);}
        double centre=nx*px+ny*py,extent=half*(std::abs(nx)+std::abs(ny));
        if(centre+extent<low||centre-extent>high){separated=true;break;}
      }
      if(!separated)return false;
    }
    return true;
  }
  bool clear(const std::vector<std::array<double,3>> &samples,double margin=0.) {
    for(const auto &p:samples)if(!clear(p[0],p[1],p[2],margin))return false;return true;
  }
  bool swept_clear(const std::vector<std::array<double,3>> &path,double margin) {
    for(size_t i=0;i<path.size();++i) {
      const auto &a=path[i?i-1:i],&b=path[i];
      const double da=std::remainder(b[2]-a[2],2*std::acos(-1.));
      const int n=std::max({1,static_cast<int>(std::ceil(std::hypot(b[0]-a[0],b[1]-a[1])/.01)),static_cast<int>(std::ceil(std::abs(da)/.01))});
      for(int k=0;k<=n;++k){double f=static_cast<double>(k)/n;if(!clear(a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*da,margin))return false;}
    }
    return !path.empty();
  }
  bool pivot_room(double x,double y,double margin=0.) {
    double radius=0.;for(const auto &p:_costmap_ros->getRobotFootprint())radius=std::max(radius,std::hypot(p.x,p.y)+std::sqrt(2.)*margin);
    const double r=_costmap->getResolution(),ox=_costmap->getOriginX(),oy=_costmap->getOriginY();
    const int lx=std::floor((x-radius-ox)/r),ux=std::floor((x+radius-ox)/r),ly=std::floor((y-radius-oy)/r),uy=std::floor((y+radius-oy)/r);
    if(lx<0||ly<0||ux>=static_cast<int>(_costmap->getSizeInCellsX())||uy>=static_cast<int>(_costmap->getSizeInCellsY()))return false;
    if(empty_box(lx,ly,ux,uy))return true;
    for(int iy=ly;iy<=uy;++iy)for(int ix=lx;ix<=ux;++ix)if(_costmap->getCost(ix,iy)>=254){double px,py;_costmap->mapToWorld(ix,iy,px,py);if(std::hypot(std::max(0.,std::abs(px-x)-r/2),std::max(0.,std::abs(py-y)-r/2))<=radius)return false;}
    return true;
  }
  std::vector<std::array<double,3>> visibility(const std::vector<std::array<double,2>> &guide,double yaw,double margin,std::chrono::steady_clock::time_point deadline) {
    if(guide.empty())return {};
    std::vector<std::array<double,2>> points{guide.front()};
    for(size_t i=1;i+1<guide.size();++i)if(std::hypot(guide[i][0]-points.back()[0],guide[i][1]-points.back()[1])>=.25&&pivot_room(guide[i][0],guide[i][1],margin))points.push_back(guide[i]);
    points.push_back(guide.back());std::vector<bool> failed(points.size(),false);
    std::vector<std::array<double,3>> out{{guide.front()[0],guide.front()[1],yaw}};
    std::function<bool(size_t)> visit=[&](size_t from) {
      if(std::chrono::steady_clock::now()>deadline)return false;
      for(size_t to=points.size()-1;to>from;--to){
        if(std::chrono::steady_clock::now()>deadline)return false;
        if(failed[to])continue;
        const auto &q=points[to];const auto pose=out.back();const double a=std::atan2(q[1]-pose[1],q[0]-pose[0]);
        auto leg=terminal_path(pose,{q[0],q[1],a},std::numeric_limits<double>::infinity());
        if(!clear(leg,margin))continue;
        auto size=out.size();out.insert(out.end(),leg.begin()+1,leg.end());
        if(to+1==points.size()||visit(to))return true;
        out.resize(size);
      }
      failed[from]=true;return false;
    };
    return visit(0)?out:std::vector<std::array<double,3>>{};
  }
  double heading(double x,double y,double desired) {
    auto angles=_metadata.heading_angles;
    std::sort(angles.begin(),angles.end(),[&](double a,double b){return std::abs(std::remainder(a-desired,2*std::acos(-1.)))<std::abs(std::remainder(b-desired,2*std::acos(-1.)));});
    unsigned ix,iy;if(!_costmap->worldToMap(x,y,ix,iy))throw nav2_core::PlannerException("Position endpoint outside costmap");
    double cx,cy;_costmap->mapToWorld(ix,iy,cx,cy);
    for(double a:angles)if(clear(x,y,a)&&clear(cx,cy,a))return a;
    throw nav2_core::PlannerException("No full-body heading fits position endpoint ("+std::to_string(x)+", "+std::to_string(y)+")");
  }
  nav_msgs::msg::Path completed(nav_msgs::msg::Path path) {
    // This timestamp belongs to the newly computed path, not the original
    // user's goal request. Relay nodes must preserve this producer timestamp.
    // Sensor, odometry and costmap measurement stamps are never renewed here.
    const auto node=_node.lock();
    if(!node)throw nav2_core::PlannerException("Planner node expired");
    path.header.stamp=node->now();
    for(auto &pose:path.poses)pose.header=path.header;
    return path;
  }
  nav_msgs::msg::Path message(const std::vector<std::array<double,3>> &samples,const std_msgs::msg::Header &header) {
    nav_msgs::msg::Path out;out.header=header;
    for(const auto &p:samples){geometry_msgs::msg::PoseStamped pose;pose.header=header;pose.pose.position.x=p[0];pose.pose.position.y=p[1];pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(p[2]);out.poses.push_back(pose);}return completed(std::move(out));
  }
 public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> cm) override {
    SmacPlannerLattice::configure(parent,name,tf,cm);auto node=parent.lock();
    nav2_util::declare_parameter_if_not_declared(node,name+".route_rejoin_distance",rclcpp::ParameterValue(2.));
    node->get_parameter(name+".route_rejoin_distance",route_rejoin_distance_);
    if(!std::isfinite(route_rejoin_distance_)||route_rejoin_distance_<=0.)throw nav2_core::PlannerException("Invalid route rejoin distance");
  }
  void cleanup() override {
    std::lock_guard<std::mutex> stable_lock(stable_mutex_);
    route_memory_.clear();reference_memory_.clear();stable_path_={};stable_goal_={};
    SmacPlannerLattice::cleanup();
  }
  // A corridor reference denotes a passage area. This method is never used
  // for a final visit: that remains an exact PositionOnly/ExactHeading request.
  nav_msgs::msg::Path createReferencePlan(const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &reference,double radius) {
    if(!std::isfinite(radius)||radius<=0.||radius>=.75)
      throw nav2_core::PlannerException("Invalid corridor reference radius");
    std::vector<geometry_msgs::msg::PoseStamped> candidates;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(900);
    auto remember_reference=[&](const geometry_msgs::msg::PoseStamped &selected) {
      for(auto i=reference_memory_.begin();i!=reference_memory_.end();++i)
        if(same_goal(i->original,reference)){reference_memory_.erase(i);break;}
      reference_memory_.push_back({reference,selected});
      while(reference_memory_.size()>8)reference_memory_.pop_front();
    };
    {
      std::lock_guard<std::mutex> stable_lock(stable_mutex_);
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*_costmap->getMutex());
      prepare_map();
      const double gx=reference.pose.position.x,gy=reference.pose.position.y;
      auto add=[&](geometry_msgs::msg::PoseStamped p) {
        if(std::hypot(p.pose.position.x-gx,p.pose.position.y-gy)>radius+1e-9)return;
        for(const auto &q:candidates)if(same_goal(p,q))return;
        try {p.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(
          heading(p.pose.position.x,p.pose.position.y,tf2::getYaw(reference.pose.orientation)));}
        catch(const nav2_core::PlannerException &){return;}
        candidates.push_back(p);
      };
      add(reference);
      // Keep the selected passage stable while the original centre is occupied.
      // It is only a proposal: each candidate gets a complete LIVE body plan.
      for(const auto &entry:reference_memory_)if(same_goal(entry.original,reference))add(entry.selected);
      std::vector<geometry_msgs::msg::PoseStamped> nearby;
      const double resolution=_costmap->getResolution();
      const int steps=static_cast<int>(std::ceil(radius/resolution));
      for(int dy=-steps;dy<=steps;++dy)for(int dx=-steps;dx<=steps;++dx) {
        if(!dx&&!dy)continue;
        auto p=reference;p.pose.position.x=gx+dx*resolution;p.pose.position.y=gy+dy*resolution;
        if(std::hypot(dx*resolution,dy*resolution)>radius)continue;
        nearby.push_back(p);
      }
      std::stable_sort(nearby.begin(),nearby.end(),[&](const auto &a,const auto &b) {
        return std::hypot(a.pose.position.x-gx,a.pose.position.y-gy)<std::hypot(b.pose.position.x-gx,b.pose.position.y-gy);
      });
      struct Option {geometry_msgs::msg::PoseStamped pose;int quality;double distance;};
      std::vector<Option> options;
      for(auto p:nearby) {
        if(std::chrono::steady_clock::now()>deadline)break;
        try {p.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(
          heading(p.pose.position.x,p.pose.position.y,tf2::getYaw(reference.pose.orientation)));}
        catch(const nav2_core::PlannerException &){continue;}
        const double yaw=tf2::getYaw(p.pose.orientation);
        const int quality=clear(p.pose.position.x,p.pose.position.y,yaw,.08)?0:
          (clear(p.pose.position.x,p.pose.position.y,yaw,.04)?1:2);
        options.push_back({p,quality,std::hypot(p.pose.position.x-gx,p.pose.position.y-gy)});
      }
      // Nearest cells can all be on an obstacle edge. Prefer actual approach
      // room, then spread proposals across the area before spending time on a
      // full lattice search. These margins rank proposals; acceptance still
      // checks the original unchanged footprint.
      std::stable_sort(options.begin(),options.end(),[](const auto &a,const auto &b) {
        return a.quality!=b.quality?a.quality<b.quality:a.distance<b.distance;
      });
      for(const auto &option:options) {
        bool crowded=false;for(const auto &p:candidates)
          if(std::hypot(p.pose.position.x-option.pose.pose.position.x,p.pose.position.y-option.pose.pose.position.y)<.15){crowded=true;break;}
        if(!crowded)candidates.push_back(option.pose);
        if(candidates.size()>=24)break;
      }
      map_lock.unlock();
      for(const auto &candidate:candidates) {
        if(std::chrono::steady_clock::now()>deadline)break;
        select_route(candidate);
        try {
          const auto trial_deadline=std::min(deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(120));
          auto path=plan_once(start,candidate,true,trial_deadline);
          remember_reference(candidate);
          return retain_route(std::move(path),candidate);
        }catch(const nav2_core::PlannerException &){}
      }
    }
    // One ordinary full lattice fallback, rather than an unbounded expensive
    // search for every nearby cell. No obstacle or unknown cell is changed.
    if(!candidates.empty()) {
      auto path=PositionOnlyLattice::createPlan(start,candidates.front());
      std::lock_guard<std::mutex> stable_lock(stable_mutex_);
      remember_reference(candidates.front());return path;
    }
    throw nav2_core::PlannerException("No full-body corridor reference near ("+
      std::to_string(reference.pose.position.x)+", "+std::to_string(reference.pose.position.y)+")");
  }
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    std::lock_guard<std::mutex> stable_lock(stable_mutex_);
    select_route(goal);
    try { return retain_route(plan_once(start,goal),goal); }
    catch(const nav2_core::PlannerException &original_error) {
      // A locally curved exit can defeat the long-distance point guide even
      // though its two complete native plans fit. Search an intermediate
      // reference, never an executable partial goal or a partial success.
      const auto saved_path=stable_path_;const auto saved_goal=stable_goal_;
      const double x=start.pose.position.x,y=start.pose.position.y;
      const double gx=goal.pose.position.x,gy=goal.pose.position.y;
      if(start.header.frame_id!=goal.header.frame_id||std::hypot(gx-x,gy-y)<=3.)throw;
      // Do not spend recovery time searching anchors across a disconnected
      // known-space component, or when the full destination body cannot fit.
      {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
        prepare_map();heading(gx,gy,std::atan2(gy-y,gx-x));
        if(position_guide(_costmap->getCharMap(),_costmap->getSizeInCellsX(),_costmap->getSizeInCellsY(),_costmap->getResolution(),_costmap->getOriginX(),_costmap->getOriginY(),{x,y},{gx,gy},0.).empty())
          throw nav2_core::PlannerException(original_error.what());}
      const double yaw=tf2::getYaw(start.pose.orientation),pi=std::acos(-1.);
      const double error=std::remainder(std::atan2(gy-y,gx-x)-yaw,2*pi);
      const double side=error>=0.?1.:-1.;
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
      for(double distance:{3.,1.5})for(double offset:{side*pi/4.,-side*pi/4.}) {
        if(std::chrono::steady_clock::now()>deadline)break;
        auto via=goal;via.pose.position.x=x+distance*std::cos(yaw+offset);
        via.pose.position.y=y+distance*std::sin(yaw+offset);
        stable_path_=saved_path;stable_goal_=saved_goal;
        try {
          auto joined=plan_once(start,via,true,deadline);
          if(joined.poses.empty())continue;
          auto tail=plan_once(joined.poses.back(),goal,true,deadline);
          if(tail.poses.empty())continue;
          joined.poses.insert(joined.poses.end(),tail.poses.begin()+1,tail.poses.end());
          std::vector<std::array<double,3>> samples;
          for(const auto &p:joined.poses)samples.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
          // Both subplans and their join must still fit the latest unchanged
          // full body map. No occupied/unknown cell is erased or reclassified.
          {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
            prepare_map();if(!swept_clear(samples,0.))continue;}
          RCLCPP_INFO(_logger,"Complete position route recovered through local reference: %.3f %.3f, %zu poses",via.pose.position.x,via.pose.position.y,joined.poses.size());
          return retain_route(completed(std::move(joined)),goal);
        }catch(const nav2_core::PlannerException &){}
      }
      stable_path_=saved_path;stable_goal_=saved_goal;
      throw nav2_core::PlannerException(original_error.what());
    }
  }
 private:
  nav_msgs::msg::Path plan_once(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal,bool visibility_only=false,
                                std::chrono::steady_clock::time_point deadline=std::chrono::steady_clock::time_point::max()) {
    if(start.header.frame_id!=goal.header.frame_id)throw nav2_core::PlannerException("Position goal frame mismatch");
    const double yaw=tf2::getYaw(start.pose.orientation),x=start.pose.position.x,y=start.pose.position.y;
    const double gx=goal.pose.position.x,gy=goal.pose.position.y;
    auto requested=goal;
    const double bearing=std::atan2(gy-y,gx-x);
    auto direct=terminal_path({x,y,yaw},{gx,gy,bearing},std::hypot(_costmap->getSizeInMetersX(),_costmap->getSizeInMetersY()));
    std::vector<std::array<double,2>> guide;std::vector<std::array<double,3>> pivot,narrow_reuse;
    auto seed=start;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
      prepare_map();
      if(clear(direct,tracking_room_preference_))return remember(message(direct,goal.header),goal);
      // Keep a valid detour on the same side of an obstacle. Re-solving a
      // symmetric corridor every second can flip left/right under small AMCL
      // updates. Recheck the complete remaining body path on this LIVE map;
      // never reuse occupied/unknown cells or a stale source timestamp.
      if(!stable_path_.poses.empty()&&stable_goal_.header.frame_id==goal.header.frame_id&&
          std::hypot(stable_goal_.pose.position.x-gx,stable_goal_.pose.position.y-gy)<1e-6) {
        std::vector<std::array<double,3>> stored;
        for(const auto &p:stable_path_.poses)stored.push_back({p.pose.position.x,p.pose.position.y,tf2::getYaw(p.pose.orientation)});
        const auto rejoin_deadline=std::min(deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(100));
        auto joined=route_rejoin({x,y,yaw},stored,route_rejoin_distance_,[&](const auto &samples){
          // Match the direct-route tracking preference. The former 8 cm
          // shortcut could beat the roomier point guide, leaving little room
          // for measured lateral drift. A merely collision-free
          // remembered path must not beat a roomier newly observed detour before
          // we have even searched for it (for example room on the left of a
          // close obstacle on the right). Keep a fully checked fallback so a
          // genuinely narrow passage still works with the original body.
          if(swept_clear(samples,tracking_room_preference_))return true;
          if(!swept_clear(samples,0.))return false;
          if(narrow_reuse.empty())narrow_reuse=samples;
          return false;
        },rejoin_deadline);
        if(!joined.empty())return remember(message(joined,goal.header),goal);
      }

      // Point targets need no terminal heading. Checked pivot/straight
      // visibility connections also avoid a car-like turnaround search. Joints
      // are places where the unchanged body can pivot; narrow straight gaps
      // are crossed as an edge without requiring a turn inside them.
      // Prefer tracking room when available. These are route-search costs,
      // not a larger accepted body: narrow valid routes still fall back to the
      // original full footprint. One shared bound covers all preferences. A visibility shortcut must
      // also prefer tracking room, otherwise it can undo the wider point guide
      // and graze the end of a virtual line after ordinary localization error.
      const auto visibility_deadline=std::min(deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(narrow_reuse.empty()?900:300));
      // Do not jump from a wide preference straight to a zero-clearance point
      // guide. In ordinary corridors intermediate guides can fit even though
      // the wider preferences are disconnected; the shortest point-only guide
      // may instead enter a slit which no complete body path can follow.
      for(double centre_clearance:{.64,.48,.40,.28,.20,.12,.06,0.}) {
        if(std::chrono::steady_clock::now()>visibility_deadline)break;
        auto candidate=position_guide(_costmap->getCharMap(),_costmap->getSizeInCellsX(),_costmap->getSizeInCellsY(),_costmap->getResolution(),_costmap->getOriginX(),_costmap->getOriginY(),{x,y},{gx,gy},centre_clearance);
        if(centre_clearance==0.)guide=candidate;
        for(double margin:{tracking_room_preference_,.16,.08,.04,0.}) {
          if(!narrow_reuse.empty()&&margin<tracking_room_preference_)continue;
          auto connected=visibility(candidate,yaw,margin,visibility_deadline);
          if(!connected.empty())return remember(message(connected,goal.header),goal);
        }
      }
      if(!narrow_reuse.empty()) {
        RCLCPP_INFO(_logger,"Reusing complete route with original footprint after bounded roomier-detour search");
        return remember(message(narrow_reuse,goal.header),goal);
      }
      // Recovery references share one budget; they do not recursively start
      // another expensive lattice repair for every possible local anchor.
      if(visibility_only)throw nav2_core::PlannerException("No complete visibility connection within recovery budget");
      if(guide.size()>1) {
        requested.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(heading(gx,gy,guide_heading(guide,true)));
        const double initial=heading(x,y,guide_heading(guide,false));
        pivot=terminal_path({x,y,yaw},{x,y,initial});
        if(clear(pivot))seed.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(initial);
        else pivot.clear();
      }else requested.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(heading(gx,gy,bearing));
    }
    // Repair invalid reference centres on a PRIVATE search copy. Physical
    // cells, body size, source timestamps and the published costmap do not change.
    auto *original=_costmap;
    nav2_costmap_2d::Costmap2D frozen;
    {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*original->getMutex());frozen=*original;}
    _costmap=&frozen;prepare_map();_costmap=original;
    nav2_costmap_2d::Costmap2D search(frozen);
    const auto saved_checker=_collision_checker;
    struct Restore {
      PositionOnlyLattice *self;nav2_costmap_2d::Costmap2D *map;nav2_smac_planner::GridCollisionChecker checker;
      ~Restore(){self->_costmap=map;self->_collision_checker=checker;self->_a_star->setCollisionChecker(&self->_collision_checker);}
    } restore{this,original,saved_checker};
    const auto began=std::chrono::steady_clock::now();
    for(int attempt=0;attempt<3;++attempt) {
      if(attempt&&std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>3.25)break;
      _costmap=&search;_collision_checker=nav2_smac_planner::GridCollisionChecker(&search,72u,_node.lock());
      auto path=SmacPlannerLattice::createPlan(seed,requested);
      _costmap=&frozen;
      if(path.poses.empty())break;
      auto prefix=message(pivot,path.header);if(prefix.poses.empty())prefix.poses.push_back(start);
      path.poses.insert(path.poses.begin(),prefix.poses.begin(),prefix.poses.end());
      if(!clear(pivot))throw nav2_core::PlannerException("Initial pivot changed during planning");
      std::vector<std::array<double,3>> bad;
      for(size_t i=0;i<path.poses.size();++i) {
        const auto &a=path.poses[i?i-1:i].pose;const auto &b=path.poses[i].pose;
        const double da=std::remainder(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation),2*std::acos(-1.));
        const int n=std::max({1,static_cast<int>(std::ceil(std::hypot(b.position.x-a.position.x,b.position.y-a.position.y)/.01)),static_cast<int>(std::ceil(std::abs(da)/.01))});
        for(int k=0;k<=n;++k){double f=static_cast<double>(k)/n;std::array<double,3> p{a.position.x+f*(b.position.x-a.position.x),a.position.y+f*(b.position.y-a.position.y),tf2::getYaw(a.orientation)+f*da};if(!clear(p[0],p[1],p[2]))bad.push_back(p);}
      }
      if(bad.empty())return remember(completed(std::move(path)),goal);
      unsigned added=0;
      for(const auto &p:bad){unsigned x,y;if(search.worldToMap(p[0],p[1],x,y))for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=static_cast<int>(x)+dx,ny=static_cast<int>(y)+dy;if(nx>=0&&ny>=0&&nx<static_cast<int>(search.getSizeInCellsX())&&ny<static_cast<int>(search.getSizeInCellsY())&&search.getCost(nx,ny)<253){search.setCost(nx,ny,253);++added;}}}
      RCLCPP_INFO(_logger,"Repair %d: %zu actual full-body intersections, %u search-only centre exclusions",attempt,bad.size(),added);
      if(!added)break;
    }
    throw nav2_core::PlannerException("No full-body position route in bounded repair search");
  }
};
}
