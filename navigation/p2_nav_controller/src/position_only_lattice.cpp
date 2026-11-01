#include <cmath>
#include <mutex>
#include <functional>
#include <chrono>
#include <deque>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "p2_nav_controller/terminal_path.hpp"
#include "p2_nav_controller/position_guide.hpp"
namespace p2_nav_controller {
class PositionOnlyLattice : public nav2_smac_planner::SmacPlannerLattice {
  bool clear(double x,double y,double yaw) {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(_costmap);
    const auto body=_costmap_ros->getRobotFootprint();
    if(checker.footprintCostAtPose(x,y,yaw,body)>=254)return false;
    // Outline checking alone can miss an occupied cell inside a large body.
    std::vector<std::array<double,2>> polygon;
    double lx=x,ux=x,ly=y,uy=y;const double c=std::cos(yaw),s=std::sin(yaw);
    for(const auto &p:body){double px=x+c*p.x-s*p.y,py=y+s*p.x+c*p.y;polygon.push_back({px,py});lx=std::min(lx,px);ux=std::max(ux,px);ly=std::min(ly,py);uy=std::max(uy,py);}
    unsigned ax,ay,bx,by;
    if(!_costmap->worldToMap(lx,ly,ax,ay)||!_costmap->worldToMap(ux,uy,bx,by))return false;
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
  bool clear(const std::vector<std::array<double,3>> &samples) {
    for(const auto &p:samples)if(!clear(p[0],p[1],p[2]))return false;return true;
  }
  bool pivot_room(double x,double y) {
    double radius=0.;for(const auto &p:_costmap_ros->getRobotFootprint())radius=std::max(radius,std::hypot(p.x,p.y));
    const double r=_costmap->getResolution(),ox=_costmap->getOriginX(),oy=_costmap->getOriginY();
    const int lx=std::floor((x-radius-ox)/r),ux=std::floor((x+radius-ox)/r),ly=std::floor((y-radius-oy)/r),uy=std::floor((y+radius-oy)/r);
    if(lx<0||ly<0||ux>=static_cast<int>(_costmap->getSizeInCellsX())||uy>=static_cast<int>(_costmap->getSizeInCellsY()))return false;
    for(int iy=ly;iy<=uy;++iy)for(int ix=lx;ix<=ux;++ix)if(_costmap->getCost(ix,iy)>=254){double px,py;_costmap->mapToWorld(ix,iy,px,py);if(std::hypot(std::max(0.,std::abs(px-x)-r/2),std::max(0.,std::abs(py-y)-r/2))<=radius)return false;}
    return true;
  }
  std::vector<std::array<double,3>> visibility(const std::vector<std::array<double,2>> &guide,double yaw) {
    if(guide.empty())return {};
    std::vector<std::array<double,2>> points{guide.front()};
    for(size_t i=1;i+1<guide.size();++i)if(std::hypot(guide[i][0]-points.back()[0],guide[i][1]-points.back()[1])>=.25&&pivot_room(guide[i][0],guide[i][1]))points.push_back(guide[i]);
    points.push_back(guide.back());std::vector<bool> failed(points.size(),false);
    std::vector<std::array<double,3>> out{{guide.front()[0],guide.front()[1],yaw}};
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(750);
    std::function<bool(size_t)> visit=[&](size_t from) {
      if(std::chrono::steady_clock::now()>deadline)return false;
      for(size_t to=points.size()-1;to>from;--to){
        if(failed[to])continue;
        const auto &q=points[to];const auto pose=out.back();const double a=std::atan2(q[1]-pose[1],q[0]-pose[0]);
        auto leg=terminal_path(pose,{q[0],q[1],a},std::numeric_limits<double>::infinity());
        if(!clear(leg))continue;
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
    throw nav2_core::PlannerException("No full-body heading fits position endpoint");
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
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                const geometry_msgs::msg::PoseStamped &goal) override {
    if(start.header.frame_id!=goal.header.frame_id)throw nav2_core::PlannerException("Position goal frame mismatch");
    const double yaw=tf2::getYaw(start.pose.orientation),x=start.pose.position.x,y=start.pose.position.y;
    const double gx=goal.pose.position.x,gy=goal.pose.position.y;
    auto requested=goal;
    const double bearing=std::atan2(gy-y,gx-x);
    auto direct=terminal_path({x,y,yaw},{gx,gy,bearing},std::hypot(_costmap->getSizeInMetersX(),_costmap->getSizeInMetersY()));
    std::vector<std::array<double,2>> guide;std::vector<std::array<double,3>> pivot;
    auto seed=start;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
      if(clear(direct))return message(direct,goal.header);
      guide=position_guide(_costmap->getCharMap(),_costmap->getSizeInCellsX(),_costmap->getSizeInCellsY(),_costmap->getResolution(),_costmap->getOriginX(),_costmap->getOriginY(),{x,y},{gx,gy});
      // Point targets need no terminal heading. Checked pivot/straight
      // visibility connections also avoid a car-like turnaround search. Joints
      // are places where the unchanged body can pivot; narrow straight gaps
      // are crossed as an edge without requiring a turn inside them.
      auto connected=visibility(guide,yaw);
      if(!connected.empty())return message(connected,goal.header);
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
      if(bad.empty())return completed(std::move(path));
      unsigned added=0;
      for(const auto &p:bad){unsigned x,y;if(search.worldToMap(p[0],p[1],x,y))for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int nx=static_cast<int>(x)+dx,ny=static_cast<int>(y)+dy;if(nx>=0&&ny>=0&&nx<static_cast<int>(search.getSizeInCellsX())&&ny<static_cast<int>(search.getSizeInCellsY())&&search.getCost(nx,ny)<253){search.setCost(nx,ny,253);++added;}}}
      RCLCPP_INFO(_logger,"Repair %d: %zu actual full-body intersections, %u search-only centre exclusions",attempt,bad.size(),added);
      if(!added)break;
    }
    throw nav2_core::PlannerException("No full-body position route in bounded repair search");
  }
};
}
PLUGINLIB_EXPORT_CLASS(p2_nav_controller::PositionOnlyLattice,nav2_core::GlobalPlanner)
