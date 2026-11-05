// Synthetic planar comparison with the ACTUAL installed own RPP plugin.
// No robot model, motor API, socket client, recorded office map or hardware I/O.
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <map>
#include <limits>
#include <dlfcn.h>
#include "nav2_core/global_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "nav2_core/progress_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"
#include "speed_response.hpp"
#include "p2_nav_controller/boundary_escape.hpp"

constexpr double pi=3.14159265358979323846,dt=.05,half=.275;
struct Point{double x,y;double yaw=std::numeric_limits<double>::quiet_NaN();};
double wrap(double a){return std::remainder(a,2*pi);}
std::vector<Point> path_points(const std::string &kind){
 std::vector<Point> p;
 if(const char *input=std::getenv("P2_TEST_REPLAY_CSV")){std::ifstream in(input);std::string line;std::getline(in,line);while(std::getline(in,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);Point q;if(row>>q.x>>q.y>>q.yaw)p.push_back(q);}if(p.empty())throw std::runtime_error("Empty replay path");return p;}
 if(kind=="corner"){
  for(int i=0;i<=100;i++)p.push_back({i*.05,0.});
  for(int i=1;i<=60;i++){double a=-pi/2+i*pi/120;p.push_back({5+1.5*std::cos(a),1.5+1.5*std::sin(a)});}
  for(int i=1;i<=140;i++)p.push_back({6.5,1.5+i*.05});
 }else for(int i=0;i<=400;i++){double x=i*.05;p.push_back({x,kind=="gentle_s"?1.2*std::pow(std::sin(x*pi/10.),3):0.});}
 return p;
}
bool cell_overlap(double x,double y,double yaw,double cx,double cy,double h){
 double c=std::cos(yaw),s=std::sin(yaw),dx=cx-x,dy=cy-y;
 // Separating axes of both the full rotated body and the cell, including interiors.
 return std::abs(dx)<=half*(std::abs(c)+std::abs(s))+h &&
        std::abs(dy)<=half*(std::abs(c)+std::abs(s))+h &&
        std::abs(c*dx+s*dy)<=half+h*(std::abs(c)+std::abs(s)) &&
        std::abs(-s*dx+c*dy)<=half+h*(std::abs(c)+std::abs(s));
}
int main(int argc,char **argv){
 if(argc<5){std::cerr<<"usage: fixture output-prefix path-kind gain delay [--ros-args ...]\n";return 2;}
 const std::string prefix=argv[1],kind=argv[2];const double gain=std::stod(argv[3]),delay=std::stod(argv[4]);
 const double yaw_deadzone=std::getenv("P2_TEST_YAW_DEADZONE")?std::stod(std::getenv("P2_TEST_YAW_DEADZONE")):0.;
 const double pose_xy_noise=std::getenv("P2_TEST_POSE_XY_NOISE")?std::stod(std::getenv("P2_TEST_POSE_XY_NOISE")):0.;
 if(!std::isfinite(pose_xy_noise)||pose_xy_noise<0.||pose_xy_noise>.02)throw std::runtime_error("Invalid synthetic XY pose noise");
 const double pose_yaw_noise=std::getenv("P2_TEST_POSE_YAW_NOISE")?std::stod(std::getenv("P2_TEST_POSE_YAW_NOISE")):0.;
 if(!std::isfinite(pose_yaw_noise)||pose_yaw_noise<0.||pose_yaw_noise>.1)throw std::runtime_error("Invalid synthetic pose yaw noise");
 const double input_scale=std::getenv("P2_TEST_INPUT_SCALE")?std::stod(std::getenv("P2_TEST_INPUT_SCALE")):1.;
 const double x_deadzone=std::getenv("P2_TEST_X_DEADZONE")?std::stod(std::getenv("P2_TEST_X_DEADZONE")):0.;
 if(!std::isfinite(input_scale)||input_scale<=0.||input_scale>1.||!std::isfinite(x_deadzone)||x_deadzone<0.||x_deadzone>=.75)throw std::runtime_error("Invalid synthetic input scale/deadzone");
 const double lateral_ratio=std::getenv("P2_TEST_LATERAL_RATIO")?std::stod(std::getenv("P2_TEST_LATERAL_RATIO")):0.;
 const double lateral_ripple=std::getenv("P2_TEST_LATERAL_RIPPLE")?std::stod(std::getenv("P2_TEST_LATERAL_RIPPLE")):0.;
 if(!std::isfinite(lateral_ripple)||lateral_ripple<0.||lateral_ripple>.3)throw std::runtime_error("Invalid synthetic lateral ripple");
 if(!std::isfinite(lateral_ratio)||std::abs(lateral_ratio)>.5)throw std::runtime_error("Invalid synthetic lateral drift");
 if(!std::isfinite(yaw_deadzone)||yaw_deadzone<0.||yaw_deadzone>=1.)throw std::runtime_error("Invalid synthetic yaw deadzone");
 rclcpp::init(argc,argv);
 int result=0;
 try{
 auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("preview_controller");
 if(rcl_enable_ros_time_override(parent->get_clock()->get_clock_handle())!=RCL_RET_OK||rcl_set_ros_time_override(parent->get_clock()->get_clock_handle(),1000000000LL)!=RCL_RET_OK)throw std::runtime_error("Fixture clock setup failed");
 auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("preview_costmap");
 cm->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[0.275,0.275],[0.275,-0.275],[-0.275,-0.275],[-0.275,0.275]]"),rclcpp::Parameter("footprint_padding",0.),rclcpp::Parameter("resolution",.05),rclcpp::Parameter("width",40),rclcpp::Parameter("height",40)});
 if(cm->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)throw std::runtime_error("costmap configure failed");
 auto map=cm->getCostmap();map->resizeMap(800,800,.05,-10.,-10.);
 auto paint=[&](bool obstacle,double ox,double oy){
  if(const char *input=std::getenv("P2_TEST_REPLAY_MAP")){std::ifstream in(input);unsigned w,h;double r,ox,oy;in>>w>>h>>r>>ox>>oy;map->resizeMap(w,h,r,ox,oy);for(unsigned i=0;i<w*h;++i){int c;if(!(in>>c))throw std::runtime_error("Incomplete replay map");map->getCharMap()[i]=c;}return;}
  std::fill(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY(),0);
  if(kind=="corner")for(unsigned iy=0;iy<map->getSizeInCellsY();++iy)for(unsigned ix=0;ix<map->getSizeInCellsX();++ix){double x,y;map->mapToWorld(ix,iy,x,y);if(y<-.9||x>7.4||(x<5.6&&y>.9))map->setCost(ix,iy,254);}
  if(obstacle)for(unsigned iy=0;iy<map->getSizeInCellsY();++iy)for(unsigned ix=0;ix<map->getSizeInCellsX();++ix){double x,y;map->mapToWorld(ix,iy,x,y);if(std::abs(x-ox)<.12&&std::abs(y-oy)<.3)map->setCost(ix,iy,254);}
 };
 paint(false,0,0);
 bool static_probe_verified=false;
 if(const char *probe=std::getenv("P2_TEST_OBSTACLE_PROBE")){
   std::istringstream input(probe);double px,py;unsigned ix,iy;
   if(!(input>>px>>py)||!map->worldToMap(px,py,ix,iy)||map->getCost(ix,iy)!=254)
     throw std::runtime_error("Static obstacle probe did not reach actual costmap");
   static_probe_verified=true;
 }
 auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
 // A real listener thread satisfies the timed-transform API contract. Direct
 // fixture transforms are still inserted synchronously before every call.
 auto listener=std::make_shared<tf2_ros::TransformListener>(*tf);
 pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
 auto control=loader.createSharedInstance("p2_nav_controller::SettledGoalRPP");control->configure(parent,"FollowPath",tf,cm);control->activate();
 Dl_info controller_image{};
 if(!dladdr(*reinterpret_cast<void **>(control.get()),&controller_image)||!controller_image.dli_fname)
   throw std::runtime_error("Loaded controller library identity unavailable");
 const std::string controller_library=controller_image.dli_fname;
 std::ofstream actual(prefix+".actual-parameters.json");actual<<"{";
 const std::vector<std::string> names={"min_lookahead_dist","max_lookahead_dist","lookahead_time","rotate_to_heading_angular_vel","rotate_to_heading_min_angle","regulated_linear_scaling_min_speed","use_collision_detection","allow_reversing","corner_preview","curved_forward_input","minimum_pivot_input","settled_pivot_response","terminal_input_response","terminal_approach_continuity","boundary_forward_escape"};
 for(size_t i=0;i<names.size();++i){if(i)actual<<",";actual<<"\""<<names[i]<<"\":"<<parent->get_parameter("FollowPath."+names[i]).value_to_string();}
 actual<<",\"actual_footprint\":[";auto footprint=cm->getRobotFootprint();for(size_t i=0;i<footprint.size();++i){if(i)actual<<",";actual<<"["<<footprint[i].x<<","<<footprint[i].y<<"]";}actual<<"]}";actual.close();
 pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
 const bool position_only=std::getenv("P2_TEST_POSITION_GOAL")!=nullptr;
 auto checker=goals.createSharedInstance(position_only?"p2_nav_controller::PositionGoalChecker":"nav2_controller::SimpleGoalChecker");checker->initialize(parent,"goal_checker",cm);
 auto points=path_points(kind);nav_msgs::msg::Path path;path.header.frame_id="fixture_world";
 for(size_t i=0;i<points.size();++i){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=points[i].x;p.pose.position.y=points[i].y;auto a=points[i?i-1:i],b=points[std::min(i+1,points.size()-1)];p.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(std::isfinite(points[i].yaw)?points[i].yaw:std::atan2(b.y-a.y,b.x-a.x));path.poses.push_back(p);}
 if(position_only&&!std::getenv("P2_TEST_REPLAY_CSV"))path.poses.back().pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(pi);
 control->setPlan(path);
 const auto requested_goal=path.poses.back();
 pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_loader("nav2_core","nav2_core::ProgressChecker");
 auto actual_progress=progress_loader.createSharedInstance("nav2_controller::SimpleProgressChecker");actual_progress->initialize(parent,"progress_checker");
 pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader("nav2_core","nav2_core::GlobalPlanner");
 auto planner=planner_loader.createSharedInstance("p2_nav_controller::PositionOnlyLattice");planner->configure(parent,"GridBased",tf,cm);planner->activate();
 bool planning_failed=false;int planning_failures=0,replans=0;std::map<std::string,int> plan_errors,control_errors;std::ofstream diagnostics(prefix+".diagnostics.log");
 double x=0.,y=.25,yaw=.15,v=0.,w=0.,time=0.,rms=0.,max_error=0.,last_sign=0.,progress=0.;int counted=0,reversals=0,collisions=0,stops=0;bool reached=false,obstacle=false,triggered=false,verified_obstacle=false,cleared=false;double obstacle_at=-1,ox=0,oy=0;
 if(std::getenv("P2_TEST_REPLAY_CSV")){x=points.front().x;y=points.front().y;yaw=points.front().yaw;}
 const bool boundary_case=std::getenv("P2_TEST_BOUNDARY_ESCAPE")!=nullptr;
 const auto initial_body=p2_nav_controller::boundary_geometry::at(cm->getRobotFootprint(),x,y,yaw);
 int existing_overlap_samples=0;
 p2_nav::SpeedResponse native;
 std::array<double,2> queued{0.,0.};
 if(const char *initial=std::getenv("P2_TEST_INITIAL_SPEED")) {
   std::istringstream row(initial);double px,pw;
   if(!(row>>v>>w>>px>>pw))throw std::runtime_error("Invalid initial measured/native response");
   for(int i=0;i<100;++i)native.step(px,pw,v,w,dt,1000000000ULL,1000000000ULL,true);
   queued={native.state().policy_x,native.state().policy_yaw};
 }
 std::deque<std::array<double,2>> latency(static_cast<size_t>(std::round(delay/dt))+1,queued);
 std::ofstream trace(prefix+".csv");trace<<"t,x,y,yaw,raw_x,raw_z,input_x,input_z,measured_x,measured_z,reference_error,collision_stop,obstacle\n";
 for(int step=0;step<2400;++step){
  time=step*dt;
  if(rcl_set_ros_time_override(parent->get_clock()->get_clock_handle(),static_cast<int64_t>((time+1.)*1e9))!=RCL_RET_OK)throw std::runtime_error("Fixture clock tick failed");
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="fixture_world";pose.header.stamp=parent->now();pose.pose.position.x=x+pose_xy_noise*std::sin(4*pi*time);pose.pose.position.y=y+pose_xy_noise*std::sin(4*pi*time+.3);pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw+pose_yaw_noise*std::sin(10.*pi*time));
  if(!actual_progress->check(pose)){std::cout<<"Actual configured progress allowance exceeded\n";break;}
  geometry_msgs::msg::TransformStamped tr;tr.header=pose.header;tr.child_frame_id="fixture_base";tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation=pose.pose.orientation;if(!tf->setTransform(tr,"synthetic_planar_fixture"))throw std::runtime_error("fixture TF rejected");
  const double ripple=lateral_ripple*std::sin(4*pi*time);
  geometry_msgs::msg::Twist measured;measured.linear.x=v;measured.linear.y=lateral_ratio*v+ripple;measured.angular.z=w;
  if(step%20==0&&!std::getenv("P2_TEST_FIXED_REFERENCE")){try{auto fresh=planner->createPlan(pose,requested_goal);if(fresh.poses.empty())throw std::runtime_error("Empty replan");control->setPlan(fresh);planning_failed=false;++replans;}catch(const std::exception &e){planning_failed=true;++planning_failures;if(++plan_errors[e.what()]==1)diagnostics<<"plan t="<<time<<" "<<e.what()<<std::endl;}}
  if(checker->isGoalReached(pose.pose,requested_goal.pose,measured)){reached=true;break;}
  if(std::getenv("P2_TEST_BOUNDARY_OBSTACLE")&&!triggered&&time>=.25) {
    triggered=true;obstacle=true;obstacle_at=time;ox=.75;oy=0.;
    for(unsigned iy=0;iy<map->getSizeInCellsY();++iy)for(unsigned ix=0;ix<map->getSizeInCellsX();++ix){double px,py;map->mapToWorld(ix,iy,px,py);if(std::abs(px-ox)<.08&&std::abs(py-oy)<.15)map->setCost(ix,iy,254);}
    unsigned ix,iy;if(!map->worldToMap(ox,oy,ix,iy)||map->getCost(ix,iy)!=254)throw std::runtime_error("Boundary forward obstacle not applied");verified_obstacle=true;
  }
  if(kind=="obstacle"&&!triggered&&x>=6.){triggered=true;obstacle=true;obstacle_at=time;ox=x+.95;oy=0.;paint(true,ox,oy);unsigned ix,iy;if(!map->worldToMap(ox,oy,ix,iy)||map->getCost(ix,iy)!=254)throw std::runtime_error("obstacle did not reach actual costmap");verified_obstacle=true;}
  if(obstacle&&time-obstacle_at>=2.){obstacle=false;cleared=true;paint(false,0,0);}
  geometry_msgs::msg::Twist raw;bool stopped=false;
  try{if(planning_failed)throw nav2_core::PlannerException("No replay plan");raw=control->computeVelocityCommands(pose,measured,checker.get()).twist;}
  catch(const nav2_core::PlannerException &e){stopped=true;if(++control_errors[e.what()]==1)diagnostics<<"control t="<<time<<" "<<e.what()<<std::endl;if(std::string(e.what()).find("collision")!=std::string::npos)++stops;raw=geometry_msgs::msg::Twist();}
  if(raw.linear.y!=0.)throw std::runtime_error("Unexpected lateral command");
  auto stamp=static_cast<std::uint64_t>((time+1)*1e9);auto input=native.step(std::min(.75,raw.linear.x)*input_scale,raw.angular.z*input_scale,v,w,dt,stamp,stamp,true);
  // Already queued physical response drains after a stop. Erasing it hid the
  // overshoot being measured by this delayed-plant regression.
  latency.push_back({input.policy_x,input.policy_yaw});auto applied=latency.front();latency.pop_front();
  const double response_yaw=std::abs(applied[1])<yaw_deadzone?0.:gain*applied[1];
  const double response_x=std::abs(applied[0])<x_deadzone?0.:.95*applied[0];
  v+=(response_x-v)*(1-std::exp(-dt/.2));w+=(response_yaw-w)*(1-std::exp(-dt/.15));
  const double old_x=x,old_y=y,old_yaw=yaw;
  const double sideways=lateral_ratio*v+ripple;
  x+=dt*(v*std::cos(yaw+w*dt/2)-sideways*std::sin(yaw+w*dt/2));y+=dt*(v*std::sin(yaw+w*dt/2)+sideways*std::cos(yaw+w*dt/2));yaw=wrap(yaw+w*dt);
  double best=1e30;size_t nearest=0;for(size_t j=0;j+1<points.size();++j){auto a=points[j],b=points[j+1];double dx=b.x-a.x,dy=b.y-a.y;if(dx*dx+dy*dy<1e-15)continue;double f=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0.,1.);double dist=std::hypot(x-a.x-f*dx,y-a.y-f*dy);if(dist<best){best=dist;nearest=j;}}
  progress=points[nearest].x;
  if(time>5.){rms+=best*best;max_error=std::max(max_error,best);++counted;if(std::abs(input.policy_yaw)>.15){double sign=std::copysign(1.,input.policy_yaw);if(last_sign&&sign!=last_sign)++reversals;last_sign=sign;}}
  bool intrusion=false;
  for(int sub=1;sub<=5&&!intrusion;++sub){
  const double f=sub/5.,sx=old_x+(x-old_x)*f,sy=old_y+(y-old_y)*f,syaw=old_yaw+wrap(yaw-old_yaw)*f;
  int lx=static_cast<int>((sx-.425-map->getOriginX())/.05),ux=static_cast<int>((sx+.425-map->getOriginX())/.05),ly=static_cast<int>((sy-.425-map->getOriginY())/.05),uy=static_cast<int>((sy+.425-map->getOriginY())/.05);
  for(int iy=ly;iy<=uy&&!intrusion;iy++)for(int ix=lx;ix<=ux;ix++){if(ix<0||iy<0||ix>=static_cast<int>(map->getSizeInCellsX())||iy>=static_cast<int>(map->getSizeInCellsY())){intrusion=true;break;}if(map->getCost(ix,iy)>=254){double cx,cy;map->mapToWorld(ix,iy,cx,cy);if(cell_overlap(sx,sy,syaw,cx,cy,.025)){
      bool existing=false;
      if(boundary_case&&map->getCost(ix,iy)==254){
        using namespace p2_nav_controller::boundary_geometry;
        auto current=at(cm->getRobotFootprint(),sx,sy,syaw);auto combined=initial_body;combined.insert(combined.end(),current.begin(),current.end());
        const double before=area(cell_clip(initial_body,cx-.025,cy-.025,.05));
        existing=before>1e-12&&area(cell_clip(hull(combined),cx-.025,cy-.025,.05))<=before+1e-10;
      }
      if(existing)++existing_overlap_samples;else{intrusion=true;break;}
    }}}
  }
  if(intrusion)++collisions;
  trace<<std::setprecision(10)<<time<<','<<x<<','<<y<<','<<yaw<<','<<raw.linear.x<<','<<raw.angular.z<<','<<input.policy_x<<','<<input.policy_yaw<<','<<v<<','<<w<<','<<best<<','<<stopped<<','<<obstacle<<'\n';
  if(intrusion)break;
 }
 double error=std::hypot(x-points.back().x,y-points.back().y),yaw_error=std::abs(wrap(yaw-tf2::getYaw(path.poses.back().pose.orientation)));
 std::ofstream out(prefix+".json");out<<std::boolalpha<<"{\"loaded_controller_library\":\""<<controller_library<<"\",\"actual_plugin\":\"p2_nav_controller::SettledGoalRPP\",\"scope\":\"Synthetic planar lag/gain model, no biped dynamics or hardware\",\"seed\":0,\"kind\":\""<<kind<<"\",\"gain\":"<<gain<<",\"delay_s\":"<<delay<<",\"yaw_deadzone\":"<<yaw_deadzone<<",\"lateral_ratio\":"<<lateral_ratio<<",\"lateral_ripple_mps\":"<<lateral_ripple<<",\"static_obstacle_probe_verified\":"<<static_probe_verified<<",\"lateral_ripple_hz\":2,\"commanded_y\":0,\"dt_s\":"<<dt<<",\"duration_s\":"<<time<<",\"goal_reached\":"<<reached<<",\"goal_error_m\":"<<error<<",\"goal_yaw_error_rad\":"<<yaw_error<<",\"cross_track_rms_m\":"<<std::sqrt(rms/std::max(1,counted))<<",\"cross_track_max_m\":"<<max_error<<",\"yaw_reversals\":"<<reversals<<",\"existing_boundary_overlap_samples\":"<<existing_overlap_samples<<",\"body_intrusions\":"<<collisions<<",\"controller_collision_stops\":"<<stops<<",\"obstacle_injected\":"<<triggered<<",\"obstacle_verified_in_costmap\":"<<verified_obstacle<<",\"obstacle_cleared_after_two_seconds\":"<<cleared<<",\"progress_x\":"<<progress<<"}\n";
 std::cout<<kind<<" reached="<<reached<<" error="<<error<<" intrusions="<<collisions<<" reversals="<<reversals<<'\n';
 std::cout<<"replans="<<replans<<" failed="<<planning_failures<<"\n";planner->deactivate();planner->cleanup();planner.reset();
 control->deactivate();control->cleanup();control.reset();actual_progress.reset();checker.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();parent.reset();
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
 rclcpp::shutdown();return result;
}
