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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "nav2_util/geometry_utils.hpp"
#include "speed_response.hpp"

constexpr double pi=3.14159265358979323846,dt=.05,half=.275;
struct Point{double x,y;};
double wrap(double a){return std::remainder(a,2*pi);}
std::vector<Point> path_points(const std::string &kind){
 std::vector<Point> p;
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
 rclcpp::init(argc,argv);
 int result=0;
 try{
 auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("preview_controller");
 auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("preview_costmap");
 cm->set_parameters({rclcpp::Parameter("global_frame","fixture_world"),rclcpp::Parameter("robot_base_frame","fixture_base"),rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("footprint","[[0.275,0.275],[0.275,-0.275],[-0.275,-0.275],[-0.275,0.275]]"),rclcpp::Parameter("footprint_padding",0.),rclcpp::Parameter("resolution",.05),rclcpp::Parameter("width",40),rclcpp::Parameter("height",40)});
 if(cm->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS)throw std::runtime_error("costmap configure failed");
 auto map=cm->getCostmap();map->resizeMap(800,800,.05,-10.,-10.);
 auto paint=[&](bool obstacle,double ox,double oy){
  std::fill(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY(),0);
  if(kind=="corner")for(unsigned iy=0;iy<map->getSizeInCellsY();++iy)for(unsigned ix=0;ix<map->getSizeInCellsX();++ix){double x,y;map->mapToWorld(ix,iy,x,y);if(y<-.9||x>7.4||(x<5.6&&y>.9))map->setCost(ix,iy,254);}
  if(obstacle)for(unsigned iy=0;iy<map->getSizeInCellsY();++iy)for(unsigned ix=0;ix<map->getSizeInCellsX();++ix){double x,y;map->mapToWorld(ix,iy,x,y);if(std::abs(x-ox)<.12&&std::abs(y-oy)<.3)map->setCost(ix,iy,254);}
 };
 paint(false,0,0);
 auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
 pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
 auto control=loader.createSharedInstance("p2_nav_controller::SettledGoalRPP");control->configure(parent,"FollowPath",tf,cm);control->activate();
 std::ofstream actual(prefix+".actual-parameters.json");actual<<"{";
 const std::vector<std::string> names={"min_lookahead_dist","max_lookahead_dist","lookahead_time","rotate_to_heading_angular_vel","rotate_to_heading_min_angle","regulated_linear_scaling_min_speed","use_collision_detection","allow_reversing"};
 for(size_t i=0;i<names.size();++i){if(i)actual<<",";actual<<"\""<<names[i]<<"\":"<<parent->get_parameter("FollowPath."+names[i]).value_to_string();}
 actual<<",\"actual_footprint\":[";auto footprint=cm->getRobotFootprint();for(size_t i=0;i<footprint.size();++i){if(i)actual<<",";actual<<"["<<footprint[i].x<<","<<footprint[i].y<<"]";}actual<<"]}";actual.close();
 pluginlib::ClassLoader<nav2_core::GoalChecker> goals("nav2_core","nav2_core::GoalChecker");
 auto checker=goals.createSharedInstance("nav2_controller::SimpleGoalChecker");checker->initialize(parent,"goal_checker",cm);
 auto points=path_points(kind);nav_msgs::msg::Path path;path.header.frame_id="fixture_world";
 for(size_t i=0;i<points.size();++i){geometry_msgs::msg::PoseStamped p;p.header=path.header;p.pose.position.x=points[i].x;p.pose.position.y=points[i].y;auto a=points[i?i-1:i],b=points[std::min(i+1,points.size()-1)];p.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(b.y-a.y,b.x-a.x));path.poses.push_back(p);}
 control->setPlan(path);
 double x=0.,y=.25,yaw=.15,v=0.,w=0.,time=0.,rms=0.,max_error=0.,last_sign=0.,progress=0.;int counted=0,reversals=0,collisions=0,stops=0;bool reached=false,obstacle=false,triggered=false,verified_obstacle=false,cleared=false;double obstacle_at=-1,ox=0,oy=0;
 p2_nav::SpeedResponse native;std::deque<std::array<double,2>> latency(static_cast<size_t>(std::round(delay/dt))+1,{0.,0.});
 std::ofstream trace(prefix+".csv");trace<<"t,x,y,yaw,raw_x,raw_z,input_x,input_z,measured_x,measured_z,reference_error,collision_stop,obstacle\n";
 for(int step=0;step<2400;++step){
  time=step*dt;
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="fixture_world";pose.header.stamp=parent->now();pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  geometry_msgs::msg::TransformStamped tr;tr.header=pose.header;tr.child_frame_id="fixture_base";tr.transform.translation.x=x;tr.transform.translation.y=y;tr.transform.rotation=pose.pose.orientation;if(!tf->setTransform(tr,"synthetic_planar_fixture"))throw std::runtime_error("fixture TF rejected");
  geometry_msgs::msg::Twist measured;measured.linear.x=v;measured.angular.z=w;
  if(checker->isGoalReached(pose.pose,path.poses.back().pose,measured)){reached=true;break;}
  if(kind=="obstacle"&&!triggered&&x>=6.){triggered=true;obstacle=true;obstacle_at=time;ox=x+.95;oy=0.;paint(true,ox,oy);unsigned ix,iy;if(!map->worldToMap(ox,oy,ix,iy)||map->getCost(ix,iy)!=254)throw std::runtime_error("obstacle did not reach actual costmap");verified_obstacle=true;}
  if(obstacle&&time-obstacle_at>=2.){obstacle=false;cleared=true;paint(false,0,0);}
  geometry_msgs::msg::Twist raw;bool stopped=false;
  try{raw=control->computeVelocityCommands(pose,measured,checker.get()).twist;}
  catch(const nav2_core::PlannerException &){stopped=true;++stops;raw={};}
  auto stamp=static_cast<std::uint64_t>((time+1)*1e9);auto input=native.step(std::min(.75,raw.linear.x),raw.angular.z,v,w,dt,stamp,stamp,true);
  // Hard stop clears command latency; the plant still decelerates, it is not teleported.
  if(stopped||(raw.linear.x==0.&&raw.angular.z==0.))for(auto &item:latency)item={0.,0.};
  latency.push_back({input.policy_x,input.policy_yaw});auto applied=latency.front();latency.pop_front();
  v+=(.95*applied[0]-v)*(1-std::exp(-dt/.2));w+=(gain*applied[1]-w)*(1-std::exp(-dt/.15));
  const double old_x=x,old_y=y,old_yaw=yaw;
  x+=v*dt*std::cos(yaw+w*dt/2);y+=v*dt*std::sin(yaw+w*dt/2);yaw=wrap(yaw+w*dt);
  double best=1e30;size_t nearest=0;for(size_t j=0;j+1<points.size();++j){auto a=points[j],b=points[j+1];double dx=b.x-a.x,dy=b.y-a.y,f=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(dx*dx+dy*dy),0.,1.);double dist=std::hypot(x-a.x-f*dx,y-a.y-f*dy);if(dist<best){best=dist;nearest=j;}}
  progress=points[nearest].x;
  if(time>5.){rms+=best*best;max_error=std::max(max_error,best);++counted;if(std::abs(input.policy_yaw)>.15){double sign=std::copysign(1.,input.policy_yaw);if(last_sign&&sign!=last_sign)++reversals;last_sign=sign;}}
  bool intrusion=false;
  for(int sub=1;sub<=5&&!intrusion;++sub){
  const double f=sub/5.,sx=old_x+(x-old_x)*f,sy=old_y+(y-old_y)*f,syaw=old_yaw+wrap(yaw-old_yaw)*f;
  int lx=static_cast<int>((sx-.425-map->getOriginX())/.05),ux=static_cast<int>((sx+.425-map->getOriginX())/.05),ly=static_cast<int>((sy-.425-map->getOriginY())/.05),uy=static_cast<int>((sy+.425-map->getOriginY())/.05);
  for(int iy=ly;iy<=uy&&!intrusion;iy++)for(int ix=lx;ix<=ux;ix++){if(ix<0||iy<0||ix>=static_cast<int>(map->getSizeInCellsX())||iy>=static_cast<int>(map->getSizeInCellsY())){intrusion=true;break;}if(map->getCost(ix,iy)>=253){double cx,cy;map->mapToWorld(ix,iy,cx,cy);if(cell_overlap(sx,sy,syaw,cx,cy,.025)){intrusion=true;break;}}}
  }
  if(intrusion)++collisions;
  trace<<std::setprecision(10)<<time<<','<<x<<','<<y<<','<<yaw<<','<<raw.linear.x<<','<<raw.angular.z<<','<<input.policy_x<<','<<input.policy_yaw<<','<<v<<','<<w<<','<<best<<','<<stopped<<','<<obstacle<<'\n';
  if(intrusion)break;
 }
 double error=std::hypot(x-points.back().x,y-points.back().y),yaw_error=std::abs(wrap(yaw-tf2::getYaw(path.poses.back().pose.orientation)));
 std::ofstream out(prefix+".json");out<<std::boolalpha<<"{\"actual_plugin\":\"p2_nav_controller::SettledGoalRPP\",\"scope\":\"Synthetic planar lag/gain model, no biped dynamics or hardware\",\"seed\":0,\"kind\":\""<<kind<<"\",\"gain\":"<<gain<<",\"delay_s\":"<<delay<<",\"dt_s\":"<<dt<<",\"duration_s\":"<<time<<",\"goal_reached\":"<<reached<<",\"goal_error_m\":"<<error<<",\"goal_yaw_error_rad\":"<<yaw_error<<",\"cross_track_rms_m\":"<<std::sqrt(rms/std::max(1,counted))<<",\"cross_track_max_m\":"<<max_error<<",\"yaw_reversals\":"<<reversals<<",\"body_intrusions\":"<<collisions<<",\"controller_collision_stops\":"<<stops<<",\"obstacle_injected\":"<<triggered<<",\"obstacle_verified_in_costmap\":"<<verified_obstacle<<",\"obstacle_cleared_after_two_seconds\":"<<cleared<<",\"progress_x\":"<<progress<<"}\n";
 std::cout<<kind<<" reached="<<reached<<" error="<<error<<" intrusions="<<collisions<<" reversals="<<reversals<<'\n';
 control->deactivate();control->cleanup();control.reset();checker.reset();cm->on_cleanup(rclcpp_lifecycle::State());cm.reset();parent.reset();
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
 rclcpp::shutdown();return result;
}
