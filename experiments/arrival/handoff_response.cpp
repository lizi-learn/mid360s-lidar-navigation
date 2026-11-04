// Isolated ABI for the actual production pivot/input-ramp arithmetic only.
// No ROS, robot, network, device or filesystem interface.
#include "p2_nav_controller/settled_pivot.hpp"
#include "speed_response.hpp"
struct HandoffResponse {
  p2_nav_controller::SettledPivot pivot;
  p2_nav::SpeedResponse input;
};
extern "C" {
void *p2_handoff_create(){return new HandoffResponse;}
void p2_handoff_destroy(void *p){delete static_cast<HandoffResponse*>(p);}
double p2_handoff_pivot(void *p,double error,double yaw,double rate,double now){
  return static_cast<HandoffResponse*>(p)->pivot.step(error,yaw,rate,now,.03);
}
double p2_handoff_input(void *p,double yaw,double measured,double dt,unsigned long long stamp){
  return static_cast<HandoffResponse*>(p)->input.step(0.,yaw,0.,measured,dt,stamp,stamp,true).policy_yaw;
}
}
