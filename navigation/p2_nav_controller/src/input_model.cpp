// Simulation-only ABI around the same production manual-equivalent input ramp.
// No ROS, socket, device, filesystem or motor interface.
#include <new>
#include "speed_response.hpp"
extern "C" {
int p2_input_model_abi(){return 1;}
void *p2_input_model_create(){return new(std::nothrow) p2_nav::SpeedResponse;}
void p2_input_model_destroy(void *p){delete static_cast<p2_nav::SpeedResponse*>(p);}
unsigned p2_input_model_step(void *p,double v,double w,double mv,double mw,double dt,
    unsigned long long source,unsigned long long now,int owned,double *out){
  if(!p||!out)return p2_nav::speed_invalid_input;
  auto s=static_cast<p2_nav::SpeedResponse*>(p)->step(v,w,mv,mw,dt,source,now,owned!=0);
  out[0]=s.policy_x;out[1]=s.policy_yaw;return s.reason;
}
}
