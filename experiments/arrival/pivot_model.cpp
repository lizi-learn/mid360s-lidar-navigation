#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "pivot_breakaway.hpp"
#include "terminal_velocity.hpp"
#include "speed_response.hpp"
int main(int argc,char **argv){
 if(argc<2)return 2;std::ofstream out(argv[1]);out<<"minimum,gain,delay,deadzone,goal,reached,settled_error,pulses\n";int failed=0;
 for(double minimum:{0.,.5})for(double gain:{.8,1.2})for(double delay:{.3,.8,1.1})for(double dz:{.28,.35})for(double goal:{-.3,.3,1.0}){
  p2_nav_controller::PivotBreakaway assist;p2_nav::SpeedResponse native;std::deque<double> latency(static_cast<int>(std::round(delay/.05))+1,0.);double yaw=0,rate=0;int reached=-1,pulses=0;
  for(int k=0;k<600;++k){double error=std::remainder(goal-yaw,2*std::acos(-1.));if(reached<0&&std::abs(error)<.15)reached=k;
   double raw=0.;if(reached<0){raw=p2_nav_controller::terminal_velocity(0.,0.,error,true,1.)[1];double next=assist.step(raw,error,rate,.15,minimum,1.,1.+k*.05);if(std::abs(next)>std::abs(raw)+1e-6)++pulses;raw=next;}
   auto stamp=static_cast<std::uint64_t>((1.+k*.05)*1e9);auto input=native.step(0,raw,0,rate,.05,stamp,stamp,true);latency.push_back(input.policy_yaw);double command=latency.front();latency.pop_front();
   double target=std::abs(command)<dz?0.:gain*command;rate+=(target-rate)*(1-std::exp(-.05/.15));yaw+=rate*.05;
   if(reached>=0&&k-reached>=60)break;
  }
  double err=std::abs(std::remainder(goal-yaw,2*std::acos(-1.)));out<<minimum<<','<<gain<<','<<delay<<','<<dz<<','<<goal<<','<<(reached>=0)<<','<<std::setprecision(9)<<err<<','<<pulses<<'\n';if(minimum>0&&std::abs(goal)<.5&&(!(reached>=0)||err>.15))++failed;
 }
 std::cout<<"candidate failed "<<failed<<"/24; full physical plant and timestamps not modeled\n";return failed?1:0;
}
