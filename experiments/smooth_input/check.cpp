#include "input_s_curve.hpp"
using p2_nav::SmoothInputAxis;
#include <cassert>
#include <cmath>
#include <random>
#include <iostream>
int main(){
 for(double dt:{.002,.01,.02,.04,.07}){
  SmoothInputAxis r;double v=0.,a=0.;
  for(int k=0;k<static_cast<int>(4./dt);++k){double next=r.step(.75,dt,.30,.40),na=r.acceleration();assert(next>=v-1e-12&&next<=.75+1e-12);assert(std::abs(na)<=.30+1e-10);assert(std::abs(na-a)<=.40*dt+1e-10);v=next;a=na;}
  assert(std::abs(v-.75)<1e-10&&std::abs(a)<1e-10);
  for(int k=0;k<static_cast<int>(4./dt);++k){double next=r.step(0.,dt,.30,.40),na=r.acceleration();assert(next<=v+1e-12&&next>=-1e-12);assert(std::abs(na)<=.30+1e-10);assert(std::abs(na-a)<=.40*dt+1e-10);v=next;a=na;}
  assert(std::abs(v)<1e-10&&std::abs(a)<1e-10);
 }
 // Changing targets preserve acceleration, including reversal while braking.
 std::mt19937 gen(20261105);std::uniform_real_distribution<double> target(-1.,1.),duration(.002,.08);
 SmoothInputAxis r;double a=0.,goal=0.;
 for(int k=0;k<200000;++k){if(k%7==0)goal=target(gen);double dt=duration(gen);double v=r.step(goal,dt,.4,.6),na=r.acceleration();assert(std::isfinite(v)&&std::abs(v)<=1.+1e-10);assert(std::abs(na)<=.4+1e-10);assert(std::abs(na-a)<=.6*dt+1e-10);a=na;}
 std::cout<<"start, normal stop, moving targets and200000 seeded steps pass\n";
}
