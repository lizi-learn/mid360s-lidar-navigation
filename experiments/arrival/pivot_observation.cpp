#include <cmath>
#include <iostream>
#include <stdexcept>
#include "p2_nav_controller/settled_pivot.hpp"

int main() {
  using p2_nav_controller::SettledPivot;
  auto pulse=[&](double displacement,double peak) {
    SettledPivot p;double next=0.;
    for(int i=0;i<=29;++i) {
      const double t=1.+i*.05;
      next=p.step(.3,i<23?0.:displacement,i<23?peak:0.,t);
    }
    return std::pair<SettledPivot,double>{p,next};
  };
  auto noisy=pulse(.03,.05);
  if(noisy.second!=1.)throw std::runtime_error("Standing jitter accepted as useful turn");
  if(noisy.first.step(.3,.03,0.,3.1)!=1.)throw std::runtime_error("Callback gap lost learned input");
  if(noisy.first.step(0.,.03,0.,3.6)!=0.)throw std::runtime_error("Gap restarted aligned pivot");
  noisy.first.reset();
  if(noisy.first.step(.3,.03,0.,4.1)!=.5)throw std::runtime_error("Explicit reset retained old goal learning");
  auto useful=pulse(.12,.3);
  if(useful.second!=.5)throw std::runtime_error("Useful small pulse unnecessarily escalated");
  auto away=pulse(-.04,.2);
  if(away.second!=1.)throw std::runtime_error("Wrong-direction displacement counted as useful response");
  if(away.first.step(.3,0.,0.,.5)!=.5)throw std::runtime_error("Backward clock retained old response learning");
  SettledPivot slow,unknown,fast;
  for(int i=0;i<=80;++i){
    const double t=i*.05;
    const double held=slow.step(2.8-.15*t,.15*t,.15,1.+t);
    if(i>=60&&held!=1.)throw std::runtime_error("Observed slow large turn restarted its yaw ramp");
    const double probe=unknown.step(2.8,0.,0.,1.+t);
    const double rapid=fast.step(2.8,.7*t,.7,1.+t);
    if(i==60&&(probe!=0.||rapid!=0.))throw std::runtime_error("Unknown or rapid response lost bounded probe");
  }
  if(slow.step(2.,.61,.8,5.05)!=0.)throw std::runtime_error("Changed fast response retained slow cruise evidence");
  std::cout<<"{\"passed\":true,\"cases\":11,\"hardware_commands\":0,\"scope\":\"Actual pivot class, synthetic measured samples and clock; no physical policy calibration\"}\n";
}
