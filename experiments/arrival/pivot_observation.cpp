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
  std::cout<<"{\"passed\":true,\"cases\":7,\"hardware_commands\":0,\"scope\":\"Actual pivot class, synthetic measured samples and clock; no physical policy calibration\"}\n";
}
