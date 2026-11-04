#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "p2_nav_controller/lateral_motion.hpp"
int main(){
  p2_nav_controller::LateralMotion estimate;
  double maximum=0.;
  for(int k=0;k<1000;++k){double t=k*.01;
    double value=estimate.step(.12+.25*std::sin(4*std::acos(-1.)*t),t);
    if(t>2.)maximum=std::max(maximum,std::abs(value-.12));
  }
  if(maximum>.07)throw std::runtime_error("Alternating sway treated as sustained drift");
  if(std::abs(estimate.step(.12,10.)-.12)>.07)throw std::runtime_error("Bias lost");
  estimate.reset();
  for(int k=0;k<40;++k)if(std::abs(estimate.step(-.2,k*.05)+.2)>1e-9)
    throw std::runtime_error("Constant real drift suppressed");
  if(estimate.step(.18,3.)!=.18||estimate.step(-.16,1.)!=-.16)
    throw std::runtime_error("Gap or backward clock carried old state");
  bool rejected=false;
  try{estimate.step(std::numeric_limits<double>::quiet_NaN(),1.1);}catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)throw std::runtime_error("Invalid sample accepted");
  std::cout<<"{\"passed\":true,\"steady_ripple_peak\":"<<maximum
    <<",\"constant_drift_retained\":true,\"clock_and_gap_reset\":true,\"hardware_commands\":0}\n";
}
