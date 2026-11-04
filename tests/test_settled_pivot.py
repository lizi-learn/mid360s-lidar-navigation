"""Measure the actual pure-pivot helper with the unchanged native input ramp."""
import pathlib,shutil,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "settled_pivot.hpp"
#include "settled_arrival.hpp"
#include "speed_response.hpp"
#include <cassert>
#include <deque>
#include <limits>
using p2_nav_controller::SettledPivot;
int main(){
 p2_nav_controller::SettledArrival arrival;
 for(int i=0;i<80;++i)assert(!arrival.step(.01,.01,0,0,.1,i*.05,.15,.15));
 for(int i=0;i<40;++i)assert(!arrival.step(.01,.01,0,0,0,4.+i*.05,.15,.15));
 assert(arrival.step(.01,.01,0,0,0,6.01,.15,.15));
 arrival.reset();assert(!arrival.step(.01,.01,0,0,0,6.06,.15,.15));
 assert(!arrival.step(.16,.01,0,0,0,6.11,.15,.15));
 assert(!arrival.step(.01,.16,0,0,0,6.16,.15,.15));
 assert(!arrival.step(.01,.01,.04,0,0,6.21,.15,.15));
 assert(!arrival.step(.01,.01,0,0,0,20.,.15,.15));
 assert(!arrival.step(.01,.01,0,0,0,2.,.15,.15));
 SettledPivot c;
 assert(c.step(.1,0,0,0)==0.&&!c.pending());
 assert(c.step(1.,0,0,.05)>0.&&c.pending());
 c.reset();assert(!c.pending());assert(c.step(-1.,0,0,.1)<0.);
 // A missing/reversed control clock restarts the phase instead of replaying
 // a previous pulse; a settled aligned observation remains zero.
 assert(c.step(.1,0,0,1.)==0.&&!c.pending());
 c.step(1.,0,0,1.05);assert(c.step(.1,0,0,.2)==0.&&!c.pending());
 // A large initial turn may use the existing three-second bound, rather than
 // cycling the downstream ramp every two seconds. It must still brake early
 // when measured angular motion approaches the target.
 for(double sign:{-1.,1.}) {
  SettledPivot large;
  for(int k=0;k<=44;++k)assert(large.step(sign*2.8,0.,0.,k*.05)==sign);
  assert(large.step(sign*.7,0.,sign*.5,2.25)==0.);
 }
 bool rejected=false;try{c.step(std::numeric_limits<double>::quiet_NaN(),0,0,2.);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
 // Gyro vibration without accumulated heading progress is not a successful
 // turn. A deadzone must trigger the existing bounded full-input retry even
 // when short rate spikes are above the quiet threshold.
 for(double sign:{-1.,1.}) {
  SettledPivot noisy;bool boosted=false;
  for(int k=0;k<160;++k) {
   const double rate=k%20==0?.08:0.;
   const double raw=noisy.step(sign*.7,0.,rate,k*.05);
   assert(raw*sign>=0.&&std::abs(raw)<=1.);
   if(std::abs(raw)>.99)boosted=true;
  }
  assert(boosted);
 }
 // Frozen independent response parameters, both signs. Settled success is
 // checked after transport drains, rather than first entry into tolerance.
 for(double gain:{.55,.62,1.25,1.65,1.9})for(double delay:{.55,1.45,1.95})for(double dead:{.32,.44})for(double goal:{-.4,.4,-2.8,2.8})for(double scale:{.5,1.}){
  SettledPivot pivot;p2_nav::SpeedResponse native;std::deque<double> q(static_cast<int>(std::round(delay/.05))+1,0.);
  double yaw=0,rate=0,quiet=0;bool reached=false;
  for(int k=0;k<900;++k){double error=std::remainder(goal-yaw,2*std::acos(-1.));double raw=pivot.step(error,yaw,rate,k*.05);
   auto stamp=static_cast<std::uint64_t>((1.+k*.05)*1e9);auto input=native.step(0,raw*scale,0,rate,.05,stamp,stamp,true);
   assert(input.policy_x==0.&&std::abs(input.policy_yaw)<=1.);
   q.push_back(input.policy_yaw);double applied=q.front();q.pop_front();
   double target=std::abs(applied)<dead?0.:gain*applied;rate+=(target-rate)*(1-std::exp(-.05/.15));yaw+=rate*.05;
   if(std::abs(std::remainder(goal-yaw,2*std::acos(-1.)))<.15&&std::abs(rate)<.03&&raw==0.&&!pivot.pending())quiet+=.05;else quiet=0.;
   if(quiet>=2.){reached=true;break;}
  }
  assert(reached);
 }
}
'''
class SettledPivotTests(unittest.TestCase):
 @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
 def test_settled_response_reset_and_measured_feedback(self):
  with tempfile.TemporaryDirectory()as d:
   p=pathlib.Path(d);(p/'test.cpp').write_text(SOURCE)
   subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation'),'-I'+str(ROOT/'navigation/p2_nav_controller/include/p2_nav_controller'),str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True)
   subprocess.run([str(p/'test')],check=True,capture_output=True)
