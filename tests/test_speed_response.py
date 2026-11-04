"""Exercise the production native loop adapter and original source timestamps."""
import pathlib, shutil, subprocess, tempfile, unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
SOURCE=r'''
#include "speed_response.hpp"
#include <cassert>
#include <limits>
using namespace p2_nav;
int main(){
 SpeedResponse c;std::uint64_t now=10000000000ULL;SpeedTrackingState s;
 double measured=.02, previous=0.;
 // Same manual maximum0.8, exactly one upstream half multiplier.
 // A different measured speed is reported, not mistaken for this input.
 for(int k=0;k<400;++k){
  now+=10000000;s=c.step(.8*.5,0.,measured,0.,.01,now-100000000,now,true);
  assert(s.valid&&s.policy_x>=previous&&s.policy_x-previous<=.00400001);
  previous=s.policy_x;
 }
 assert(std::abs(s.target_x-.4)<1e-12&&std::abs(s.policy_x-.4)<1e-12);
 assert(std::abs(s.measured_x-.02)<1e-12);
 for(int k=0;k<20;++k){now+=10000000;s=c.step(.4,0.,k%2?.2:-.1,0.,.01,now,now,true);assert(s.policy_x==.4);}
 s=c.step(.2,0.,.2,0.,.01,now,now,true);assert(s.policy_x>.2&&s.policy_x<.4);
 s=c.step(0.,0.,measured,0.,.01,now,now,true);assert(s.policy_x==0.&&s.policy_yaw==0.);
 for(int i=0;i<4;++i){
  for(int k=0;k<50;++k){now+=10000000;c.step(.075,.1,0.,0.,.01,now,now,true);}
  if(i==0)s=c.step(.075,.1,0.,0.,.01,now,now,false); // manual/cancel/obstacle ownership
  if(i==1)s=c.step(.075,.1,0.,0.,.01,now-500000001,now,true);
  if(i==2)s=c.step(.075,.1,0.,0.,.01,now+5000001,now,true);
  if(i==3)s=c.step(.075,.1,std::numeric_limits<double>::quiet_NaN(),0.,.01,now,now,true);
  assert(s.policy_x==0.&&s.policy_yaw==0.&&!s.valid);
 }
 // A zero on one axis is a controller withdrawal, not a normal-brake tag.
 // The other axis may stay active; translation must not trail into a pivot.
 for(int k=0;k<400;++k){now+=10000000;c.step(.75,.6,0.,0.,.01,now,now,true);}
 s=c.step(0.,.6,0.,0.,.01,now,now,true);assert(s.policy_x==0.&&s.policy_yaw>.59);
 s=c.step(.75,0.,0.,0.,.01,now,now,true);assert(s.policy_yaw==0.&&s.policy_x<=.000021);
 // Retain the entire old yaw contract, including changing signs and zeros.
 c.reset();double old_yaw=0.;
 for(int k=0;k<1200;++k){now+=10000000;const double target=k<220?1.:(k<620?-.8:(k<640?0.:.6));
  if(target==0.)old_yaw=0.;else old_yaw=std::clamp(target,old_yaw-.005,old_yaw+.005);
  s=c.step(.4,target,0.,0.,.01,now,now,true);assert(std::abs(s.policy_yaw-old_yaw)<1e-10);}
 // Repeated packets cannot renew an old measurement timestamp.
 const auto source=now;for(int k=0;k<60;++k){now+=10000000;s=c.step(.075,0.,0.,0.,.01,source,now,true);}
 assert(s.reason==speed_feedback_expired&&s.policy_x==0.);
}
'''
class SpeedResponseTests(unittest.TestCase):
 @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
 def test_manual_half_input_curve_and_immediate_stop(self):
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d);(p/'test.cpp').write_text(SOURCE)
   subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation'),str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True)
   subprocess.run([str(p/'test')],check=True,capture_output=True)
if __name__=='__main__':unittest.main()
