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
 double measured=0.;
 // An unknown learned deadband, absent any inverse curve or minimum kick.
 // Closed loop must leave standing, track the target, then clear immediately.
 for(int k=0;k<6000;++k){
  now+=10000000;s=c.step(.075,0.,measured,0.,.01,now-100000000,now,true);
  measured+=(std::max(0.,(s.policy_x-.22)*4.)-measured)*.05;
  assert(s.valid&&s.policy_x>=0.&&s.policy_x<=.30);
 }
 assert(measured>.065&&measured<.085);
 s=c.step(0.,0.,measured,0.,.01,now,now,true);assert(s.policy_x==0.&&s.policy_yaw==0.);
 for(int i=0;i<4;++i){
  for(int k=0;k<50;++k){now+=10000000;c.step(.075,.1,0.,0.,.01,now,now,true);}
  if(i==0)s=c.step(.075,.1,0.,0.,.01,now,now,false); // manual/cancel/obstacle ownership
  if(i==1)s=c.step(.075,.1,0.,0.,.01,now-500000001,now,true);
  if(i==2)s=c.step(.075,.1,0.,0.,.01,now+5000001,now,true);
  if(i==3)s=c.step(.075,.1,std::numeric_limits<double>::quiet_NaN(),0.,.01,now,now,true);
  assert(s.policy_x==0.&&s.policy_yaw==0.&&!s.valid);
 }
 // Repeated packets cannot renew an old measurement timestamp.
 const auto source=now;for(int k=0;k<60;++k){now+=10000000;s=c.step(.075,0.,0.,0.,.01,source,now,true);}
 assert(s.reason==speed_feedback_expired&&s.policy_x==0.);
}
'''
class SpeedResponseTests(unittest.TestCase):
 @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
 def test_measured_feedback_deadband_and_immediate_stop(self):
  with tempfile.TemporaryDirectory() as d:
   p=pathlib.Path(d);(p/'test.cpp').write_text(SOURCE)
   subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation'),str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True)
   subprocess.run([str(p/'test')],check=True,capture_output=True)
if __name__=='__main__':unittest.main()
