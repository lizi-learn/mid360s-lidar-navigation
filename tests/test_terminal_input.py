"""Model deadzone regression for actual terminal response and native S input."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler required')
class TerminalInputTests(unittest.TestCase):
    def test_half_input_stall_progress_limits_and_reset(self):
        root = Path(__file__).resolve().parents[1]
        source = r'''
#include <cassert>
#include <cmath>
#include "p2_nav_controller/terminal_input.hpp"
#include "p2_nav_controller/terminal_velocity.hpp"
#include "speed_response.hpp"
using p2_nav_controller::TerminalInput;
int main(){
  for(double scale:{.5,1.})for(double deadzone:{.15,.25,.30})for(bool enabled:{false,true}){
    TerminalInput response;p2_nav::SpeedResponse native;double d=.49,measured=0.;bool reached=false;
    for(int i=0;i<240;++i){const double t=1.+i*.05;
      const double raw=enabled?response.step(.1,d,t,.8):.1;
      assert(raw>=.1&&raw<=.8);
      const auto stamp=static_cast<std::uint64_t>(t*1e9);
      const auto input=native.step(raw*scale,0.,measured,0.,.05,stamp,stamp,true);
      measured=input.policy_x>deadzone?.7*input.policy_x:0.;d-=measured*.05;
      if(d<=.4){reached=true;assert(t<9.);break;}
    }
    assert(reached==enabled);
  }
  // Actual observed bearing jitter at the forward/pivot boundary. The old
  // branch alternates every tick and never completes native S startup.
  for(bool continuous:{false,true}){
    p2_nav_controller::TerminalApproach approach;TerminalInput response;
    p2_nav::SpeedResponse native;double distance=.49,measured=0.;bool reached=false;
    for(int i=0;i<240;++i){double now=1.+i*.05,bearing=i%2?.16:.14;
      auto c=continuous?approach.step(distance,bearing,0.,false,1.,now):
        p2_nav_controller::terminal_velocity(distance,bearing,0.,false,1.);
      double raw=response.step(c[0],distance,now,.8);
      auto stamp=static_cast<std::uint64_t>(now*1e9);
      auto input=native.step(raw*.5,c[1]*.5,measured,0.,.05,stamp,stamp,true);
      measured=input.policy_x>.25?.7*input.policy_x:0.;distance-=measured*.05;
      if(distance<=.4){reached=true;break;}
    }
    assert(reached==continuous);
  }
  p2_nav_controller::TerminalApproach approach;
  assert(approach.step(.49,.14,0.,false,1.,1.)[0]>.0);
  assert(approach.step(.49,.16,0.,false,1.,1.05)[0]>.0);
  assert(approach.step(.49,.36,0.,false,1.,1.1)[0]==0.);
  assert(approach.step(.49,.16,0.,false,1.,1.15)[0]==0.);
  assert(approach.step(.49,.14,0.,false,1.,1.2)[0]>.0);
  assert(approach.step(.49,.16,.8,true,1.,1.25)[0]==0.);
  assert(approach.step(.49,.16,0.,false,1.,1.3)[0]==0.);
  assert(approach.step(.49,.14,0.,false,1.,1.35)[0]>.0);
  assert(approach.step(.49,.16,0.,false,1.,3.)[0]==0.);
  approach.reset();assert(approach.step(.49,.16,0.,false,1.,3.05)[0]==0.);
  TerminalInput p;double d=1.;
  for(int i=0;i<200;++i){d-=.002;assert(p.step(.1,d,1.+i*.05,.8)==.1);}
  p.reset();for(int i=0;i<200;++i)assert(p.step(.1,.49,1.+i*.05,.2)<=.2);
  assert(p.step(0.,.49,12.,.8)==0.);
  assert(p.step(.1,.49,12.05,.8)==.1);
  for(int i=0;i<200;++i)p.step(.1,.49,13.+i*.05,.8);
  assert(p.step(.1,.49,30.,.8)==.1); // stale observations do not retain a learned floor
  assert(p.step(.1,.49,1.,.8)==.1); // clock reset
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'test.cpp'
            exe = Path(tmp) / 'test'
            src.write_text(source)
            build = subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                                    '-I' + str(root / 'navigation/p2_nav_controller/include'),
                                    '-I' + str(root / 'navigation'), str(src), '-o', str(exe)],
                                   capture_output=True, text=True, timeout=30)
            self.assertEqual(build.returncode, 0, build.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
