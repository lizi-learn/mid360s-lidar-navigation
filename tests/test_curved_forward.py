import pathlib,shutil,subprocess,tempfile,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
CODE=r'''
#include "curved_forward.hpp"
#include <cassert>
#include <cmath>
using p2_nav_controller::curved_forward;
int main(){
 assert(curved_forward(.8,0,.6)==.8);
 assert(curved_forward(.8,.15,.6)==.8);
 assert(std::abs(curved_forward(.8,.4,.6)-.6)<1e-12);
 assert(curved_forward(.8,.8,0)==.8);
 assert(curved_forward(0,1,.6)==0.);
 assert(curved_forward(.2,1,.6)==.2);
 double previous=.8;
 for(int i=0;i<=1000;++i){double yaw=i*.001,out=curved_forward(.8,yaw,.6);assert(out<=previous+1e-12&&out<=.8&&out>=.6-1e-12);assert(out==curved_forward(.8,-yaw,.6));assert(std::abs(out-previous)<.0013);previous=out;}
 for(double scale:{.5,1.})assert(std::abs(curved_forward(.8,1.,.6)*scale-.6*scale)<1e-12);
}
'''
class CurvedForwardTests(unittest.TestCase):
 @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
 def test_smooth_symmetric_curved_input_preserves_zero_and_multiplier(self):
  with tempfile.TemporaryDirectory()as d:
   p=pathlib.Path(d);(p/'test.cpp').write_text(CODE)
   subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation/p2_nav_controller/include/p2_nav_controller'),str(p/'test.cpp'),'-o',str(p/'test')],check=True,capture_output=True)
   subprocess.run([str(p/'test')],check=True,capture_output=True)
