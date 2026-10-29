import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gait_velocity import GaitVelocityTracker
class GaitVelocityTests(unittest.TestCase):
 def test_emergency_zero_bypasses_filter_and_integral(self):
  t=GaitVelocityTracker()
  for _ in range(50):t.step((.2,.2),(0.,0.),.05,.01)
  self.assertEqual(t.step((0.,0.),(.2,.2),.05,.01),(0.,0.));self.assertEqual(t.integral,[0.,0.])
 def test_stale_nonfinite_and_clock_gap_feedback_stop(self):
  for value in [((.2,.2),(.1,.1),.05,.3),((.2,.2),(math.nan,0),.05,.01),((.2,.2),(.1,.1),.3,.01)]:
   self.assertEqual(GaitVelocityTracker().step(*value),(0.,0.))
 def test_correction_is_bounded_and_never_reverses_requested_direction(self):
  t=GaitVelocityTracker()
  for _ in range(500):
   v,w=t.step((.29,-.34),(-1.,3.),.05,.01);self.assertTrue(0<=v<=.30);self.assertTrue(-.35<=w<=0)
 def test_underresponsive_mean_velocity_converges_within_bounds(self):
  t=GaitVelocityTracker();v=w=0.
  for _ in range(400):
   u=t.step((.2,.16),(v,w),.05,.01);v+=(.8*u[0]-v)*.2;w+=(.8*u[1]-w)*.2
  self.assertLess(abs(v-.2),.01);self.assertLess(abs(w-.16),.01)
