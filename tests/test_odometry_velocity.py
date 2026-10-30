import unittest,sys,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from odometry_velocity import PlanarVelocity
class VelocityTests(unittest.TestCase):
 def test_constant_motion_and_current_body_frame(self):
  e=PlanarVelocity();v=None
  for t in np.arange(0,1,.02):v=e.update(float(t),[0.,.2*t,math.pi/2])
  np.testing.assert_allclose(v,[.2,0.,0.],atol=1e-10)
 def test_wrapped_yaw_is_not_a_full_turn(self):
  e=PlanarVelocity();v=None
  for t in np.arange(0,1,.02):v=e.update(float(t),[0.,0.,(3.1+.3*t+math.pi)%(2*math.pi)-math.pi])
  self.assertAlmostEqual(v[2],.3,places=10)
 def test_stepping_ripple_is_reduced_without_changing_pose(self):
  e=PlanarVelocity();estimated=[];raw=[]
  for t in np.arange(0,4,.02):
   v=e.update(float(t),[.2*t+.006*math.sin(2*math.pi*5*t),0.,.008*math.sin(2*math.pi*5*t)])
   if t>1:estimated.append(v);raw.append([.2+.006*2*math.pi*5*math.cos(2*math.pi*5*t),.008*2*math.pi*5*math.cos(2*math.pi*5*t)])
  a=np.asarray(estimated);b=np.asarray(raw)
  self.assertLess(a[:,0].std(),b[:,0].std()*.3);self.assertLess(a[:,2].std(),b[:,1].std()*.3)
  self.assertAlmostEqual(a[:,0].mean(),.2,places=2)
 def test_dropout_requires_new_history_and_duplicates_do_not_extend_it(self):
  e=PlanarVelocity();self.assertIsNone(e.update(1.,[0.,0.,0.]))
  self.assertIsNone(e.update(1.,[0.,0.,0.]))
  self.assertIsNotNone(e.update(1.2,[.02,0.,0.]))
  self.assertIsNone(e.update(2.,[.02,0.,0.]))
  self.assertIsNone(e.update(2.1,[.03,0.,0.]))
 def test_bad_time_nan_and_teleport_clear_estimate(self):
  for t,p in [(0.,[0.,0.,0.]),(float('nan'),[0.,0.,0.]),(1.1,[2.,0.,0.]),(1.,[.01,0.,0.])]:
   e=PlanarVelocity();e.update(1.,[0.,0.,0.])
   with self.assertRaises(ValueError):e.update(t,p)
   self.assertIsNone(e.last);self.assertFalse(e.samples)
if __name__=='__main__':unittest.main()
