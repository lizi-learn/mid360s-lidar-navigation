import unittest,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from rolling_scan import PoseSeries,raycast_boxes
class RollingScanTests(unittest.TestCase):
 def test_compensation_preserves_stationary_wall_during_translation_and_rotation(self):
  t=np.linspace(0,.1,21);p=np.column_stack([t*2,t*0,np.ones(len(t))]);r=Rotation.from_euler('z',t*3)
  fixed=np.column_stack([np.full(len(t),5),np.linspace(-1,1,len(t)),np.ones(len(t))]);raw=r.inv().apply(fixed-p)
  world=PoseSeries(t,p,r.as_quat()).world_points(raw,t)
  self.assertTrue(np.allclose(world,fixed,atol=1e-8));naive=r[-1].apply(raw)+p[-1];self.assertGreater(np.max(np.linalg.norm(naive-fixed,axis=1)),.5)
 def test_unknown_pose_times_and_gaps_are_not_clamped(self):
  s=PoseSeries([0.,.1],[[0,0,0],[0,0,0]],[[0,0,0,1],[0,0,0,1]])
  for t in ([-.01],[.05],[.11]):
   with self.assertRaises(ValueError):s.at(t)
 def test_three_dimensional_wall_floor_and_parallel_rays(self):
  o=np.array([[0,0,1.],[0,0,1],[0,0,1]]);d=np.array([[1.,0,0],[0,0,-1.],[0,1.,0]])
  r=raycast_boxes(o,d,[[5,-1,0,5.1,1,2]])
  self.assertAlmostEqual(r[0],5.);self.assertAlmostEqual(r[1],1.);self.assertTrue(np.isinf(r[2]))
 def test_ray_inside_box_hits_exit_surface(self):
  r=raycast_boxes([[0,0,1]],[[1,0,0]],[[-1,-1,0,1,1,2]]);self.assertAlmostEqual(r[0],1.)
