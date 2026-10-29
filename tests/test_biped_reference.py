import sys,tempfile,unittest,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from biped_reference import KinematicChain,JointHistory,rigid,inverse,body_from_lidar,ground_pose,validate_transform

URDF='''<robot name="test"><link name="base"/><link name="head"/><joint name="neck" type="revolute"><parent link="base"/><child link="head"/><origin xyz="0.2 0 0.6" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>'''
class BipedReferenceTests(unittest.TestCase):
 def chain(self,xml=URDF,overrides=()):
  f=tempfile.NamedTemporaryFile(suffix='.urdf',delete=False);f.write(xml.encode());f.close();self.addCleanup(Path(f.name).unlink)
  return KinematicChain(f.name,'base','head',overrides)
 def test_head_rotation_does_not_become_body_yaw_or_translation(self):
  chain=self.chain();base=rigid(Rotation.from_euler('xyz',[.1,-.12,.6]).as_matrix(),[2.,3.,.9]);mount=rigid(Rotation.from_euler('x',math.pi).as_matrix(),[.1,0,.08])
  for angle in [-.8,0.,.8]:
   head=chain.transform({'neck':angle});lidar=base@head@mount
   result=body_from_lidar(lidar,head,mount)
   self.assertTrue(np.allclose(result,base,atol=1e-10));self.assertTrue(np.allclose(ground_pose(result),[2,3,.6],atol=1e-10))
 def test_joint_origin_precedes_joint_rotation(self):
  t=self.chain().transform({'neck':math.pi/4});self.assertTrue(np.allclose(t[:3,3],[.2,0,.6]))
 def test_fixed_head_remains_fixed_without_explicit_override(self):
  xml=URDF.replace('type="revolute"','type="fixed"');c=self.chain(xml);self.assertEqual(c.required,());self.assertTrue(np.allclose(c.transform({})[:3,:3],np.eye(3)))
  c=self.chain(xml,['neck']);self.assertEqual(c.required,('neck',))
 def test_missing_and_out_of_range_joints_rejected(self):
  c=self.chain()
  for q in ({},{'neck':1.2},{'neck':float('nan')}):
   with self.assertRaises(ValueError):c.transform(q)
 def test_feedback_interpolates_at_sensor_time(self):
  h=JointHistory(['neck']);h.add(1.,{'neck':0.});h.add(1.08,{'neck':.8});self.assertAlmostEqual(h.at(1.04)['neck'],.4)
 def test_stale_missing_backward_and_gap_feedback_rejected(self):
  h=JointHistory(['neck'])
  with self.assertRaises(ValueError):h.at(1.)
  h.add(1.,{'neck':0.});h.add(1.2,{'neck':.1})
  for t in [.9,1.1,1.3]:
   with self.assertRaises(ValueError):h.at(t)
  with self.assertRaises(ValueError):h.add(1.1,{'neck':0.})
 def test_reflection_and_nonfinite_transform_rejected(self):
  a=np.eye(4);a[0,0]=-1
  with self.assertRaises(ValueError):validate_transform(a)
  a=np.eye(4);a[0,3]=np.nan
  with self.assertRaises(ValueError):inverse(a)
 def test_excessive_body_tilt_rejected_for_flat_navigation(self):
  with self.assertRaises(ValueError):ground_pose(rigid(Rotation.from_euler('x',.6).as_matrix()))
if __name__=='__main__':unittest.main()
