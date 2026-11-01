"""Follow the user-selected 0.8*0.5 through buffering, gate and wire encoding."""
import unittest,sys,pathlib
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'navigation'))
from gate import ForwardGate,Limits
from motion_buffer import MotionBuffer
from gait_socket_client import encode_frame,Operation,Authority,VELOCITY
class ManualHalfInputTests(unittest.TestCase):
 def test_forward_point_four_not_old_physical_cap_or_double_half(self):
  b=MotionBuffer(accel=.4,decel=.8);g=ForwardGate(Limits(max_v=.8,max_w=.24,accel=.4,decel=.8))
  g.set_speed(.8);g.set_velocity_scale(.5);g.armed=True;values=[]
  for k in range(300):
   cmd=b.step((.8,0.,0.),(0.,0.,0.),k*.02,.1,frame_verified=True)
   values.append(g.step(cmd,.02,.01,.1,.1))
  self.assertAlmostEqual(values[-1][0],.4)
  self.assertTrue(all(0<=v[0]<=.4 and v[1]==0 for v in values))
  frame=encode_frame(Operation.VELOCITY,1,1,10**10,Authority(1,1,1,10**10,10**10,1,120),velocity=values[-1])
  self.assertAlmostEqual(VELOCITY.unpack(frame[-VELOCITY.size:])[6],.4,places=6)
  self.assertEqual(g.step((0.,0.,0.),.02,.01,.1,.1),(0.,0.,0.))
  with self.assertRaises(ValueError):encode_frame(Operation.VELOCITY,1,1,10**10,Authority(1,1,1,10**10,10**10,1,120),velocity=(.81,0.,0.))
if __name__=='__main__':unittest.main()
