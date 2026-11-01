import ast,pathlib,types,math,time,unittest,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation'))
from motion_buffer import MotionBuffer
class RawAngularInputTests(unittest.TestCase):
 def test_actual_callback_bounds_turn_without_discarding_forward(self):
  tree=ast.parse((ROOT/'navigation/motion_buffer_node.py').read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='MotionBufferNode')
  method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='command')
  env={'math':math,'time':time};exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_command','exec'),env)
  for w,expected in [(1.2,1.),(-1.2,-1.),(.5476,.5476),(-.55,-.55),(.1,.1)]:
   obj=types.SimpleNamespace();m=types.SimpleNamespace(linear=types.SimpleNamespace(x=.75,y=0.),angular=types.SimpleNamespace(z=w))
   env['command'](obj,m);self.assertEqual(obj.target,(.75,0.,expected))
   out=MotionBuffer().step(obj.target,(0.,0.,0.),1.,.1,frame_verified=True)
   self.assertGreater(out[0],0.)
  obj=types.SimpleNamespace();m.angular.z=math.nan;env['command'](obj,m)
  self.assertTrue(math.isnan(obj.target[2]))
  self.assertEqual(MotionBuffer().step(obj.target,(0.,0.,0.),1.,.1,frame_verified=True),(0.,0.,0.))
 def test_in_place_and_forward_share_requested_turn_range(self):
  tree=ast.parse((ROOT/'navigation/motion_buffer_node.py').read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='MotionBufferNode')
  method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='command')
  env={'math':math,'time':time};exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_command','exec'),env)
  for value,expected in [(1.2,1.),(-1.2,-1.),(.4,.4)]:
   obj=types.SimpleNamespace();m=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.,y=0.),angular=types.SimpleNamespace(z=value))
   env['command'](obj,m);self.assertEqual(obj.target,(0.,0.,expected))
  buffer=MotionBuffer()
  for i in range(200):out=buffer.step((0.,0.,1.),(0.,0.,0.),1.+i*.02,.05,frame_verified=True)
  self.assertAlmostEqual(out[2],1.)
  self.assertEqual(buffer.step((0.,0.,1.),(0.,0.,0.),5.,.05,frame_verified=True,hard_stop=True),(0.,0.,0.))
if __name__=='__main__':unittest.main()
