"""A localization estimate is not a navigation goal or a claim of free space."""
import ast,pathlib,types,math,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
class InitialPoseSeedTests(unittest.TestCase):
 def test_unknown_localization_seed_does_not_make_unknown_goal_valid(self):
  tree=ast.parse((ROOT/'navigation/console.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='Console')
  method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='checked_pose')
  def pose():return types.SimpleNamespace(header=types.SimpleNamespace(),pose=types.SimpleNamespace(position=types.SimpleNamespace(),orientation=types.SimpleNamespace()))
  env={'math':math,'PoseStamped':pose};exec(compile(ast.Module(body=[method],type_ignores=[]),'actual_checked_pose','exec'),env)
  grid=types.SimpleNamespace(info=types.SimpleNamespace(origin=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.)),resolution=1.,width=2,height=2),data=[-1,100,0,0])
  obj=types.SimpleNamespace(grid=grid,get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_msg=lambda:0)))
  data={'x':.2,'y':.2,'yaw':0.}
  with self.assertRaises(ValueError):env['checked_pose'](obj,data)
  self.assertEqual(env['checked_pose'](obj,data,require_free=False).pose.position.x,.2)
  with self.assertRaises(ValueError):env['checked_pose'](obj,{**data,'x':math.nan},require_free=False)
  with self.assertRaises(ValueError):env['checked_pose'](obj,{**data,'x':20.},require_free=False)
  self.assertEqual(grid.data,[-1,100,0,0])
if __name__=='__main__':unittest.main()
