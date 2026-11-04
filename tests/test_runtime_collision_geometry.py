"""A requested body change must reach real collision polygons, not just maps."""
import unittest,pathlib,tempfile,json,sys,yaml
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'navigation'))
from configure import config,apply_runtime_geometry
class RuntimeCollisionGeometryTests(unittest.TestCase):
 def test_both_map_live_ray_merges_survive_workflow_regeneration(self):
  with tempfile.TemporaryDirectory()as d:
   p=pathlib.Path(d)/'nav2.yaml';p.write_text(yaml.safe_dump(config()))
   shape=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
   override=p.with_name('planning_geometry.json')
   override.write_text(json.dumps(dict(footprint=shape,footprint_padding=0.,global_obstacle_combination=0,local_obstacle_combination=0)))
   apply_runtime_geometry(p);out=yaml.safe_load(p.read_text())
   for kind in ('local','global'):
    layer=out[kind+'_costmap'][kind+'_costmap']['ros__parameters']
    self.assertEqual(layer['obstacle_layer']['combination_method'],0)
    self.assertEqual(json.loads(layer['footprint']),shape)
   before=p.read_bytes();override.write_text(json.dumps(dict(footprint=shape,local_obstacle_combination=True)))
   with self.assertRaises(ValueError):apply_runtime_geometry(p)
   self.assertEqual(p.read_bytes(),before)
 def test_requested_55cm_body_preserves_clearances_and_obstacle_action(self):
  with tempfile.TemporaryDirectory()as d:
   p=pathlib.Path(d)/'nav2.yaml';original=config(gait_profile='p2-flat-model',stop_side_margin=.05)
   p.write_text(yaml.safe_dump(original));shape=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
   p.with_name('planning_geometry.json').write_text(json.dumps({'footprint':shape,'footprint_padding':0.}))
   apply_runtime_geometry(p);out=yaml.safe_load(p.read_text())
   for name,front,rear,side in [('StopZone',.53,.15,.05),('SlowZone',.95,.25,.32)]:
    zone=out['collision_monitor']['ros__parameters'][name];points=zone['points']
    self.assertAlmostEqual(max(points[::2]),.275+front);self.assertAlmostEqual(min(points[::2]),-.275-rear)
    self.assertAlmostEqual(max(points[1::2]),.275+side);self.assertAlmostEqual(min(points[1::2]),-.275-side)
    self.assertEqual(zone['max_points'],original['collision_monitor']['ros__parameters'][name]['max_points'])
    self.assertEqual(zone['action_type'],original['collision_monitor']['ros__parameters'][name]['action_type'])
   self.assertEqual(json.loads(out['local_costmap']['local_costmap']['ros__parameters']['footprint']),shape)
   self.assertEqual(out['collision_monitor']['ros__parameters']['StopZone']['action_type'],'stop')
   apply_runtime_geometry(p);self.assertEqual(yaml.safe_load(p.read_text()),out)
 def test_integer_zero_serializes_as_ros_double_and_bool_is_invalid(self):
  with tempfile.TemporaryDirectory()as d:
   p=pathlib.Path(d)/'nav2.yaml';p.write_text(yaml.safe_dump(config()))
   shape=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
   override=p.with_name('planning_geometry.json')
   override.write_text(json.dumps({'footprint':shape,'footprint_padding':0}))
   apply_runtime_geometry(p);out=yaml.safe_load(p.read_text())
   for name in ('local_costmap','global_costmap'):
    value=out[name][name]['ros__parameters']['footprint_padding']
    self.assertIs(type(value),float);self.assertEqual(value,0.)
   override.write_text(json.dumps({'footprint':shape,'footprint_padding':False}))
   before=p.read_bytes()
   with self.assertRaises(ValueError):apply_runtime_geometry(p)
   self.assertEqual(p.read_bytes(),before)
if __name__=='__main__':unittest.main()
