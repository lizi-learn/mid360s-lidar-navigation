import json,sys,unittest,tempfile,yaml
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from configure import config,apply_runtime_geometry

class RequestedEnvelopeTests(unittest.TestCase):
 def test_explicit_55cm_stays_55cm_in_both_costmaps_without_changing_stop_or_speed(self):
  box=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
  before=config(gait_profile='p2-flat-model',footprint_vertices=box,stop_side_margin=.05,planning_side_padding=0.)
  after=config(gait_profile='p2-flat-model',footprint_vertices=box,stop_side_margin=.05,planning_side_padding=0.,planning_envelope='body',footprint_padding=0.)
  for name in ('global_costmap','local_costmap'):
   grid=after[name][name]['ros__parameters'];self.assertEqual(json.loads(grid['footprint']),box);self.assertEqual(grid['footprint_padding'],0.)
   self.assertTrue(grid['obstacle_layer']['enabled']);self.assertEqual(grid['plugins'],before[name][name]['ros__parameters']['plugins'])
  for name in ('collision_monitor','velocity_smoother','controller_server','planner_server'):self.assertEqual(after[name],before[name])
 def test_unknown_envelope_and_invalid_padding_rejected(self):
  for kwargs in ({'planning_envelope':'unknown'},{'footprint_padding':float('nan')},{'footprint_padding':-.01},{'footprint_padding':.06}):
   with self.assertRaises(ValueError):config(**kwargs)
 def test_workflow_reload_preserves_explicit_geometry_without_rewriting_obstacles(self):
  box=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]]
  with tempfile.TemporaryDirectory() as folder:
   path=Path(folder)/'nav2.yaml';before=config(gait_profile='p2-flat-model',footprint_vertices=box)
   path.write_text(yaml.safe_dump(before));self.assertFalse(apply_runtime_geometry(path));self.assertEqual(yaml.safe_load(path.read_text()),before)
   path.with_name('planning_geometry.json').write_text(json.dumps({'footprint':box,'footprint_padding':0.}))
   self.assertTrue(apply_runtime_geometry(path));after=yaml.safe_load(path.read_text())
   for name in ('local_costmap','global_costmap'):
    old=before[name][name]['ros__parameters'];new=after[name][name]['ros__parameters']
    self.assertEqual(json.loads(new.pop('footprint')),box);self.assertEqual(new.pop('footprint_padding'),0.)
    old.pop('footprint');old.pop('footprint_padding');self.assertEqual(new,old)
   for name in ('collision_monitor','velocity_smoother','controller_server'):self.assertEqual(after[name],before[name])

if __name__=='__main__':unittest.main()
