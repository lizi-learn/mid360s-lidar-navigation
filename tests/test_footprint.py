import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from footprint import checked
from configure import config
class FootprintTests(unittest.TestCase):
 def test_explicit_envelope_is_used_by_both_costmaps(self):
  vertices=[[.47,.52],[.47,-.52],[-.43,-.52],[-.43,.52]];c=config(footprint_vertices=vertices)
  import json
  for n in ('local_costmap','global_costmap'):self.assertEqual(json.loads(c[n][n]['ros__parameters']['footprint']),vertices)
 def test_self_intersection_nonfinite_or_outside_origin_rejected(self):
  for p in ([[1,1],[-1,-1],[1,-1],[-1,1]],[[1,1],[1,2],[2,2],[2,1]],[[1,1],[1,-1],[-1,math.nan]],[[1,1],[1,1],[-1,-1]]):
   with self.assertRaises(ValueError):checked(p)
 def test_clockwise_and_counterclockwise_convex_accepted(self):
  p=[[1,1],[1,-1],[-1,-1],[-1,1]];self.assertEqual(checked(p),p);self.assertEqual(checked(p[::-1]),p[::-1])

 def test_lateral_odometry_model_does_not_authorize_lateral_commands(self):
  c=config(odometry_model='omni');self.assertIn('OmniMotionModel',c['amcl']['ros__parameters']['robot_model_type'])
  smoother=c['velocity_smoother']['ros__parameters'];self.assertEqual(smoother['max_velocity'][1],0.);self.assertEqual(smoother['min_velocity'][1],0.)
  self.assertFalse(c['controller_server']['ros__parameters']['FollowPath']['allow_reversing'])
 def test_collision_monitor_encloses_enlarged_envelope(self):
  c=config(footprint_vertices=[[.47,.52],[.47,-.52],[-.43,-.52],[-.43,.52]])
  p=c['collision_monitor']['ros__parameters']['StopZone']['points'];self.assertGreater(max(p[::2]),.47+.05);self.assertGreater(max(p[1::2]),.52+.05)

 def test_planner_endpoint_tolerance_is_tighter_than_arrival_tolerance(self):
  c=config();self.assertLess(c['planner_server']['ros__parameters']['GridBased']['tolerance'],c['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'])
