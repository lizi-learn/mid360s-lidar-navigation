import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from footprint import checked
from configure import config
class FootprintTests(unittest.TestCase):
 def test_explicit_side_reserve_changes_no_body_or_front_rear_limits(self):
  import json
  old=config(gait_profile='p2-flat-model');new=config(gait_profile='p2-flat-model',stop_side_margin=.05)
  a=old['collision_monitor']['ros__parameters'];b=new['collision_monitor']['ros__parameters']
  self.assertEqual(a['StopZone']['points'][::2],b['StopZone']['points'][::2]);self.assertEqual(a['SlowZone'],b['SlowZone'])
  self.assertEqual(old['local_costmap'],new['local_costmap']);self.assertEqual(old['velocity_smoother'],new['velocity_smoother'])
  self.assertAlmostEqual(max(b['StopZone']['points'][1::2]),.57)
  shape=json.loads(new['global_costmap']['global_costmap']['ros__parameters']['footprint'])
  self.assertAlmostEqual(max(p[1] for p in shape),.57)
  self.assertEqual(a['source_timeout'],b['source_timeout']);self.assertEqual(a['transform_tolerance'],b['transform_tolerance'])
 def test_side_reserve_cannot_remove_body_clearance_or_accept_nan(self):
  for value in [0.,.049,.201,float('nan'),float('inf')]:
   with self.assertRaises(ValueError):config(gait_profile='p2-flat-model',stop_side_margin=value)
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

 def test_p2_routes_reserve_monitor_clearance_without_relaxing_output_limits(self):
  import json
  c=config(gait_profile='p2-flat-model');global_shape=json.loads(c['global_costmap']['global_costmap']['ros__parameters']['footprint']);local_shape=json.loads(c['local_costmap']['local_costmap']['ros__parameters']['footprint'])
  stop=c['collision_monitor']['ros__parameters']['StopZone']['points']
  self.assertGreaterEqual(max(p[0] for p in global_shape),max(stop[::2]));self.assertGreaterEqual(max(p[1] for p in global_shape),max(stop[1::2]))
  self.assertLess(max(p[1] for p in local_shape),max(p[1] for p in global_shape))
  smoother=c['velocity_smoother']['ros__parameters'];self.assertEqual(smoother['max_velocity'][1],0.);self.assertEqual(smoother['min_velocity'][0],0.);self.assertLessEqual(smoother['max_accel'][2],.5)
  self.assertFalse(c['controller_server']['ros__parameters']['FollowPath']['allow_reversing'])
  self.assertLessEqual(smoother['max_velocity'][0],.15);self.assertLessEqual(c['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'],.15)
  self.assertGreaterEqual(max(stop[::2]),1.0)
  with self.assertRaises(ValueError):config(gait_profile='p2-flat-model',minimum_speed=.2)
  with self.assertRaises(ValueError):config(controller='mppi',gait_profile='p2-flat-model')
