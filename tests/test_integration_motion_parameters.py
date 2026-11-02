"""Integration fixes retain the installed motion inputs and preview settings."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from configure import config
class RecoveryMotionDefaults(unittest.TestCase):
    def test_integration_fix_preserves_installed_motion_parameters(self):
        c=config(gait_profile='p2-flat-model',footprint_vertices=[[.275,.275],[.275,-.275],[-.275,-.275],[-.275,.275]],footprint_padding=0.)
        p=c['controller_server']['ros__parameters'];f=p['FollowPath']
        self.assertTrue(f['settled_pivot_response']);self.assertTrue(f['corner_preview'])
        self.assertEqual((f['min_lookahead_dist'],f['max_lookahead_dist']),(1.2,1.6))
        self.assertEqual(f['rotate_to_heading_angular_vel'],1.);self.assertEqual(f['curved_forward_input'],.6)
        self.assertFalse(f['allow_reversing']);self.assertTrue(f['use_collision_detection'])
        self.assertEqual(p['goal_checker']['plugin'],'p2_nav_controller::SettledHeadingGoalChecker')
        self.assertEqual(p['progress_checker']['movement_time_allowance'],40.)
        self.assertFalse(c['planner_server']['ros__parameters']['GridBased']['smooth_path'])
