import math,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import test_position_goals as fixtures
from rotation_intent import rotation_goal,rotation_tree

class RotationTests(unittest.TestCase):
    def setup_console(self):return fixtures.PositionGoalTests.setup_console(self)
    def test_rotation_bypasses_global_planning_without_inventing_a_path(self):
        c=self.setup_console();c.reception_pose=lambda:[3.,4.,2.5]
        c.checked_pose=lambda d,**kw:object()
        c.wait_planner_active=lambda:(_ for _ in()).throw(AssertionError('must not call global planner'))
        r=c.prepare_rotation({'radians':-math.pi/2})
        self.assertFalse(r['global_path_planned']);self.assertFalse(r['hardware_execution_enabled'])
        self.assertFalse(c.planner.sent);self.assertEqual(c.session.path_points,0)
        self.assertEqual(c.session.task_kind,'rotation');self.assertEqual(c.session.goal['x'],3.)
        self.assertAlmostEqual(c.session.goal['yaw'],2.5-math.pi/2)
        c.confirm_goal({'generation':r['generation']})
        tree=ET.parse(c.nav.sent[0].behavior_tree)
        self.assertIsNone(tree.find('.//ComputePathToPose'))
        self.assertIsNone(tree.find('.//FollowPath'))
        self.assertAlmostEqual(float(tree.find('.//Spin').get('spin_dist')),-math.pi/2)
        self.assertEqual(c.session.phase,'executing')

    def test_operator_takeover_cannot_start_prepared_rotation(self):
        c=self.setup_console();c.reception_pose=lambda:[3.,4.,2.5];c.checked_pose=lambda d,**kw:object()
        r=c.prepare_rotation({'radians':-math.pi/2})
        c.session.operator('manual',True,True,False,1,2,10.01,10.01)
        with self.assertRaises(ValueError):c.confirm_goal({'generation':r['generation']})
        self.assertFalse(c.nav.sent)

    def test_rotation_validation_and_preserved_centre(self):
        for value in [None,True,0.,float('nan'),float('inf'),1e6]:
            with self.assertRaises(ValueError):rotation_goal([3.,4.,.2],value)
        with self.assertRaises(ValueError):rotation_goal(None,-1.)
        with tempfile.TemporaryDirectory()as f:
            tree=ET.parse(rotation_tree(f,-math.pi/2));self.assertIsNone(tree.find('.//BackUp'))
            self.assertIsNone(tree.find('.//ComputePathToPose'))

    def test_normal_goal_clears_rotation_intent(self):
        c=self.setup_console();c.goal_rotation=-1.
        c.set_goal(dict(x=1.,y=1.,yaw=0.))
        self.assertIsNone(c.goal_rotation)

    def test_multiple_revolutions_keep_unwrapped_action_distance(self):
        c=self.setup_console();c.reception_pose=lambda:[3.,4.,.2];c.checked_pose=lambda d,**kw:object()
        r=c.prepare_rotation({'radians':-4*math.pi})
        self.assertAlmostEqual(c.session.goal['yaw'],.2)
        c.confirm_goal({'generation':r['generation']})
        spin=ET.parse(c.nav.sent[0].behavior_tree).find('.//Spin')
        self.assertAlmostEqual(float(spin.get('spin_dist')),-4*math.pi)
        self.assertEqual(float(spin.get('time_allowance')),180.)
        self.assertEqual(c.session.phase,'executing','same final heading is not an arrival receipt')
