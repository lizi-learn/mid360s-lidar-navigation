"""Exercise actual console intent routing and action acceptance, without robot I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
from concurrent.futures import Future
import unittest
from test_console_cancellation import Console,fixture,Handle

class Goal:pass
class Client:
    def __init__(self,planning):self.planning=planning;self.sent=[]
    def wait_for_server(self,timeout_sec):return True
    def send_goal_async(self,goal,**kw):
        self.sent.append(goal);h=Handle()
        if self.planning:h.result.set_result(NS(status=4,result=NS(path=NS(poses=[NS(pose=NS(position=NS(x=1.,y=1.)))]))))
        f=Future();f.set_result(h);return f

class PositionGoalTests(unittest.TestCase):
    def setup_console(self):
        c,_=fixture();c.mode='navigation';c.gait_profile='p2-flat-model';c.gate={'localized':True};c.checked_pose=lambda d:object();c.wait_planner_active=lambda:None;c.wait_future=lambda f,t:f.result(timeout=t);c.planner=Client(True);c.nav=Client(False);c.execution_ready=lambda:True;c.health_ready=lambda:True;c.gait=None
        env=Console.set_goal.__globals__;env.update(ComputePathToPose=NS(Goal=Goal),NavigateToPose=NS(Goal=Goal),Path=Path,__file__=str(Path(__file__).resolve().parents[1]/'navigation/console.py'))
        return c
    def test_position_mode_matches_preview_and_live_planners(self):
        c=self.setup_console();receipt=c.set_goal(dict(x=1.,y=1.,yaw=2.,heading_mode='position'))
        self.assertEqual(c.planner.sent[0].planner_id,'PositionOnly');self.assertEqual(c.session.phase,'ready')
        self.assertFalse(any(z.get('armed') for z in c.emitted))
        c.confirm_goal({'generation':receipt['generation']})
        self.assertTrue(c.nav.sent[0].behavior_tree.endswith('/forward_position_navigation.xml'));self.assertEqual(c.session.phase,'executing')
    def test_legacy_pose_goal_retains_explicit_terminal_heading(self):
        c=self.setup_console();receipt=c.set_goal(dict(x=1.,y=1.,yaw=0.))
        self.assertEqual(c.planner.sent[0].planner_id,'GridBased');c.confirm_goal({'generation':receipt['generation']});self.assertFalse(hasattr(c.nav.sent[0],'behavior_tree'))
    def test_bad_mode_does_not_prepare_or_start_any_goal(self):
        c=self.setup_console();generation=c.session.generation
        with self.assertRaisesRegex(ValueError,'未知目标'):c.set_goal(dict(x=1.,y=1.,yaw=0.,heading_mode='typo'))
        self.assertEqual(c.session.generation,generation);self.assertFalse(c.planner.sent);self.assertFalse(c.nav.sent)
    def test_both_trees_have_unambiguous_checker_when_two_are_loaded(self):
        import xml.etree.ElementTree as ET
        root=Path(__file__).resolve().parents[1]/'navigation'
        for filename,planner,checker in [('forward_navigation.xml','GridBased','goal_checker'),('forward_position_navigation.xml','PositionOnly','position_goal_checker')]:
            tree=ET.parse(root/filename);self.assertEqual(tree.find('.//ComputePathToPose').attrib['planner_id'],planner);self.assertEqual(tree.find('.//FollowPath').attrib['goal_checker_id'],checker)

if __name__=='__main__':unittest.main()
