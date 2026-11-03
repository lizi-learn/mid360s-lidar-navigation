"""Exercise actual console intent routing and action acceptance, without robot I/O."""
from pathlib import Path
from types import SimpleNamespace as NS
from concurrent.futures import Future
from unittest.mock import Mock
import unittest,tempfile,xml.etree.ElementTree as ET
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
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        c,_=fixture();c.runtime=Path(temporary.name);c.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000000000));c.mode='navigation';c.gait_profile='p2-flat-model';c.gate={'localized':True};c.checked_pose=lambda d:object();c.wait_planner_active=lambda:None;c.wait_future=lambda f,t:f.result(timeout=t);c.planner=Client(True);c.nav=Client(False);c.execution_ready=lambda:True;c.health_ready=lambda:True;c.gait=None
        env=Console.set_goal.__globals__;env.update(ComputePathToPose=NS(Goal=Goal),NavigateToPose=NS(Goal=Goal),Path=Path,__file__=str(Path(__file__).resolve().parents[1]/'navigation/console.py'))
        return c
    def test_position_mode_matches_preview_and_live_planners(self):
        c=self.setup_console();receipt=c.set_goal(dict(x=1.,y=1.,yaw=2.,heading_mode='position'))
        self.assertEqual(c.planner.sent[0].planner_id,'PositionOnly');self.assertEqual(c.session.phase,'ready')
        self.assertFalse(any(z.get('armed') for z in c.emitted))
        c.confirm_goal({'generation':receipt['generation']})
        tree=ET.parse(c.nav.sent[0].behavior_tree);self.assertEqual(tree.find('.//ComputePathToPose').get('planner_id'),'PositionOnly');self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),'position_goal_checker');self.assertEqual(c.session.phase,'executing')
    def test_legacy_pose_goal_retains_explicit_terminal_heading(self):
        c=self.setup_console();receipt=c.set_goal(dict(x=1.,y=1.,yaw=0.))
        self.assertEqual(c.planner.sent[0].planner_id,'GridBased');c.confirm_goal({'generation':receipt['generation']});tree=ET.parse(c.nav.sent[0].behavior_tree);self.assertEqual(tree.find('.//ComputePathToPose').get('planner_id'),'GridBased');self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),'goal_checker')
    def test_bad_mode_does_not_prepare_or_start_any_goal(self):
        c=self.setup_console();generation=c.session.generation
        with self.assertRaisesRegex(ValueError,'未知目标'):c.set_goal(dict(x=1.,y=1.,yaw=0.,heading_mode='typo'))
        self.assertEqual(c.session.generation,generation);self.assertFalse(c.planner.sent);self.assertFalse(c.nav.sent)
    def directed_route(self):
        c=self.setup_console();c.planner_through=Client(True);c.nav_through=Client(False)
        c.route_final_goal=Mock()
        Console.set_goal.__globals__.update(ComputePathThroughPoses=NS(Goal=Goal),NavigateThroughPoses=NS(Goal=Goal))
        data=dict(x=1.,y=0.,yaw=.6,heading_mode='pose',_route_goals=[dict(x=.5,y=0.,yaw=0.),dict(x=1.,y=0.,yaw=.6)])
        return c,data
    def test_directed_route_preview_and_replanning_share_final_request(self):
        c,data=self.directed_route();receipt=c.set_goal(data)
        self.assertEqual(c.planner_through.sent[0].planner_id,'GuideDirected')
        c.route_final_goal.assert_called_once_with(data,c.goal_route[-1],'GuideDirected')
        c.confirm_goal({'generation':receipt['generation']})
        tree=ET.parse(c.nav_through.sent[0].behavior_tree)
        self.assertEqual(tree.find('.//ComputeReferencePath').get('planner_id'),'GuideDirected')
        self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),'goal_checker')
    def test_position_corridor_uses_same_guide_planner_for_preview_and_execution(self):
        c,data=self.directed_route();data['heading_mode']='position'
        receipt=c.set_goal(data)
        self.assertEqual(c.planner_through.sent[0].planner_id,'GuidePosition')
        c.route_final_goal.assert_called_once_with(data,c.goal_route[-1],'GuidePosition')
        c.confirm_goal({'generation':receipt['generation']})
        tree=ET.parse(c.nav_through.sent[0].behavior_tree)
        self.assertEqual(tree.find('.//ComputeReferencePath').get('planner_id'),'GuidePosition')
        self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),'position_goal_checker')
        self.assertEqual(tree.find('.//RemovePassedGoals').get('radius'),'0.75')

    def test_explicit_exact_through_route_preserves_original_constraints(self):
        for mode,planner,tree in [('pose','RouteDirected','forward_directed_route.xml'),('position','PositionOnly','forward_position_through_poses.xml')]:
            c,data=self.directed_route();data.update(heading_mode=mode,_route_reference_mode='exact')
            receipt=c.set_goal(data);self.assertEqual(c.planner_through.sent[0].planner_id,planner)
            c.confirm_goal({'generation':receipt['generation']})
            self.assertEqual(ET.parse(c.nav_through.sent[0].behavior_tree).find('.//ComputeReferencePath').get('planner_id'),planner)

    def test_invalid_corridor_mode_does_not_prepare_any_request(self):
        c,data=self.directed_route();data['_route_reference_mode']='typo'
        generation=c.session.generation
        with self.assertRaisesRegex(ValueError,'通道参考'):c.set_goal(data)
        self.assertEqual(c.session.generation,generation)
        self.assertFalse(c.planner_through.sent);self.assertFalse(c.nav_through.sent)

    def test_manual_cancel_during_route_parameters_never_sends_plan_or_motion(self):
        c,data=self.directed_route();c.route_final_goal.side_effect=lambda *_:c.session.cancel()
        with self.assertRaisesRegex(ValueError,'取消'):c.set_goal(data)
        self.assertFalse(c.planner_through.sent);self.assertFalse(c.nav_through.sent)
    def test_route_parameter_failure_cleans_up_pending_preview(self):
        c,data=self.directed_route();c.route_final_goal.side_effect=ValueError('synthetic parameter failure')
        with self.assertRaisesRegex(ValueError,'parameter failure'):c.set_goal(data)
        self.assertFalse(c.pending_plan_accept);self.assertFalse(c.planner_through.sent)

    def test_both_trees_have_unambiguous_checker_when_two_are_loaded(self):
        import xml.etree.ElementTree as ET
        root=Path(__file__).resolve().parents[1]/'navigation'
        for filename,planner,checker in [('forward_navigation.xml','GridBased','goal_checker'),('forward_position_navigation.xml','PositionOnly','position_goal_checker')]:
            tree=ET.parse(root/filename);self.assertEqual(tree.find('.//ComputePathToPose').attrib['planner_id'],planner);self.assertEqual(tree.find('.//FollowPath').attrib['goal_checker_id'],checker)

if __name__=='__main__':unittest.main()
