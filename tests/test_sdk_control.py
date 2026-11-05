import sys
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from sdk_control import stop,mission
from session import NavigationSession

class SDKControlTests(unittest.TestCase):
    def node(self):
        n=NS(lock=threading.RLock(),session=NS(generation=3),reception=NS(engine=NS(run_id='r',epoch=2)),stops=0)
        def stopped():n.stops+=1
        n.reception.map_id=lambda:'example'
        n.stop=stopped;n.reception.command=lambda command:command
        return n
    def test_conditional_cancel_never_stops_replacement(self):
        n=self.node();expected={'generation':3,'run_id':'r','epoch':2,'map_id':'example'};n.session.generation=4
        with self.assertRaises(ValueError):stop(n,{'_sdk_expected':expected})
        self.assertEqual(n.stops,0)
    def test_old_app_stop_and_mission_still_work(self):
        n=self.node();stop(n,{});self.assertEqual(n.stops,1)
        self.assertEqual(mission(n,{'reception_command':{'command':'start'}}),{'command':'start'})
    def test_compare_mission_inside_owner_lock(self):
        n=self.node();expected={'generation':3,'run_id':'r','epoch':1,'map_id':'example'}
        with self.assertRaises(ValueError):mission(n,{'_sdk_expected':expected,'reception_command':{'command':'pause'}})
    def test_map_reference_change_rejects_old_command(self):
        n=self.node();expected={'generation':3,'run_id':'r','epoch':2,'map_id':'old-map'}
        with self.assertRaises(ValueError):stop(n,{'_sdk_expected':expected})
        self.assertEqual(n.stops,0)
    def test_cancel_also_ends_paused_resumable_mission(self):
        n=self.node();n.reception.engine.phase='paused'
        n.reception.engine.cancel=lambda:setattr(n.reception.engine,'phase','cancelled')
        stop(n,{'_sdk_expected':{'generation':3,'run_id':'r','epoch':2,'map_id':'example'}})
        self.assertEqual(n.reception.engine.phase,'cancelled');self.assertEqual(n.stops,1)
    def test_session_prepare_rejects_stale_generation_atomically(self):
        s=NavigationSession();old=s.generation;s.prepare(dict(x=0,y=0,yaw=0),0.)
        with self.assertRaises(ValueError):s.prepare(dict(x=1,y=0,yaw=0,_sdk_expected_generation=old),0.)
        self.assertEqual(s.goal['x'],0.)

class ActualGoalGuardTests(unittest.TestCase):
    def test_new_mission_cannot_be_overwritten_between_sdk_snapshot_and_prepare(self):
        from test_position_goals import PositionGoalTests
        setup=PositionGoalTests();c=setup.setup_console();self.addCleanup(setup.doCleanups)
        c.reception=NS(source_recovery_active=False,map_id=lambda:'example',engine=NS(run_id='new-app-mission',epoch=2,phase='navigating'))
        expected={'generation':c.session.generation,'run_id':'old-sdk-mission','epoch':1,'map_id':'example'}
        with self.assertRaises(ValueError):
            c.set_goal(dict(x=1.,y=1.,yaw=0.,_sdk_expected_generation=c.session.generation,_sdk_expected=expected))
        self.assertFalse(c.planner.sent);self.assertFalse(c.nav.sent)
