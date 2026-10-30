import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from session import NavigationSession

class SessionTests(unittest.TestCase):
    def op(self,s,t,mode='navigation_allowed',seq=None,source=1,manual=False):
        return s.operator(mode,True,True,manual,source,seq or int(t*1000),t,t)
    def ready(self,s,t=10.):
        self.op(s,t);g=s.prepare({'x':1.,'y':2.,'yaw':0.},t);self.assertTrue(s.planned(g,8,t));return g
    def active(self,s,t=10.):
        g=self.ready(s,t);s.start(g,t,True,True);self.assertTrue(s.started(g,t));return g
    def test_left_does_not_start_and_confirmation_checks_health(self):
        s=NavigationSession();g=self.ready(s)
        self.assertEqual(s.phase,'ready');self.assertTrue(s.status(10.)['force_navigation_zero'])
        with self.assertRaisesRegex(ValueError,'尚未接通'):s.start(g,10.,False,True)
        with self.assertRaisesRegex(ValueError,'未就绪'):s.start(g,10.,True,False)
        self.assertEqual(s.phase,'ready')
    def test_right_wins_without_new_navigation_packet_and_left_no_resume(self):
        s=NavigationSession();g=self.active(s);self.op(s,10.001,'manual')
        x=s.poll(10.001);self.assertTrue(x['force_navigation_zero']);self.assertTrue(x['cancel_pending']);self.assertEqual(s.phase,'manual')
        self.op(s,10.002);self.assertEqual(s.phase,'manual');self.assertFalse(s.started(g,10.002))
        with self.assertRaises(ValueError):s.prepare({'x':2,'y':3,'yaw':0},10.002)
        self.assertFalse(s.acknowledge_cancel(g+1));self.assertTrue(s.acknowledge_cancel(g))
        new=s.prepare({'x':2,'y':3,'yaw':0},10.002);self.assertGreater(new,g)
    def test_middle_and_stick_cancel(self):
        for mode,manual in [('paused',False),('navigation_allowed',True)]:
            s=NavigationSession();self.active(s);self.op(s,10.001,mode,manual=manual)
            self.assertTrue(s.poll(10.001)['force_navigation_zero']);self.assertTrue(s.cancel_pending)
    def test_cancel_during_planning_or_before_action_acceptance(self):
        for phase in ['planning','ready','starting']:
            s=NavigationSession();g=self.ready(s);s.phase=phase;s.cancel()
            self.assertFalse(s.planned(g,9,10.));self.assertFalse(s.started(g,10.));self.assertEqual(s.phase,'cancelled')
    def test_stale_replayed_changed_source_and_clock_rollback_stop(self):
        for failure in ['stale','replay','source','rollback']:
            s=NavigationSession();self.active(s);s.poll(10.)
            if failure=='stale':s.poll(10.101)
            if failure=='replay':self.op(s,10.,seq=10000)
            if failure=='source':self.op(s,10.001,source=2)
            if failure=='rollback':s.poll(9.999)
            self.assertNotEqual(s.phase,'executing');self.assertTrue(s.cancel_pending)
    def test_obstacle_requires_new_path_and_continuous_clear(self):
        s=NavigationSession();self.active(s);s.obstacle(10.)
        self.assertTrue(s.status(10.)['force_navigation_zero'])
        s.reroute(10.1,False,True);s.reroute(10.9,False,True);self.assertEqual(s.phase,'replanning')
        self.op(s,10.91);s.reroute(10.91,True,True);self.assertEqual(s.phase,'executing')
    def test_blocked_obstacle_has_bounded_zero_and_notification(self):
        s=NavigationSession();self.active(s);s.obstacle(10.)
        for i in range(3):s.reroute(10.01+i*.01,True,False)
        self.assertEqual(s.phase,'blocked');self.assertTrue(s.status(10.03)['force_navigation_zero']);self.assertIn('归零',s.message)
        s=NavigationSession();self.active(s);s.obstacle(10.);s.reroute(18.,False,True)
        self.assertEqual(s.phase,'blocked');self.assertIn('无法绕行',s.message)
    def test_no_old_result_can_complete_new_goal(self):
        s=NavigationSession();old=self.active(s);s.cancel();s.acknowledge_cancel(old);new=self.ready(s,10.01)
        self.assertFalse(s.finished(old,True));self.assertEqual(s.phase,'ready');self.assertGreater(new,old)
    def test_preview_without_hardware_does_not_gain_authority(self):
        s=NavigationSession();g=s.prepare({'x':0,'y':0,'yaw':0},10);s.planned(g,1,10);s.poll(10.01)
        self.assertEqual(s.phase,'ready')
        with self.assertRaises(ValueError):s.start(g,10.01,True,True)

if __name__=='__main__':unittest.main()
