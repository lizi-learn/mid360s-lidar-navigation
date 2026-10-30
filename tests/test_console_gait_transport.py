import unittest,time
from concurrent.futures import Future
from test_console_cancellation import fixture,Handle,Result

class Gait:
    def __init__(self):self.phase='idle';self.events=[];self.reject=False
    def ready(self):return True
    def operator_authority(self):return None
    def arm(self,generation):
        self.events.append(('arm',generation))
        if self.reject:raise ValueError('lease rejected')
        self.phase='active'
    def stop(self):self.events.append(('stop',));self.phase='stopped'
    def acknowledge_cancel(self):self.events.append(('ack',))
    def status(self):return {'phase':self.phase,'cancel_pending':False}
    def update(self,*args):pass

class Server:
    def __init__(self):self.calls=0
    def wait_for_server(self,timeout_sec):self.calls+=1;return False

class ConsoleGaitTests(unittest.TestCase):
    def setup_console(self):
        c,g=fixture();c.simulation=False;c.gait=Gait();c.health_ready=lambda:True
        c.checked_pose=lambda goal:object();c.nav=Server();c.session.planned(g,10,time.monotonic());return c,g
    def test_gait_reply_is_physical_authority_and_duplicate_does_not_renew(self):
        c,g=self.setup_console();now=time.monotonic()
        d={'mode':'navigation_allowed','verified':True,'connected':True,'activity':False,
            'source':77,'sequence':1,'stamp':now,'reply_stamp':now}
        c.gait.operator_authority=lambda:d
        c.poll_gait_authority(now);self.assertEqual(c.session.source,77)
        c.poll_gait_authority(now+.02);self.assertTrue(c.session.verified)
        self.assertEqual(c.session.source_time,now)
        c.poll_gait_authority(now+.101);self.assertFalse(c.session.verified)
    def test_alternative_ros_source_cannot_override_attached_gait(self):
        c,g=self.setup_console();source=c.session.source
        c.on_authority(object());self.assertEqual(c.session.source,source)
    def test_gait_source_restart_cancels_prepared_goal(self):
        c,g=self.setup_console();now=time.monotonic()
        d={'mode':'navigation_allowed','verified':True,'connected':True,'activity':False,
            'source':1,'sequence':2,'stamp':now,'reply_stamp':now}
        c.gait.operator_authority=lambda:d;c.poll_gait_authority(now)
        d.update(source=2,sequence=1,stamp=now+.01,reply_stamp=now+.01)
        c.poll_gait_authority(now+.01);self.assertEqual(c.session.phase,'stopped')
        self.assertTrue(c.session.cancel_pending)
    def test_real_zero_handshake_precedes_nav_action(self):
        c,g=self.setup_console()
        with self.assertRaisesRegex(ValueError,'导航服务未就绪'):c.confirm_goal({'generation':g})
        self.assertEqual(c.gait.events[0],('arm',g));self.assertIn(('stop',),c.gait.events)
        self.assertEqual(c.nav.calls,1);self.assertFalse(c.session.cancel_pending)
    def test_rejected_lease_prevents_nav_action(self):
        c,g=self.setup_console();c.gait.reject=True
        with self.assertRaisesRegex(ValueError,'lease rejected'):c.confirm_goal({'generation':g})
        self.assertEqual(c.nav.calls,0);self.assertIn(('stop',),c.gait.events)
    def test_cancel_ack_waits_for_actual_terminal_result(self):
        c,g=self.setup_console();c.session.start(g,time.monotonic(),True,True)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f)
        c.session.cancel();c.stop_actions();self.assertNotIn(('ack',),c.gait.events)
        h.result.set_result(Result());self.assertIn(('ack',),c.gait.events)
    def test_disconnect_cancels_active_goal_and_does_not_rearm(self):
        c,g=self.setup_console();c.session.start(g,time.monotonic(),True,True)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f)
        c.gait.phase='disconnected';c.gate={};c.poll_session()
        self.assertIn(c.session.phase,('cancelled','stopped'));self.assertEqual(h.cancel_count,1)
        self.assertTrue(c.session.cancel_pending);self.assertEqual(c.emitted[-1],{'armed':False,'paused':True})
        self.assertFalse(any(e[0]=='arm' for e in c.gait.events))

if __name__=='__main__':unittest.main()
