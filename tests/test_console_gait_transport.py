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
    def test_confirm_after_slow_plan_consumes_current_original_authority(self):
        c,g=self.setup_console();c.session.source_time=time.monotonic()-2.1
        def actual():
            now=time.monotonic();return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.operator_authority=actual
        with self.assertRaisesRegex(ValueError,'导航服务未就绪'):c.confirm_goal({'generation':g})
        self.assertIn(('arm',g),c.gait.events)
        self.assertLess(time.monotonic()-c.session.source_time,.1)
    def test_confirm_consumes_real_manual_takeover_before_any_arm(self):
        c,g=self.setup_console()
        def actual():
            now=time.monotonic();return dict(mode='manual',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.operator_authority=actual
        with self.assertRaises(ValueError):c.confirm_goal({'generation':g})
        self.assertNotIn(('arm',g),c.gait.events);self.assertEqual(c.nav.calls,0)
        self.assertEqual(c.session.phase,'manual')
    def test_confirm_cannot_renew_a_truly_expired_original_packet(self):
        c,g=self.setup_console();stamp=time.monotonic()-2.1
        c.gait.operator_authority=lambda:dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
            source=c.session.source,sequence=c.session.sequence+1,stamp=stamp,reply_stamp=None,source_transport='native_packet_snapshot')
        with self.assertRaises(ValueError):c.confirm_goal({'generation':g})
        self.assertNotIn(('arm',g),c.gait.events);self.assertEqual(c.nav.calls,0)
        self.assertTrue(c.session.cancel_pending)
    def test_original_packet_is_read_after_waiting_for_session_lock(self):
        c,g=self.setup_console();old_lock=c.session.lock;entered=[]
        class Lock:
            def __enter__(self):old_lock.acquire();entered.append(True)
            def __exit__(self,*args):entered.pop();old_lock.release()
        c.session.lock=Lock()
        def sample():
            self.assertTrue(entered,'Packet was read before acquiring session lock')
            now=time.monotonic();return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=now)
        c.gait.operator_authority=sample;c.poll_gait_authority(time.monotonic())
        self.assertEqual(c.session.phase,'ready');self.assertTrue(c.session.verified)
    def test_fresh_left_packet_preserves_original_stop_cause(self):
        c,g=self.setup_console();now=time.monotonic();c.session.start(g,now,True,True);c.session.started(g,now)
        c.session.poll(now+2.001);reason=c.session.message
        self.assertEqual(c.session.phase,'stopped');self.assertIn('过期',reason)
        fresh=time.monotonic();c.session.operator('navigation_allowed',True,True,False,c.session.source,c.session.sequence+1,fresh,fresh)
        self.assertEqual(c.session.message,reason);self.assertEqual(c.session.last_stop['reason'],reason)
        self.assertEqual(c.session.phase,'stopped');self.assertTrue(c.session.status(fresh)['force_navigation_zero'])
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
        c.poll_gait_authority(now+2.001);self.assertFalse(c.session.verified)
    def test_alternative_ros_source_cannot_override_attached_gait(self):
        c,g=self.setup_console();source=c.session.source
        c.on_authority(object());self.assertEqual(c.session.source,source)
    def test_original_switch_snapshot_has_no_fabricated_controller_reply(self):
        c,g=self.setup_console();now=time.monotonic();c.session.source=77;c.session.sequence=10
        d={'mode':'navigation_allowed','verified':True,'connected':True,'activity':False,
            'source':77,'sequence':11,'stamp':now,'reply_stamp':None,'source_transport':'native_packet_snapshot'}
        c.gait.operator_authority=lambda:d;c.poll_gait_authority(now);self.assertTrue(c.session.verified)
        c.poll_gait_authority(now+.16);self.assertTrue(c.session.verified)
        self.assertEqual(c.session.source_time,now)
        c.poll_gait_authority(now+2.001);self.assertFalse(c.session.verified)
    def test_slow_gait_status_cannot_turn_new_operator_time_into_the_future(self):
        c,g=self.setup_console();c.gate={};c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic());c.gait.phase='active'
        def actual_source():
            now=time.monotonic();return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=now)
        c.gait.operator_authority=actual_source
        def delayed_status():
            time.sleep(.025);now=time.monotonic();c.session.operator('navigation_allowed',True,True,False,c.session.source,c.session.sequence+1,now,now)
            return dict(phase='active',cancel_pending=False)
        c.gait.status=delayed_status;c.poll_session()
        self.assertEqual(c.session.phase,'executing');self.assertTrue(c.emitted[-1]['armed']);self.assertFalse(c.emitted[-1]['paused'])
    def test_descheduled_ui_read_retries_actual_packet_instead_of_false_cancel(self):
        c,g=self.setup_console();c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic());calls=[]
        def sample():
            stamp=time.monotonic();calls.append(stamp)
            if len(calls)==1:time.sleep(.12)
            return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+len(calls),stamp=stamp,reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.operator_authority=sample;c.poll_gait_authority(time.monotonic())
        self.assertEqual(len(calls),1);self.assertEqual(c.session.phase,'executing');self.assertFalse(c.session.cancel_pending)
        self.assertEqual(c.session.source_time,calls[0])
    def test_retry_does_not_renew_truly_expired_native_packet(self):
        c,g=self.setup_console();c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic());stamp=time.monotonic()-2.01;calls=[]
        def sample():
            calls.append(True);return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=stamp,reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.operator_authority=sample;c.poll_gait_authority(time.monotonic())
        self.assertEqual(len(calls),2);self.assertEqual(c.session.phase,'stopped');self.assertTrue(c.session.cancel_pending)
    def test_slow_ui_rpc_resamples_original_native_packet_before_velocity_intent(self):
        c,g=self.setup_console();c.gate={};c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic());c.gait.phase='active'
        def actual_source():
            now=time.monotonic();return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.operator_authority=actual_source
        def slow_status():
            # The original native packet continues outside this UI process,
            # but its Python observer cannot run during UI GIL work.
            time.sleep(.15);return dict(phase='active',cancel_pending=False)
        c.gait.status=slow_status;c.poll_session()
        self.assertEqual(c.session.phase,'executing');self.assertTrue(c.emitted[-1]['armed'])
        self.assertFalse(c.emitted[-1]['paused'])
    def test_observer_update_between_clock_read_and_session_poll_is_not_expiry(self):
        import inspect
        from unittest.mock import patch
        c,g=self.setup_console();c.gate={};c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic());c.gait.phase='active'
        def observe():
            now=time.monotonic();return dict(mode='navigation_allowed',verified=True,connected=True,activity=False,
                source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=now)
        c.gait.operator_authority=observe
        actual_time=time;reads=0
        class Clock:
            @staticmethod
            def monotonic():
                nonlocal reads
                now=actual_time.monotonic()
                if inspect.currentframe().f_back.f_code.co_name=='poll_session':
                    reads+=1
                    if reads==2:
                        # The separate real-source observer commits while this
                        # UI caller is descheduled after reading its clock.
                        actual_time.sleep(.025);fresh=actual_time.monotonic()
                        c.session.operator('navigation_allowed',True,True,False,c.session.source,c.session.sequence+1,fresh,fresh)
                return now
        with patch.dict(c.poll_session.__globals__,time=Clock):c.poll_session()
        self.assertEqual(c.session.phase,'executing');self.assertTrue(c.emitted[-1]['armed'])
        self.assertFalse(c.emitted[-1]['paused']);self.assertTrue(c.session.verified)
    def test_out_of_order_read_only_native_snapshots_do_not_cancel(self):
        c,g=self.setup_console();now=time.monotonic()
        c.session.source=77;c.session.sequence=100;c.session.source_time=now-.04
        newer={'mode':'navigation_allowed','verified':True,'connected':True,'activity':False,
            'source':77,'sequence':102,'stamp':now,'reply_stamp':now}
        c.gait.operator_authority=lambda:newer;c.poll_gait_authority(now)
        c.gait.operator_authority=lambda:{**newer,'sequence':101,'stamp':now-.02,'reply_stamp':now-.01}
        c.poll_gait_authority(now+.01)
        self.assertEqual(c.session.sequence,102);self.assertEqual(c.session.source_time,now)
        self.assertEqual(c.session.phase,'ready');self.assertFalse(c.session.cancel_pending)
        c.gait.operator_authority=lambda:{**newer,'sequence':103,'activity':True,'stamp':now+.02,'reply_stamp':now+.02}
        c.poll_gait_authority(now+.02)
        self.assertEqual(c.session.phase,'manual');self.assertTrue(c.session.cancel_pending)
    def test_gait_source_restart_cancels_prepared_goal(self):
        c,g=self.setup_console();now=time.monotonic()
        d={'mode':'navigation_allowed','verified':True,'connected':True,'activity':False,
            'source':1,'sequence':2,'stamp':now,'reply_stamp':now}
        c.gait.operator_authority=lambda:d;c.poll_gait_authority(now)
        d.update(source=2,sequence=1,stamp=now+.01,reply_stamp=now+.01)
        c.poll_gait_authority(now+.01);self.assertEqual(c.session.phase,'stopped')
        self.assertTrue(c.session.cancel_pending)
    def test_real_zero_handshake_precedes_nav_action(self):
        c,g=self.setup_console();now=time.monotonic()
        c.gait.operator_authority=lambda:dict(mode='navigation_allowed',verified=True,connected=True,activity=False,source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=now)
        with self.assertRaisesRegex(ValueError,'导航服务未就绪'):c.confirm_goal({'generation':g})
        self.assertEqual(c.gait.events[0],('arm',g));self.assertIn(('stop',),c.gait.events)
        self.assertEqual(c.nav.calls,1);self.assertFalse(c.session.cancel_pending)
    def test_rejected_lease_prevents_nav_action(self):
        c,g=self.setup_console();c.gait.reject=True;now=time.monotonic()
        c.gait.operator_authority=lambda:dict(mode='navigation_allowed',verified=True,connected=True,activity=False,source=c.session.source,sequence=c.session.sequence+1,stamp=now,reply_stamp=now)
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

class ConfirmAcceptancePhaseTests(unittest.TestCase):
    setup_console=ConsoleGaitTests.setup_console
    def accepted_console(self,phase):
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import patch
        import tempfile
        c,g=self.setup_console();tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        c.runtime=Path(tmp.name);c.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=time.monotonic_ns()))
        globals_=c.confirm_goal.__func__.__globals__
        globals_['Path']=Path;globals_['__file__']=str(Path(__file__).resolve().parents[1]/'navigation/console.py')
        globals_['NavigateToPose']=SimpleNamespace(Goal=SimpleNamespace)
        c.gait.operator_authority=lambda:dict(mode='navigation_allowed',verified=True,connected=True,activity=False,source=c.session.source,sequence=c.session.sequence+1,stamp=time.monotonic(),reply_stamp=None,source_transport='native_packet_snapshot')
        c.gait.status=lambda:dict(phase=c.gait.phase,cancel_pending=False,navigation_owned=c.gait.phase=='active')
        handle=Handle();future=Future();future.set_result(handle)
        c.nav=SimpleNamespace(wait_for_server=lambda timeout_sec:True,send_goal_async=lambda goal,feedback_callback:future)
        original=c.track_acceptance
        def acceptance(*args):
            result=original(*args)
            if phase=='replanning':c.session.obstacle(time.monotonic())
            elif phase=='arrived':c.session.finished(g,True)
            elif phase=='manual':
                fresh=time.monotonic();c.session.operator('manual',True,True,False,c.session.source,c.session.sequence+1,fresh,fresh)
            return result
        c.track_acceptance=acceptance
        return c,g,handle
    def test_accepted_goal_can_enter_checked_obstacle_wait_before_http_returns(self):
        c,g,h=self.accepted_console('replanning');r=c.confirm_goal({'generation':g})
        self.assertTrue(r['accepted']);self.assertEqual(c.session.phase,'replanning');self.assertEqual(h.cancel_count,0)
        self.assertTrue(c.session.status(time.monotonic())['force_navigation_zero'])
    def test_actual_success_before_http_returns_is_accepted_without_new_motion(self):
        c,g,h=self.accepted_console('arrived');r=c.confirm_goal({'generation':g})
        self.assertTrue(r['accepted']);self.assertEqual(c.session.phase,'arrived')
        self.assertTrue(c.session.status(time.monotonic())['force_navigation_zero'])
    def test_actual_manual_takeover_before_http_returns_is_not_accepted(self):
        c,g,h=self.accepted_console('manual')
        with self.assertRaises(ValueError):c.confirm_goal({'generation':g})
        self.assertEqual(c.session.phase,'manual');self.assertTrue(c.session.status(time.monotonic())['force_navigation_zero'])

if __name__=='__main__':unittest.main()
