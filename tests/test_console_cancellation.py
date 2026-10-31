"""Exercise the actual Console action tracking with controlled action futures.
No ROS imports or transport/motors are involved; callback order is deliberate.
"""
import ast,unittest,threading,time,json
from pathlib import Path
from concurrent.futures import Future
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from session import NavigationSession

source=Path(__file__).resolve().parents[1]/'navigation/console.py'
cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Console')
ns={'Node':object,'time':time,'json':json};exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),ns)
Console=ns['Console']
class Handle:
    accepted=True
    def __init__(self):self.result=Future();self.cancel_count=0
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.cancel_count+=1;return Future()
class Result:
    status=5

def fixture():
    c=Console.__new__(Console);c.lock=threading.RLock();c.session=NavigationSession()
    now=time.monotonic();c.session.operator('navigation_allowed',True,True,False,1,1,now,now)
    g=c.session.prepare({'x':1,'y':1,'yaw':0},now)
    c.goal_handle=c.plan_handle=None;c.pending_nav_accept=c.pending_plan_accept=False
    c.action_results={};c.accepted_actions={};c.cancel_sent_handles=[];c.speed_value=.15
    c.goal_generation=g;c.emitted=[];c.op=lambda **kw:c.emitted.append(kw);return c,g

class ConsoleCancellationTests(unittest.TestCase):
    def test_collision_source_fault_cancels_goal_and_recovery_does_not_resume(self):
        c,g=fixture();c.session.planned(g,8,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f)
        message=type('Message',(),{})();message.data=json.dumps({'collision_source_valid':False})
        c.on_gate(message)
        self.assertEqual(h.cancel_count,1);self.assertTrue(c.session.cancel_pending)
        message.data=json.dumps({'collision_source_valid':True,'data_fresh':True,'localized':True,'ground_ok':True,'scan_age':0.,'odom_age':0.})
        c.on_gate(message);self.assertTrue(c.health_ready())
        self.assertNotEqual(c.session.phase,'executing')
        self.assertEqual(c.emitted[-1],{'armed':False,'paused':True})
    def test_cancel_before_delayed_nav_acceptance_waits_for_terminal(self):
        c,g=fixture();c.session.planned(g,8,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        c.pending_nav_accept=True;c.session.cancel();c.stop_actions();self.assertTrue(c.session.cancel_pending)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f)
        self.assertEqual(h.cancel_count,1);self.assertTrue(c.session.cancel_pending)
        self.assertFalse(any(v.get('armed') is True for v in c.emitted))
        h.result.set_result(Result());self.assertFalse(c.session.cancel_pending);self.assertEqual(c.session.phase,'cancelled')
    def test_planning_cancel_does_not_restore_late_path(self):
        c,g=fixture();c.pending_plan_accept=True;c.session.cancel();h=Handle();f=Future();f.set_result(h)
        c.track_acceptance('plan',g,f);self.assertEqual(h.cancel_count,1);self.assertTrue(c.session.cancel_pending)
        self.assertFalse(c.session.planned(g,30,time.monotonic()));h.result.set_result(Result());self.assertFalse(c.session.cancel_pending)
    def test_planner_completion_cannot_ack_pending_live_nav_goal(self):
        c,g=fixture();plan=Handle();f=Future();f.set_result(plan);c.track_acceptance('plan',g,f)
        c.session.planned(g,8,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        nav=Handle();f=Future();f.set_result(nav);c.track_acceptance('nav',g,f);c.session.cancel();c.stop_actions()
        plan.result.set_result(Result());self.assertTrue(c.session.cancel_pending)
        nav.result.set_result(Result());self.assertFalse(c.session.cancel_pending)
    def test_live_confirm_never_infers_bridge_ready_from_switch(self):
        c,g=fixture();c.session.planned(g,8,time.monotonic());c.simulation=False;c.health_ready=lambda:True
        with self.assertRaisesRegex(ValueError,'尚未接通'):c.confirm_goal({'generation':g})
        self.assertEqual(c.session.phase,'ready');self.assertFalse(c.emitted)
    def test_repeated_cancel_rpc_is_not_acknowledgement(self):
        c,g=fixture();c.session.planned(g,8,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f);c.session.cancel()
        c.stop_actions();c.stop_actions();self.assertEqual(h.cancel_count,1);self.assertTrue(c.session.cancel_pending)
    def test_acceptance_transport_failure_cannot_prove_rejection(self):
        c,g=fixture();c.pending_plan_accept=True;f=Future();f.set_exception(RuntimeError('lost reply'))
        self.assertIsNone(c.track_acceptance('plan',g,f));c.stop_actions()
        self.assertTrue(c.session.cancel_pending);self.assertTrue(c.pending_plan_accept)
        with self.assertRaises(ValueError):c.session.prepare({'x':2,'y':1,'yaw':0},time.monotonic())
    def test_terminal_transport_failure_keeps_zero_and_cancel_latched(self):
        c,g=fixture();c.session.planned(g,8,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        h=Handle();f=Future();f.set_result(h);c.track_acceptance('nav',g,f)
        h.result.set_exception(RuntimeError('terminal reply unavailable'))
        self.assertTrue(c.session.cancel_pending);self.assertIs(c.goal_handle,h)
        self.assertEqual(h.cancel_count,1);self.assertEqual(c.emitted[-1],{'armed':False,'paused':True})

if __name__=='__main__':unittest.main()
