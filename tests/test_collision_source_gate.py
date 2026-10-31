"""Exercise the real gate tick with source-loss and reauthorization transitions."""
import ast,json,math,time,unittest,sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gate import ForwardGate

p=Path(__file__).resolve().parents[1]/'navigation/gate_node.py'
cls=next(x for x in ast.parse(p.read_text()).body if isinstance(x,ast.ClassDef))
class Twist:
    def __init__(self):self.linear=NS(x=0.,y=0.);self.angular=NS(z=0.)
class String:
    def __init__(self):self.data=''
scope={'Node':object,'time':time,'math':math,'json':json,'ForwardGate':ForwardGate,'Twist':Twist,'String':String}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(p),'exec'),scope)

class CollisionSourceGateTests(unittest.TestCase):
    def fixture(self):
        n=scope['GateNode'].__new__(scope['GateNode']);now=time.monotonic()
        n.gate=ForwardGate();n.gate.armed=True;n.simulation=False;n.require_localization=False
        n.handoff=n.recovery=None;n.recovery_wait=False;n.collision_stop=False
        n.last=now-.05;n.command_time=now;n.command=(.1,0.,0.);n.invalid_command_until=0.
        n.scan_stamp=n.odom_stamp=100.;n.calibration_ready=True;n.calibration_time=now
        n.ground=True;n.ground_time=now;n.paused=False;n.measured=(0,0,0);n.covariance=None
        n.collision_source_valid=True;n.collision_source_time=now;n.counter=0
        n.arm_generation=4;n.collision_fault_generation=-1
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_010_000_000))
        n.sent=[];n.statuses=[];n.output=NS(publish=n.sent.append);n.status=NS(publish=n.statuses.append)
        return n
    def test_source_failure_disarms_and_recovery_needs_new_arm(self):
        n=self.fixture();n.on_collision_source(NS(data=False));n.tick()
        self.assertFalse(n.gate.armed);self.assertEqual(n.sent[-1].linear.x,0)
        self.assertFalse(json.loads(n.statuses[-1].data)['data_fresh'])
        n.on_collision_source(NS(data=True));n.tick()
        self.assertFalse(n.gate.armed);self.assertEqual(n.sent[-1].linear.x,0)
        n.on_operator(NS(data=json.dumps({'armed':True,'arm_generation':4})));n.tick()
        self.assertFalse(n.gate.armed);self.assertEqual(n.sent[-1].linear.x,0)
        n.on_operator(NS(data=json.dumps({'armed':True,'arm_generation':5})));n.tick()
        self.assertTrue(n.gate.armed);self.assertGreater(n.sent[-1].linear.x,0)
    def test_missing_stale_or_future_health_is_not_clear(self):
        for value,age in [(False,0),(True,.31),(True,-.1)]:
            with self.subTest(value=value,age=age):
                n=self.fixture();n.collision_source_valid=value;n.collision_source_time=time.monotonic()-age;n.tick()
                self.assertEqual(n.sent[-1].linear.x,0);self.assertFalse(n.gate.armed)
                self.assertFalse(json.loads(n.statuses[-1].data)['collision_source_valid'])
    def test_arm_while_source_invalid_is_rejected_by_tick(self):
        n=self.fixture();n.on_collision_source(NS(data=False))
        n.on_operator(NS(data=json.dumps({'armed':True,'arm_generation':5})));n.tick()
        self.assertFalse(n.gate.armed);self.assertEqual(n.sent[-1].linear.x,0)
    def test_arm_requires_explicit_generation_and_rejects_old_messages(self):
        for generation in [None,True,-1,3,'5']:
            with self.subTest(generation=generation):
                n=self.fixture();n.on_operator(NS(data=json.dumps({'armed':True,'arm_generation':generation})));n.tick()
                self.assertFalse(n.gate.armed);self.assertEqual(n.sent[-1].linear.x,0)

if __name__=='__main__':unittest.main()
