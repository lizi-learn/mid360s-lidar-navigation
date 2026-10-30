"""Run the actual ROS adapter tick with controlled clock and publisher."""
import ast,unittest,time,math,json
from pathlib import Path
from types import SimpleNamespace as NS
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from motion_buffer import MotionBuffer
p=Path(__file__).resolve().parents[1]/'navigation/motion_buffer_node.py'
cls=next(x for x in ast.parse(p.read_text()).body if isinstance(x,ast.ClassDef))
class Twist:
    def __init__(self):self.linear=NS(x=0.,y=0.);self.angular=NS(z=0.)
ns={'Node':object,'time':time,'math':math,'json':json,'Twist':Twist,'MotionBuffer':MotionBuffer}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(p),'exec'),ns)
class AdapterTests(unittest.TestCase):
    def fixture(self):
        n=ns['MotionBufferNode'].__new__(ns['MotionBufferNode']);now=time.monotonic()
        n.buffer=MotionBuffer();n.buffer.v=.1;n.buffer.last=now-.02;n.buffer.phase='forward'
        n.target=(0.,0.,0.);n.measured=(.1,0.,0.);n.command_time=now
        n.armed=True;n.paused=False;n.collision=False;n.reference=True;n.reference_time=now
        n.odom_stamp=100.;n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_001_000_000))
        n.sent=[];n.output=NS(publish=n.sent.append);return n
    def test_zero_without_explicit_normal_intent_is_immediate(self):
        n=self.fixture();n.tick();self.assertEqual(n.sent[-1].linear.x,0.)
        self.assertEqual(n.buffer.phase,'immediate_zero')
    def test_collision_does_not_wait_for_ramp(self):
        n=self.fixture();n.target=(.15,0.,0.);n.collision=True;n.tick()
        self.assertEqual(n.sent[-1].linear.x,0.)
if __name__=='__main__':unittest.main()
