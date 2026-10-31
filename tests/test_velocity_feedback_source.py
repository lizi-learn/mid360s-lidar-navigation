import ast, json, math, pathlib, sys, tempfile, time, unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation'))
from velocity_trace import VelocityTrace

class FeedbackSourceTests(unittest.TestCase):
 def test_ros_source_delay_is_retained_across_status_ticks(self):
  source=ROOT/'navigation/gate_node.py'
  cls=next(v for v in ast.parse(source.read_text()).body if isinstance(v,ast.ClassDef))
  ns={'Node':object,'time':time,'math':math};exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),ns)
  n=ns['GateNode'].__new__(ns['GateNode']);n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_190_000_000))
  msg=NS(header=NS(stamp=NS(sec=100,nanosec=0)),twist=NS(twist=NS(linear=NS(x=.02,y=0.),angular=NS(z=.01))))
  with patch('time.monotonic_ns',return_value=9_000_000_000):n.on_odom(msg)
  self.assertEqual(n.velocity_source_ns,8_810_000_000)
  self.assertEqual(n.measured,(.02,0.,.01))
  # Cached source stamp remains unchanged when the receiver clock advances.
  with patch('time.monotonic_ns',return_value=9_400_000_000):
   self.assertEqual(time.monotonic_ns()-n.velocity_source_ns,590_000_000)
 def test_private_trace_is_bounded_and_off_rpc_thread(self):
  with tempfile.TemporaryDirectory() as d:
   path=pathlib.Path(d)/'trace.jsonl';log=VelocityTrace(path,maximum_bytes=120)
   for i in range(8):
    log.last=0.;log.submit({'controller_tick':i,'policy_input_x':.24})
   log.close();self.assertFalse(log.thread.is_alive());self.assertEqual(log.error,'')
   rows=[json.loads(x) for x in path.read_text().splitlines()]
   self.assertEqual(rows[-1]['controller_tick'],7)
   self.assertTrue(path.with_name(path.name+'.1').exists())
   self.assertEqual(path.stat().st_mode&0o777,0o600)

if __name__=='__main__':unittest.main()
