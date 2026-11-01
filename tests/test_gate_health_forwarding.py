"""The real Console callback forwards the original gate decision immediately."""
import json,time,unittest
from test_console_cancellation import fixture

class HealthForwardingTests(unittest.TestCase):
    def test_received_gate_is_forwarded_before_the_next_session_timer(self):
        c,g=fixture();c.session.planned(g,10,time.monotonic());c.session.start(g,time.monotonic(),True,True);c.session.started(g,time.monotonic())
        class Gait:
            def __init__(self):self.samples=[]
            def update(self,sample,enabled):self.samples.append((sample.copy(),enabled))
        c.gait=Gait();stamp=time.monotonic_ns()-40_000_000
        source={'collision_source_valid':True,'evaluated_monotonic_ns':stamp,'command':[.01,0.,0.],
            'scan_age':.16,'odom_age':.08,'ground_ok':True,'localized':True,'data_fresh':True}
        c.on_gate(type('M',(),{'data':json.dumps(source)})())
        self.assertEqual(c.gait.samples,[(source,True)])
        self.assertEqual(c.gait.samples[0][0]['evaluated_monotonic_ns'],stamp)
        self.assertEqual(c.gait.samples[0][0]['scan_age'],.16)
        c.session.cancel();c.on_gate(type('M',(),{'data':json.dumps(source)})())
        self.assertFalse(c.gait.samples[-1][1])

if __name__=='__main__':unittest.main()
