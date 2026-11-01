"""Slow telemetry must not block fresh health or cancellation callbacks."""
import sys,time,threading,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
import gait_transport
from gait_socket_client import Operation
from test_zero_handshake_timestamp import ClockCheckingClient

class DelayedClient(ClockCheckingClient):
    latest=None
    def __init__(self,*args):
        super().__init__(*args);self.entered=threading.Event();self.release=threading.Event()
        self.operations=[];self.tracking=None;self.once=True;DelayedClient.latest=self
    def exchange(self,operation,authority,velocity=(0.,0.,0.),command_stamp_ns=None):
        if operation==Operation.MEASURED_VELOCITY and self.once:
            self.once=False;self.entered.set();self.release.wait(.5)
        self.operations.append((operation,authority.health_stamp_ns,velocity))
        return super().exchange(operation,authority,velocity,command_stamp_ns)

def sample(stamp):
    return {'evaluated_monotonic_ns':stamp,'data_fresh':True,'calibration_ready':True,
        'ground_ok':True,'localized':True,'command':(.4,0.,0.),
        'body_velocity_feedback':(0.,0.,0.),'velocity_source_monotonic_ns':stamp}

class TransportLockTests(unittest.TestCase):
    def test_gate_update_and_stop_complete_while_socket_reply_is_blocked(self):
        with patch.object(gait_transport,'GaitClient',DelayedClient):
            t=gait_transport.GaitTransport('synthetic-no-hardware')
            try:
                t.update(sample(time.monotonic_ns()),True)
                end=time.monotonic()+1
                while DelayedClient.latest is None and time.monotonic()<end:time.sleep(.002)
                c=DelayedClient.latest;self.assertTrue(c.entered.wait(1.))
                completed=threading.Event();stamp=time.monotonic_ns()
                def callback():t.update(sample(stamp),True);t.stop();completed.set()
                worker=threading.Thread(target=callback);worker.start()
                try:
                    self.assertTrue(completed.wait(.05),'Health/Cancel blocked behind network receive')
                    self.assertEqual(t.health_stamp,stamp)
                    self.assertFalse(c.release.is_set())
                finally:c.release.set();worker.join(timeout=1.)
                end=time.monotonic()+1
                while not (any(x[0]==Operation.STOP for x in c.operations) and any(x[0]==Operation.AUTHORITY and x[1]==stamp for x in c.operations)) and time.monotonic()<end:time.sleep(.002)
                ops=[x[0] for x in c.operations]
                self.assertIn(Operation.STOP,ops)
                self.assertNotIn(Operation.ARM,ops)
                self.assertNotIn(Operation.VELOCITY,ops)
                self.assertTrue(any(x[0]==Operation.AUTHORITY and x[1]==stamp for x in c.operations))
                self.assertEqual(t.phase,'stopped')
            finally:
                if DelayedClient.latest:DelayedClient.latest.release.set()
                t.close()

if __name__=='__main__':unittest.main()
