"""20 Hz original health samples must survive the actual RPC zero handshake."""
import sys,time,threading,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
import gait_transport
from gait_socket_client import Operation

class ClockCheckingClient:
    latest=None
    def __init__(self,*args):
        self.armed=False;self.owned=False;self.arm_at=0;self.previous=0;self.frames=[];self.failure='';ClockCheckingClient.latest=self
    def exchange(self,operation,authority,velocity=(0.,0.,0.),command_stamp_ns=None):
        now=time.monotonic_ns()
        if operation==Operation.ARM:self.armed=True;self.arm_at=now
        if operation==Operation.STOP:self.armed=self.owned=False
        if operation==Operation.VELOCITY:
            stamp=now if command_stamp_ns is None else command_stamp_ns
            self.frames.append((velocity,stamp,now))
            if stamp<self.arm_at:self.failure='navigation_packet_predates_arm'
            elif stamp<self.previous:self.failure='navigation_source_clock_rollback'
            if self.failure:self.armed=self.owned=False
            else:self.owned=True;self.previous=stamp
        return SimpleNamespace(monotonic_ns=now,readiness=495,operator_verified=True,operator_mode=1,
            manual_activity=False,cancel_pending=False,armed=self.armed,navigation_owned=self.owned,
            receiver_instance=1,receiver_generation=1,native_instance=1,native_stamp=now,
            native_sequence=now,reason=self.failure or 'navigation_active')
    def close(self):pass

class HandshakeClockTests(unittest.TestCase):
    def test_original_20hz_samples_cross_arm_fence_without_rollback_or_restamping(self):
        samples=set();done=threading.Event()
        with patch.object(gait_transport,'GaitClient',ClockCheckingClient):
            t=gait_transport.GaitTransport('synthetic-no-hardware')
            def publish():
                while not done.is_set():
                    stamp=time.monotonic_ns();samples.add(stamp)
                    t.update({'evaluated_monotonic_ns':stamp,'data_fresh':True,'calibration_ready':True,
                        'ground_ok':True,'localized':True,'command':(.075,0.,0.)},True)
                    done.wait(.05)
            writer=threading.Thread(target=publish);writer.start()
            try:
                end=time.monotonic()+1
                while not t.ready() and time.monotonic()<end:time.sleep(.002)
                t.arm(1);time.sleep(.18);c=ClockCheckingClient.latest
                self.assertEqual(c.failure,'');self.assertEqual(t.phase,'active');self.assertTrue(t.feedback.navigation_owned)
                self.assertGreaterEqual(len(c.frames),3)
                self.assertEqual(c.frames[0][0],(0.,0.,0.))
                self.assertTrue(any(v[0]>.07 for v,_,_ in c.frames[1:]))
                self.assertTrue(all(stamp in samples for _,stamp,_ in c.frames),'Never replace source time with RPC time')
                self.assertTrue(all(stamp>=c.arm_at for _,stamp,_ in c.frames))
                self.assertEqual(sorted(stamp for _,stamp,_ in c.frames),[stamp for _,stamp,_ in c.frames])
            finally:done.set();writer.join();t.close()

if __name__=='__main__':unittest.main()
