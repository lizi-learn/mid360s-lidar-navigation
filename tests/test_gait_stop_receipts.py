"""Reproduce a queued STOP with a still-fresh, pre-STOP native reply."""
import threading,time,unittest
from unittest.mock import patch
import gait_transport
from gait_socket_client import Operation
from test_zero_handshake_timestamp import ClockCheckingClient

class DelayedStopClient(ClockCheckingClient):
 latest=None
 def __init__(self,*args):
  super().__init__(*args);DelayedStopClient.latest=self;self.pending=False;self.tracking=None
  self.stop_entered=threading.Event();self.release_stop=threading.Event();self.ack_entered=threading.Event();self.release_ack=threading.Event();self.operations=[]
 def exchange(self,operation,*args,**kw):
  self.operations.append(operation)
  if operation==Operation.STOP:
   self.stop_entered.set();self.release_stop.wait(.5);self.pending=True
  if operation==Operation.CANCEL_ACK:
   self.ack_entered.set();self.release_ack.wait(.5);self.pending=False
  r=super().exchange(operation,*args,**kw)
  r.cancel_pending=self.pending;r.controller=1;r.generation=2;r.transport_epoch=3;r.cancel_generation=4
  return r

class StopReceiptTests(unittest.TestCase):
 def test_queued_stop_and_cancel_ack_do_not_expose_the_old_ready_reply(self):
  with patch.object(gait_transport,'GaitClient',DelayedStopClient):
   t=gait_transport.GaitTransport('synthetic-no-hardware')
   try:
    end=time.monotonic()+1.
    while not t.ready()and time.monotonic()<end:time.sleep(.002)
    self.assertTrue(t.ready());c=DelayedStopClient.latest;t.stop()
    self.assertTrue(c.stop_entered.wait(.2))
    self.assertFalse(t.feedback.cancel_pending,'Test must expose the pre-STOP reply')
    self.assertLess(time.monotonic_ns()-t.feedback.monotonic_ns,100_000_000)
    self.assertFalse(t.ready(),'Queued STOP must not report ready before its actual reply')
    with self.assertRaises(ValueError):t.arm(9,.01)
    self.assertNotIn(Operation.ARM,c.operations)
    c.release_stop.set();end=time.monotonic()+1.
    while not t.feedback.cancel_pending and time.monotonic()<end:time.sleep(.002)
    self.assertTrue(t.feedback.cancel_pending);self.assertFalse(t.ready())
    t.acknowledge_cancel();self.assertTrue(c.ack_entered.wait(.2));self.assertFalse(t.ready())
    c.release_ack.set();end=time.monotonic()+1.
    while not t.ready()and time.monotonic()<end:time.sleep(.002)
    self.assertTrue(t.ready());self.assertEqual(t.stop_serial,t.sent_stop);self.assertIsNone(t.ack_requested)
    self.assertIn(Operation.CANCEL_ACK,c.operations)
   finally:
    if DelayedStopClient.latest:
     DelayedStopClient.latest.release_stop.set();DelayedStopClient.latest.release_ack.set()
    t.close()

if __name__=='__main__':unittest.main()
