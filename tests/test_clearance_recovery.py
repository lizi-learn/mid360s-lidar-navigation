import unittest,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from clearance_recovery import ClearanceRecovery
class ClearanceRecoveryTests(unittest.TestCase):
 def test_clear_window_is_required_again_after_stop(self):
  c=ClearanceRecovery();self.assertFalse(c.step(0,True));self.assertTrue(c.step(1,True));self.assertFalse(c.step(1.1,False));self.assertFalse(c.step(1.2,True));self.assertFalse(c.step(1.8,True));self.assertTrue(c.step(2.,True))
 def test_chattering_observation_cannot_restart_motion(self):
  c=ClearanceRecovery()
  for i in range(100):self.assertFalse(c.step(i*.1,i%4!=0))
 def test_clock_rollback_or_nonfinite_time_requires_new_window(self):
  c=ClearanceRecovery();c.step(10,True);self.assertTrue(c.step(11,True));self.assertFalse(c.step(9,True));self.assertFalse(c.step(math.nan,True));self.assertFalse(c.step(20,True));self.assertTrue(c.step(21,True))
 def test_invalid_delay_rejected(self):
  for delay in (0,-1,math.nan,math.inf,6):
   with self.assertRaises(ValueError):ClearanceRecovery(delay)
