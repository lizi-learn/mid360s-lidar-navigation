import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from localization_health import source_age,pose_discontinuous
class HealthTests(unittest.TestCase):
    def test_freshly_received_old_odometry_is_still_stale(self):
        self.assertAlmostEqual(source_age(100.,98.5,.001),1.5)
    def test_clock_mismatch_and_unknown_stamps_are_rejected(self):
        self.assertGreater(source_age(100.,101.,0),3)
        self.assertGreater(source_age(100.,0.,0),3)
        self.assertAlmostEqual(source_age(100.,100.05,.01),.01)
    def test_actual_stationary_fault_jumps_are_detected(self):
        self.assertTrue(pose_discontinuous((0,0),(.65,.292),.1008))
        self.assertTrue(pose_discontinuous((0,0),(.368,.113),.0992))
    def test_supported_trolley_speeds_and_jitter_are_allowed(self):
        self.assertFalse(pose_discontinuous((0,0),(.13,0),.1))
        self.assertFalse(pose_discontinuous((0,0),(.001,.001),.001))
        self.assertFalse(pose_discontinuous((0,0),(1.,0),1.))
    def test_backward_pose_time_is_not_silently_accepted(self):
        self.assertTrue(pose_discontinuous((0,0),(0,0),-.1))
if __name__=='__main__':unittest.main()
