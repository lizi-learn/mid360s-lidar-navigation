import math
from pathlib import Path
import sys, unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gate import ForwardGate

class ForwardMotionContract(unittest.TestCase):
    def setUp(self):self.g=ForwardGate();self.g.armed=True
    def step(self,command=(.25,0.,.2),**kw):
        params=dict(dt=.05,command_age=.02,scan_age=.1,odom_age=.02)
        params.update(kw);return self.g.step(command,**params)
    def test_forward_only_and_bounded_acceleration(self):
        out=[self.step() for _ in range(100)]
        self.assertTrue(all(0<=x<=.2 and y==0 and abs(w)<=.35 for x,y,w in out))
        self.assertTrue(all(b[0]-a[0]<=.15*.05+1e-12 for a,b in zip(out,out[1:])))
    def test_lateral_command_is_rejected(self):
        self.step();self.assertEqual(self.step((.1,.05,0)),(0.,0.,0.))
        self.assertEqual(self.g.reason,'lateral_command_rejected')
    def test_reverse_is_rejected(self):self.assertEqual(self.step((-.1,0,0)),(0.,0.,0.))
    def test_scan_dropout_forces_stop(self):
        self.step();self.assertEqual(self.step(scan_age=.51),(0.,0.,0.))
    def test_odom_dropout_forces_stop(self):self.assertEqual(self.step(odom_age=.51),(0.,0.,0.))
    def test_source_timestamp_in_future_is_rejected(self):self.assertEqual(self.step(scan_age=-1.),(0.,0.,0.))
    def test_command_expiry_forces_stop(self):self.assertEqual(self.step(command_age=.31),(0.,0.,0.))
    def test_zero_after_motion_is_immediate(self):
        for _ in range(40):self.step()
        self.assertEqual(self.step((0,0,0)),(0.,0.,0.))
    def test_lower_speed_cap_takes_effect_immediately(self):
        for _ in range(40):self.step()
        self.g.set_speed(.05);self.assertLessEqual(self.step()[0],.05)
    def test_unobserved_ground_forces_stop(self):self.assertEqual(self.step(ground_ok=False),(0.,0.,0.))
    def test_unlocalized_forces_stop(self):self.assertEqual(self.step(localized=False),(0.,0.,0.))
    def test_disarm_forces_stop(self):
        self.step();self.g.armed=False;self.assertEqual(self.step(),(0.,0.,0.))
    def test_pause_forces_stop(self):self.assertEqual(self.step(paused=True),(0.,0.,0.))
    def test_nonfinite_commands_fail_closed(self):
        for value in (math.nan,math.inf,-math.inf):self.assertEqual(self.step((value,0,0)),(0.,0.,0.))
    def test_invalid_speed_cap_rejected(self):
        for value in (math.nan,math.inf,-1.,0.,.31):
            with self.assertRaises(ValueError):self.g.set_speed(value)
    def test_clock_gap_forces_stop(self):self.assertEqual(self.step(dt=.4),(0.,0.,0.))
    def test_velocity_clipping_preserves_turning_curvature(self):
        self.g.set_speed(.30)
        for _ in range(100):v,_,w=self.step((.30,0.,.70))
        self.assertAlmostEqual(w,.35);self.assertAlmostEqual(v,.15)

if __name__=='__main__':unittest.main()
