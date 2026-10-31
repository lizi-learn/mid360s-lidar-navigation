import math
import unittest
from navigation.model_response import ModelResponseTracker


def profile():
    return {'scope':'isolated_model', 'forward':[[0,0],[.12,.25],[.20,.30]],
            'yaw':[[0,0],[.10,.35],[.30,.50]]}


class ModelResponseTests(unittest.TestCase):
    def test_nonzero_target_can_cross_model_deadband_without_changing_target(self):
        c=ModelResponseTracker(profile());target=(.15,0.)
        for _ in range(200):out=c.step(target,(0.,0.),.02,0.)
        self.assertGreater(out[0],.25)
        self.assertLessEqual(out[0],.30)
        self.assertEqual(target,(.15,0.))
        self.assertEqual(out[1],0.)

    def test_stop_and_source_expiry_immediately_clear_compensation(self):
        for stop in ('zero','expired','invalid'):
            c=ModelResponseTracker(profile())
            for _ in range(100):c.step((.15,.2),(0.,0.),.02,0.)
            value=c.step((0.,0.) if stop=='zero' else (.15,.2),(0.,0.),.02,.151 if stop=='expired' else (math.nan if stop=='invalid' else 0.))
            self.assertEqual(value,(0.,0.))
            self.assertEqual(c.integral,[0.,0.])

    def test_navigation_target_limits_not_relaxed_to_policy_input_range(self):
        for target in [(.15001,0),(-.01,0),(0,.24001)]:
            self.assertEqual(ModelResponseTracker(profile()).step(target,(0,0),.02,0),(0.,0.))

    def test_internal_start_slew_and_saturation(self):
        c=ModelResponseTracker(profile());old=(0.,0.)
        for _ in range(500):
            out=c.step((.15,.24),(0,0),.02,0)
            self.assertLessEqual(abs(out[0]-old[0]),.003000001)
            self.assertLessEqual(abs(out[1]-old[1]),.006000001)
            self.assertLessEqual(out[0],.30)
            self.assertLessEqual(abs(out[1]),.50)
            old=out

    def test_zero_axis_is_not_replaced_by_minimum_motion(self):
        c=ModelResponseTracker(profile())
        for _ in range(100):c.step((.15,.2),(0.,0.),.02,0.)
        self.assertEqual(c.step((0.,.2),(0.,0.),.02,0.)[0],0.)
        self.assertEqual(c.step((.15,0.),(0.,0.),.02,0.)[1],0.)

    def test_reversed_yaw_passes_through_zero(self):
        c=ModelResponseTracker(profile())
        for _ in range(100):c.step((0.,.2),(0.,0.),.02,0.)
        self.assertEqual(c.step((0.,-.2),(0.,0.),.02,0.),(0.,0.))
        self.assertLess(c.step((0.,-.2),(0.,0.),.02,0.)[1],0.)

    def test_explicit_nonzero_model_deadband_does_not_lift_zero(self):
        c=ModelResponseTracker({**profile(),'nonzero_policy_floor':[0.,.20]})
        for _ in range(100):out=c.step((0.,.001),(0.,0.),.02,0.)
        self.assertGreaterEqual(out[1],.20)
        self.assertEqual(c.step((0.,0.),(0.,0.),.02,0.),(0.,0.))
        for floor in ([0.,.36],[-.1,0.],[0.,math.nan],[0.]):
            with self.assertRaises(ValueError):ModelResponseTracker({**profile(),'nonzero_policy_floor':floor})

    def test_startup_slew_does_not_wind_up_velocity_integrator(self):
        c=ModelResponseTracker(profile())
        for _ in range(50):c.step((.15,.24),(0.,0.),.02,0.)
        self.assertEqual(c.integral,[0.,0.])

    def test_explicit_model_only_monotone_profile_required(self):
        for changes in [{'scope':'hardware'},{'forward':[[0,0],[.1,.2],[.09,.3]]},
                        {'yaw':[[0,0],[.2,.4]]},{'yaw':[[0,0],[.3,math.nan]]}]:
            with self.assertRaises(ValueError):ModelResponseTracker({**profile(),**changes})


if __name__=='__main__':unittest.main()
