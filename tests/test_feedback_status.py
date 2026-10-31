import json
import math
import tempfile
from pathlib import Path
import unittest
from navigation.feedback_status import read_neck_feedback


class FeedbackStatusTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'feedback.json'
        self.identity = {'name': 'synthetic', 'installation_id': 'new-install'}
        self.data = dict(schema_version=1, robot_identity=self.identity, boot_id='boot-a',
                         received_monotonic_ns=1_000_000_000, valid=True,
                         position_counts=123456, fixture_relative_deg=6.0,
                         statusword=848, mode_display=8, error_code=0,
                         working_counter=12, expected_working_counter=12)

    def read(self, **changes):
        self.path.write_text(json.dumps({**self.data, **changes}))
        return read_neck_feedback(self.path, self.identity, now_ns=1_100_000_000, boot_id='boot-a')

    def test_valid_feedback_keeps_relative_anchor_separate_from_physical_zero(self):
        d = self.read(physical_zero_verified=True, kinematics_ready=True)
        self.assertTrue(d['valid'])
        self.assertEqual(d['position_counts'], 123456)
        self.assertAlmostEqual(d['age_s'], .1)
        self.assertFalse(d['physical_zero_verified'])
        self.assertFalse(d['kinematics_ready'])

    def test_previous_installation_or_reboot_rejected(self):
        self.assertFalse(self.read(robot_identity={'name':'other'})['valid'])
        self.assertFalse(self.read(boot_id='old-boot')['valid'])

    def test_stale_and_future_not_refreshed_on_read(self):
        for stamp in [700_000_000, 1_100_000_001]:
            self.assertFalse(self.read(received_monotonic_ns=stamp)['valid'])

    def test_invalid_source_and_bus_response_rejected(self):
        for changes in [{'valid':False}, {'error_code':1}, {'working_counter':11}]:
            self.assertFalse(self.read(**changes)['valid'])

    def test_bad_values_rejected(self):
        for changes in [{'fixture_relative_deg':math.nan}, {'fixture_relative_deg':math.inf},
                        {'position_counts':1.5}, {'position_counts':True}]:
            self.assertFalse(self.read(**changes)['valid'])

    def test_missing_or_corrupt_never_breaks_status_endpoint(self):
        for content in ['', '{', 'null', '[]']:
            self.path.write_text(content)
            self.assertFalse(read_neck_feedback(self.path,self.identity)['valid'])
        self.path.unlink()
        self.assertFalse(read_neck_feedback(self.path,self.identity)['valid'])


if __name__ == '__main__': unittest.main()
