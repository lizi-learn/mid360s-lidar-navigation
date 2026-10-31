from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation'))
from terrain import observed_front_support
from terrain_history import SupportHistory


def patches(columns):
    return np.array([(x + dx, y, 0.) for x in (.7, 1.05)
                     for y in columns for dx in (-.01, .01)])


class SupportHistoryTests(unittest.TestCase):
    def update(self, history, points, stamp, now=None, center=(0., 0.), yaw=0., plane=(0., 0., 0.)):
        return history.update(points, np.array(center), yaw, plane, stamp, stamp if now is None else now)

    def test_sparse_frames_complement_each_other_without_new_points(self):
        h = SupportHistory()
        left, right = patches([-.25]), patches([0., .25])
        self.assertFalse(observed_front_support(left, [0, 0], 0, [0, 0, 0]))
        self.assertFalse(observed_front_support(right, [0, 0], 0, [0, 0, 0]))
        self.assertFalse(self.update(h, left, 10.)[0])
        ready, report = self.update(h, right, 10.1)
        self.assertTrue(ready)
        self.assertEqual(report['support_oldest_source_stamp_s'], 10.)
        self.assertEqual(report['support_window_frames'], 2)
        self.assertEqual(report['support_patch_counts'], [2] * 6)

    def test_missing_patch_is_never_interpolated(self):
        h = SupportHistory()
        for stamp in (10., 10.1, 10.2):
            self.assertFalse(self.update(h, patches([-.25, .25]), stamp)[0])

    def test_old_source_evidence_expires_even_when_received_now(self):
        h = SupportHistory()
        self.update(h, patches([-.25]), 10.)
        self.assertFalse(self.update(h, patches([0., .25]), 10.2, now=10.3)[0])
        self.assertEqual(h.frames[0][0], 10.2)
        ready, report = self.update(h, patches([-.25]), 10.4, now=10.7)
        self.assertFalse(ready)
        self.assertIsNone(report['support_oldest_source_stamp_s'])

    def test_points_are_not_carried_with_robot_motion_or_heading(self):
        for center, yaw in [((2., 0.), 0.), ((0., 0.), np.pi / 2)]:
            with self.subTest(center=center, yaw=yaw):
                h = SupportHistory()
                self.assertTrue(self.update(h, patches([-.25, 0., .25]), 10.)[0])
                self.assertFalse(self.update(h, np.empty((0, 3)), 10.1, center=center, yaw=yaw)[0])

    def test_current_lower_surface_immediately_rejects_old_floor(self):
        h = SupportHistory()
        self.assertTrue(self.update(h, patches([-.25, 0., .25]), 10.)[0])
        ready, report = self.update(h, np.array([[.7, 0., -.2]]), 10.1)
        self.assertFalse(ready)
        self.assertTrue(any(report['support_patch_lower_returns']))
        self.assertEqual(len(h.frames), 0)
        self.assertFalse(self.update(h, patches([-.25, .25]), 10.2)[0])

    def test_changed_plane_reclassifies_historical_points(self):
        h = SupportHistory()
        self.update(h, patches([-.25, 0., .25]), 10.)
        self.assertFalse(self.update(h, np.empty((0, 3)), 10.1, plane=(0., 0., .2))[0])

    def test_duplicate_backward_stale_or_future_samples_do_not_vote(self):
        for stamp, now in [(10., 10.1), (9.9, 10.), (10.1, 10.7), (11., 10.)]:
            with self.subTest(stamp=stamp, now=now):
                h = SupportHistory()
                self.update(h, patches([-.25]), 10.)
                with self.assertRaises(ValueError):
                    self.update(h, patches([0., .25]), stamp, now)
                self.assertEqual(len(h.frames), 0)


if __name__ == '__main__':
    unittest.main()
