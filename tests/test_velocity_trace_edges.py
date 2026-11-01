import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation'))
from velocity_trace import VelocityTrace


class TraceTests(unittest.TestCase):
    def test_first_fault_is_not_hidden_by_the_periodic_sample_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'receipts.jsonl'
            trace = VelocityTrace(path)
            sample = {'phase': 'active', 'native_reason': 'navigation_active',
                      'readiness': 495, 'policy_generation': 3,
                      'armed': True, 'navigation_owned': True,
                      'health_source_ns': 100, 'target_source_ns': 100}
            try:
                with patch('velocity_trace.time.monotonic', return_value=20.):
                    trace.submit(sample)
                    trace.submit(sample)  # Same periodic sample is suppressed.
                    trace.submit({**sample, 'native_reason': 'navigation_health_not_ready',
                                  'armed': False, 'navigation_owned': False})
                    trace.submit({**sample, 'readiness': 489, 'policy_generation': 4,
                                  'native_reason': 'policy_not_ready'})
                trace.queue.join()
            finally:
                trace.close()
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(records), 3)
            self.assertEqual([r['native_reason'] for r in records],
                             ['navigation_active', 'navigation_health_not_ready', 'policy_not_ready'])
            self.assertTrue(all(r['health_source_ns'] == 100 and r['target_source_ns'] == 100 for r in records))
            self.assertTrue(all(r['trace_wall_time_ns'] > 0 for r in records))
            self.assertEqual(trace.dropped, 0)
            self.assertEqual(trace.error, '')


if __name__ == '__main__': unittest.main()
