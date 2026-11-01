import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from scan_ranges import encode_ranges

class ScanRangeTests(unittest.TestCase):
 def test_observed_far_ray_is_clearing_but_empty_height_bin_stays_unknown(self):
  source=np.array([np.inf,np.nan,25.,17.,14.,2.4,.1])
  actual=encode_ranges(source)
  self.assertTrue(np.isnan(actual[0]) and np.isnan(actual[1]))
  self.assertTrue(np.isposinf(actual[2]) and np.isposinf(actual[3]))
  self.assertEqual(actual[4:6].tolist(),[14.,2.4]);self.assertTrue(np.isnan(actual[6]))
  self.assertEqual(source[2],25.)
 def test_finite_returns_always_respect_declared_bounds(self):
  actual=encode_ranges(np.linspace(0,30,301));finite=actual[np.isfinite(actual)]
  self.assertTrue(((finite>=.12)&(finite<=14.)).all())

if __name__=='__main__':unittest.main()
