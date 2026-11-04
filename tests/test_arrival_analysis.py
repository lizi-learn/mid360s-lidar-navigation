import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'navigation'))
spec=importlib.util.spec_from_file_location('arrival_analysis',ROOT/'experiments/arrival/check.py')
analysis=importlib.util.module_from_spec(spec);spec.loader.exec_module(analysis)

class ArrivalAnalysisTests(unittest.TestCase):
 def test_stationary_body_is_checked_with_one_pose(self):
  grid=np.zeros((40,40),dtype=int);pose=[1.,1.,0.]
  self.assertEqual(analysis.intrusions(grid,.05,(0.,0.),[pose]),0)
  grid[20,20]=254
  self.assertGreater(analysis.intrusions(grid,.05,(0.,0.),[pose]),0)
  grid[20,20]=255
  self.assertGreater(analysis.intrusions(grid,.05,(0.,0.),[pose]),0)
  self.assertGreater(analysis.intrusions(grid,.05,(0.,0.),[]),0)
