"""Bound input derivatives under late retargeting and normal braking."""
from pathlib import Path
import shutil,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class InputCurveTests(unittest.TestCase):
 @unittest.skipUnless(shutil.which('g++'),'C++ compiler required')
 def test_braking_retargeting_and_varied_tick_durations(self):
  with tempfile.TemporaryDirectory()as d:
   exe=Path(d)/'check'
   subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all','-I'+str(ROOT/'navigation'),str(ROOT/'experiments/smooth_input/check.cpp'),'-o',str(exe)],check=True,capture_output=True)
   subprocess.run([str(exe)],check=True,capture_output=True)
