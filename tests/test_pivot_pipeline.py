"""Synthetic pivot chain using the real response helpers; no hardware I/O."""
import collections
import ctypes
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'navigation'))
from motion_buffer import MotionBuffer, MeasuredMotionMean
from configure import config

BRIDGE = r'''
#include <settled_pivot.hpp>
#include <speed_response.hpp>
p2_nav_controller::SettledPivot pivot;
p2_nav::SpeedResponse native;
extern "C" {
void reset(){pivot.reset();native.reset();}
double command(double error,double yaw,double rate,double t){return pivot.step(error,yaw,rate,t);}
double input(double target,double rate,double t){auto stamp=static_cast<std::uint64_t>((1.+t)*1e9);return native.step(0.,target,0.,rate,.02,stamp,stamp,true).policy_yaw;}
int pending(){return pivot.pending();}
}
'''

class PivotPipelineTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'C++ compiler required')
    def test_ripple_deadzone_and_transport_with_native_input_ramp(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder)
            (p / 'bridge.cpp').write_text(BRIDGE)
            subprocess.run(['g++', '-std=c++17', '-O2', '-shared', '-fPIC',
                            '-Wl,-Bsymbolic', '-I'+str(ROOT/'navigation'),
                            '-I'+str(ROOT/'navigation/p2_nav_controller/include/p2_nav_controller'),
                            str(p/'bridge.cpp'), '-o', str(p/'bridge.so')],
                           check=True, capture_output=True)
            native = ctypes.CDLL(str(p/'bridge.so'))
            native.command.argtypes = [ctypes.c_double]*4
            native.command.restype = ctypes.c_double
            native.input.argtypes = [ctypes.c_double]*3
            native.input.restype = ctypes.c_double
            slope = config(gait_profile='p2-flat-model')['velocity_smoother']['ros__parameters']['max_accel'][2]
            for goal in [-.7, .7]:
                for delay in [.3, .8, 1.3]:
                    native.reset()
                    buffer = MotionBuffer(accel=.4, decel=.8, angular_accel=20.)
                    mean = MeasuredMotionMean()
                    queue = collections.deque([0.]*(round(delay/.02)+1))
                    yaw = rate = smooth = quiet = 0.
                    reached = False
                    for k in range(4000):
                        t = k*.02
                        observed = rate + (.08 if k % 50 == 0 else 0.)
                        error = math.remainder(goal-yaw, 2*math.pi)
                        raw = native.command(error, yaw, observed, t)
                        measured = mean.step((0., 0., observed), 1.+t)
                        requested = buffer.step((0., 0., raw), measured, t, .02, frame_verified=True)[2]
                        # Numerical model of the configured OPEN_LOOP smoother.
                        # The actual ROS smoother and collision node are not
                        # instantiated in this unit-level synthetic plant.
                        delta = requested-smooth
                        smooth += math.copysign(min(abs(delta), slope*.02), delta)
                        input_value = native.input(smooth, observed, t)
                        self.assertLessEqual(abs(input_value), 1.)
                        queue.append(input_value)
                        applied = queue.popleft()
                        target = 0. if abs(applied)<.55 else .8*applied
                        rate += (target-rate)*(1-math.exp(-.02/.15))
                        yaw = math.remainder(yaw+rate*.02, 2*math.pi)
                        if abs(math.remainder(goal-yaw, 2*math.pi))<.15 and abs(rate)<.03 and raw==0 and not native.pending():
                            quiet += .02
                        else:
                            quiet = 0.
                        if quiet >= 2.:
                            reached = True
                            break
                    self.assertTrue(reached, (goal, delay, yaw))
                    self.assertEqual(buffer.step((0.,0.,0.), measured, t+.02, .02, hard_stop=True, frame_verified=True), (0.,0.,0.))
                    self.assertEqual(native.input(0., observed, t+.02), 0.)
