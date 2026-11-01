import unittest,math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from motion_buffer import MotionBuffer

class BufferTest(unittest.TestCase):
    def test_forward_steering_correction_does_not_enter_pivot_settling(self):
        b=MotionBuffer(accel=.4,decel=.8);values=[]
        for i in range(250):
            yaw=.24 if i%30<15 else -.24
            measured_yaw=-.12 if yaw>0 else .12
            values.append(b.step((.75,0.,yaw),(.1,0.,measured_yaw),i*.02,.1,frame_verified=True))
            self.assertEqual(b.phase,'forward')
        self.assertAlmostEqual(values[-1][0],.75)
        self.assertTrue(all(y[0]>=x[0] for x,y in zip(values,values[1:])))
        self.assertTrue(all(abs(y[2]-x[2])<=.0060001 for x,y in zip(values,values[1:])))
    def run_steps(self,b,target,velocity,steps=100,start=0.,**options):
        out=[]
        for i in range(steps):
            out.append(b.step(target,velocity,start+i*.01,.01,frame_verified=True,**options))
        return out
    def test_start_and_normal_stop(self):
        b=MotionBuffer();a=self.run_steps(b,(.15,0,0),(0,0,0),151)
        self.assertAlmostEqual(a[-1][0],.15)
        self.assertTrue(all(-1e-12<=y[0]-x[0]<=.001000001 for x,y in zip(a,a[1:])))
        a=self.run_steps(b,(0,0,0),(.15,0,0),102,start=1.51,normal_stop=True)
        self.assertEqual(a[-1],(0,0,0));self.assertEqual(b.phase,'normal_braking')
        self.run_steps(b,(0,0,0),(0,0,0),20,start=2.53,normal_stop=True)
        self.assertEqual(b.phase,'stopped')
    def test_turn_waits_for_measured_translation(self):
        b=MotionBuffer();self.run_steps(b,(.1,0,0),(.1,0,0),100)
        a=self.run_steps(b,(0,0,.2),(.1,0,0),100,start=1.)
        self.assertTrue(all(x[2]==0 for x in a))
        a=self.run_steps(b,(0,0,.2),(0,0,0),100,start=2.)
        self.assertAlmostEqual(a[-1][2],.2);self.assertTrue(all(x[0]==0 for x in a))
        a=self.run_steps(b,(.1,0,0),(0,0,.2),100,start=3.)
        self.assertTrue(all(x[0]==0 for x in a))
        a=self.run_steps(b,(.1,0,0),(0,0,0),100,start=4.)
        self.assertGreater(a[-1][0],0)
    def test_fault_and_unclassified_zero_immediate(self):
        for fault in ('hard','stale','frame','zero'):
            b=MotionBuffer();self.run_steps(b,(.1,0,0),(.1,0,0),100)
            out=b.step((0,0,0) if fault=='zero' else (.1,0,0),(.1,0,0),1.,.501 if fault=='stale' else .01,
                frame_verified=fault!='frame',hard_stop=fault=='hard')
            self.assertEqual(out,(0,0,0));self.assertEqual(b.phase,'immediate_zero')
    def test_pivot_rechecks_translation_after_turn_has_started(self):
        b=MotionBuffer()
        self.run_steps(b,(0,0,.2),(0,0,0),101)
        self.assertEqual(b.phase,'pivot');self.assertAlmostEqual(b.w,.2)
        # A body can translate again after the initial settling interval.
        # A zero forward command is not evidence that translation stayed zero.
        out=b.step((0,0,.2),(.12,0,0),1.01,.01,frame_verified=True)
        self.assertEqual(out,(0,0,0));self.assertEqual(b.phase,'braking_translation')
        held=self.run_steps(b,(0,0,.2),(0,.10,0),30,start=1.02)
        self.assertTrue(all(x==(0,0,0) for x in held))
        settling=self.run_steps(b,(0,0,.2),(0,0,0),15,start=1.32)
        self.assertTrue(all(x==(0,0,0) for x in settling))
        resumed=self.run_steps(b,(0,0,.2),(0,0,0),70,start=1.47)
        self.assertGreater(resumed[-1][2],0)
        self.assertTrue(all(0<=x[2]<=.2 for x in resumed))
    def test_ten_hz_original_feedback_does_not_repeatedly_reset_start_ramp(self):
        b=MotionBuffer();outputs=[]
        # 10Hz feedback with92ms processing age, sampled by the50Hz buffer.
        # Every sample remains within the unchanged500ms sensor contract.
        for i in range(100):
            outputs.append(b.step((.075,0.,0.),(0.,0.,0.),i*.02,.092+(i%5)*.02,frame_verified=True))
        self.assertAlmostEqual(outputs[-1][0],.075)
        self.assertTrue(all(0<=v[0]<=.075 for v in outputs))
        self.assertTrue(all(b[0]>=a[0] for a,b in zip(outputs,outputs[1:])))
        self.assertEqual(b.step((.075,0,0),(0,0,0),2.,.501,frame_verified=True),(0,0,0))
        with self.assertRaises(ValueError):MotionBuffer(feedback_timeout=.501)

    def test_no_sideways_reverse_or_excess_request(self):
        for target in ((.801,0,0),(.1,.01,0),(-.01,0,0),(0,0,1.001),(float('nan'),0,0)):
            self.assertEqual(MotionBuffer().step(target,(0,0,0),0.,.01,frame_verified=True),(0,0,0))

if __name__=='__main__':unittest.main()
