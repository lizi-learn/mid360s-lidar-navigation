import unittest,math
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from motion_buffer import MotionBuffer

class BufferTest(unittest.TestCase):
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
            out=b.step((0,0,0) if fault=='zero' else (.1,0,0),(.1,0,0),1.,.2 if fault=='stale' else .01,
                frame_verified=fault!='frame',hard_stop=fault=='hard')
            self.assertEqual(out,(0,0,0));self.assertEqual(b.phase,'immediate_zero')
    def test_no_sideways_reverse_or_excess_request(self):
        for target in ((.151,0,0),(.1,.01,0),(-.01,0,0),(0,0,.241),(float('nan'),0,0)):
            self.assertEqual(MotionBuffer().step(target,(0,0,0),0.,.01,frame_verified=True),(0,0,0))

if __name__=='__main__':unittest.main()
