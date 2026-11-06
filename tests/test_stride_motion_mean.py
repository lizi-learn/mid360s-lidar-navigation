import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from motion_buffer import MeasuredMotionMean,MotionBuffer

class StrideMotionTests(unittest.TestCase):
    def replay(self,window,hz=2.,drift=0.,fault=False):
        mean=MeasuredMotionMean(translation_window=window);buffer=MotionBuffer(ramp_enabled=False)
        values=[];source=0.;measured=(0.,0.,0.)
        for i in range(600):
            t=1.+i*.02
            if i%5==0:
                source=t;raw=(drift,.2*math.sin(2*math.pi*hz*(t-1.)),-.25)
                measured=mean.step(raw,source)
            out=buffer.step((0.,0.,-1.),measured,t,.6 if fault else t-source,
                            frame_verified=True)
            if i>=100:values.append(out)
        return values
    def test_zero_net_lateral_steps_do_not_repeatedly_withdraw_an_active_turn(self):
        old=self.replay(0.)
        self.assertGreater(sum(v==(0.,0.,0.)for v in old),50)
        for hz in [1.5,1.75,2.,2.25,2.5]:
            new=self.replay(.5,hz)
            self.assertTrue(all(v==(0.,0.,-1.)for v in new),hz)
    def test_real_sustained_translation_and_stale_source_still_withdraw_yaw(self):
        self.assertTrue(all(v==(0.,0.,0.)for v in self.replay(.5,drift=.12)))
        self.assertTrue(all(v==(0.,0.,0.)for v in self.replay(.5,fault=True)))
    def test_original_time_and_fast_yaw_response_are_preserved(self):
        mean=MeasuredMotionMean(translation_window=.5);plain=MeasuredMotionMean()
        for i in range(20):
            v=(.12,-.1,(-1 if i<10 else 1)*.3);t=1+i*.1
            a=mean.step(v,t);b=plain.step(v,t)
            self.assertAlmostEqual(a[0],.12);self.assertAlmostEqual(a[1],-.1);self.assertEqual(a[2],b[2])
        before=mean.step((1.,1.,1.),t)
        self.assertEqual(before,a);self.assertEqual(mean.source,t)
        self.assertEqual(mean.step((.2,0.,0.),t+.6),(.2,0.,0.))
    def test_irregular_source_intervals_are_time_weighted(self):
        mean=MeasuredMotionMean(translation_window=.5)
        # v(t)=t has exact trapezoidal mean .35 over [.1,.6]. Sample count
        # weighting would bias the result toward the crowded early samples.
        for t in [0.,.03,.05,.1,.4,.6]:value=mean.step((t,-t,.0),t)
        self.assertAlmostEqual(value[0],.35);self.assertAlmostEqual(value[1],-.35)

if __name__=='__main__':unittest.main()
