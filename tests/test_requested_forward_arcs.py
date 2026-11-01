"""Moving bends retain translation through yaw reversals and honor real stops."""
import math,pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'navigation'))
from configure import config
from motion_buffer import MotionBuffer
from gate import ForwardGate,Limits

class RequestedForwardArcTests(unittest.TestCase):
    def test_bends_do_not_reenter_pivot_or_settling_phase(self):
        b=MotionBuffer(accel=.4,decel=.8,angular_accel=.5)
        g=ForwardGate(Limits(max_v=.8,max_w=1.,accel=.4,decel=.8));g.set_speed(.75);g.armed=True
        outputs=[]
        for i in range(600):
            w=.8 if i<200 else -.8 if i<400 else .5
            buffered=b.step((.6,0.,w),(.2,0.,.15 if i%2 else -.15),1.+i*.02,.05,frame_verified=True)
            actual=g.step(buffered,.02,.01,.05,.05)
            if i>150:
                self.assertEqual(b.phase,'forward');self.assertGreater(actual[0],.59)
            self.assertEqual(actual[1],0.);self.assertLessEqual(abs(actual[2]),1.)
            outputs.append(actual)
        self.assertGreater(outputs[199][2],.79);self.assertLess(outputs[399][2],-.79)
        self.assertEqual(b.step((.6,0.,.5),(0.,0.,0.),13.,.05,frame_verified=True,hard_stop=True),(0.,0.,0.))
        self.assertEqual(g.step((0.,0.,0.),.02,.01,.05,.05),(0.,0.,0.))

    def test_requested_profile_retains_other_control_semantics(self):
        p=config(gait_profile='p2-flat-model');r=p['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(r['rotate_to_heading_angular_vel'],1.)
        self.assertGreater(r['rotate_to_heading_min_angle'],math.radians(65))
        self.assertEqual(r['regulated_linear_scaling_min_speed'],.6)
        self.assertTrue(r['use_collision_detection']);self.assertFalse(r['allow_reversing'])
        self.assertEqual(p['velocity_smoother']['ros__parameters']['max_velocity'],[.8,0.,1.])
        self.assertEqual(p['velocity_smoother']['ros__parameters']['min_velocity'],[0.,0.,-1.])

if __name__=='__main__':unittest.main()
