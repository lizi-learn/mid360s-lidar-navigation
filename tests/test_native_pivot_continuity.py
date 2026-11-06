"""Native phase ownership: stepping sway must not re-enter translation braking."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from motion_buffer import MotionBuffer

class NativePivotTests(unittest.TestCase):
    def native(self):
        return MotionBuffer(ramp_enabled=False,recheck_pivot_translation=False)
    def begin(self,b):
        for i in range(20):out=b.step((0.,0.,1.),(0.,0.,0.),1.+i*.02,.1,frame_verified=True)
        self.assertEqual(out,(0.,0.,1.));return 1.4
    def test_active_pivot_keeps_turn_request_across_large_stepping_sway(self):
        b=self.native();t=self.begin(b)
        for i in range(300):
            # Both low-frequency body sway and step-rate velocity ripple occur
            # in the field trace; neither requests a phase change.
            v=(.16*math.sin(i*.09),.24*math.cos(i*.13),.6)
            self.assertEqual(b.step((0.,0.,1.),v,t+i*.02,.1,frame_verified=True),(0.,0.,1.))
            self.assertEqual(b.phase,'pivot')
    def test_forward_to_pivot_still_waits_for_initial_translation_settling(self):
        b=self.native();b.step((.75,0.,0.),(.2,0.,0.),1.,.1,frame_verified=True)
        for i in range(1,30):
            self.assertEqual(b.step((0.,0.,1.),(.2,0.,0.),1.+i*.02,.1,frame_verified=True),(0.,0.,0.))
        for i in range(30,50):out=b.step((0.,0.,1.),(0.,0.,0.),1.+i*.02,.1,frame_verified=True)
        self.assertEqual(out,(0.,0.,1.))
    def test_active_pivot_does_not_override_collision_source_or_cancel_stops(self):
        for options,target in [({'hard_stop':True},(0,0,1)),({'feedback_age':.501},(0,0,1)),({'frame_verified':False},(0,0,1)),({},(0,0,0))]:
            b=self.native();t=self.begin(b);kw=dict(feedback_age=.1,frame_verified=True);kw.update(options)
            self.assertEqual(b.step(target,(.1,.1,.4),t,**kw),(0.,0.,0.))
    def test_opposite_turn_still_settles_before_reversing(self):
        b=self.native();t=self.begin(b)
        self.assertEqual(b.step((0,0,-1),(0,0,.5),t,.1,frame_verified=True),(0,0,0))

if __name__=='__main__':unittest.main()
