import unittest,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from gait_handoff import GaitHandoff
from gate import ForwardGate
class HandoffTests(unittest.TestCase):
 def test_turn_waits_for_translation_then_forward_waits_for_yaw(self):
  h=GaitHandoff();c=(.15,0.,.2);turn=(0.,0.,.2)
  for t in [0.,.05,.1]:self.assertEqual(h.step(c,turn,(.1,0.,0.),t,0.,0.),(0.,0.,0.))
  for t in [.15,.2,.25]:self.assertEqual(h.step(c,turn,(.02,0.,0.),t,0.,0.),(0.,0.,0.))
  self.assertEqual(h.step(c,turn,(.02,0.,0.),.31,0.,0.),(0.,0.,.2))
  forward=(.15,0.,.02)
  self.assertEqual(h.step(forward,forward,(0.,0.,.15),.36,0.,0.),(0.,0.,0.))
  for t in [.41,.46,.51]:self.assertEqual(h.step(forward,forward,(0.,0.,.02),t,0.,0.),(0.,0.,0.))
  self.assertEqual(h.step(forward,forward,(0.,0.,.02),.57,0.,0.),forward)
 def test_new_stop_intent_clears_a_still_smoothed_forward_command(self):
  h=GaitHandoff();self.assertEqual(h.step((.15,0.,.1),(0.,0.,0.),(.1,0.,.1),0.,0.,0.),(0.,0.,0.))
 def test_restart_after_stop_waits_for_remaining_body_rotation(self):
  h=GaitHandoff();h.step((.1,0.,.2),(0.,0.,0.),(0.,0.,.2),0.,0.,0.)
  self.assertEqual(h.step((.1,0.,0.),(.1,0.,0.),(0.,0.,.2),.05,0.,0.),(0.,0.,0.))
 def test_collision_zero_is_preserved_during_pivot(self):
  h=GaitHandoff()
  for t in [0.,.1,.2]:self.assertEqual(h.step((0.,0.,0.),(0.,0.,.2),(0.,0.,0.),t,0.,0.),(0.,0.,0.))
 def test_bad_feedback_or_stale_intent_cannot_release_motion(self):
  for age,velocity in [(.4,(0.,0.,0.)),(0.,(float('nan'),0.,0.))]:
   h=GaitHandoff();self.assertEqual(h.step((.1,0.,0.),(.1,0.,0.),velocity,0.,age,0.),(0.,0.,0.));self.assertEqual(h.phase,'feedback_unavailable')
 def test_final_ramp_is_still_applied_after_handoff(self):
  h=GaitHandoff();g=ForwardGate();g.armed=True
  for t in [0.,.05,.1,.15,.2]:
   command=h.step((0.,0.,.24),(0.,0.,.24),(0.,0.,0.),t,0.,0.);v=g.step(command,.05,0.,0.,0.)
   self.assertLessEqual(abs(v[2]),.5*.05*max(0,int(round(t/.05))-2)+1e-9)
if __name__=='__main__':unittest.main()
