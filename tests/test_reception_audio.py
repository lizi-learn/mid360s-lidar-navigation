import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from audio_feedback import CuePolicy

class ReceptionCueTests(unittest.TestCase):
    def data(self,phase,g=1,mission='navigating',run='one'):
        return dict(request_token='synthetic',session=dict(phase=phase,generation=g),reception=dict(mission=dict(run_id=run,phase=mission)))
    def test_one_start_per_tour_and_internal_reroute_is_quiet(self):
        p=CuePolicy();p.observe(self.data('ready',mission='dispatching'),0)
        self.assertEqual(p.observe(self.data('executing'),1).kind,'started')
        self.assertIsNone(p.observe(self.data('cancelled',mission='rerouting'),2))
        self.assertIsNone(p.observe(self.data('executing',2),3))
        p.observe(self.data('arrived',2,mission='completed'),4)
        p.observe(self.data('ready',3,mission='dispatching',run='two'),5)
        self.assertEqual(p.observe(self.data('executing',3,run='two'),6).kind,'started')
    def test_waiting_route_is_debounced_and_takeover_invalidates(self):
        p=CuePolicy();p.observe(self.data('executing'),0)
        self.assertIsNone(p.observe(self.data('cancelled',mission='waiting_route'),1))
        cue=p.observe(self.data('cancelled',mission='waiting_route'),2.6)
        self.assertEqual(cue.kind,'obstacle');self.assertTrue(p.valid(cue,self.data('cancelled',mission='waiting_route'),2.6))
        self.assertFalse(p.valid(cue,self.data('manual',mission='manual'),2.7))
    def test_single_goal_without_reception_keeps_old_cues(self):
        p=CuePolicy();a=dict(request_token='plain',session=dict(phase='ready',generation=1))
        p.observe(a,0);a['session']['phase']='executing'
        self.assertEqual(p.observe(a,1).kind,'started')
if __name__=='__main__':unittest.main()
