"""Reception regressions: ordered actions and unspecified route headings."""
import copy
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_mission import Mission, MissionService
from test_reception import fixture, permitted
from audio_feedback import CuePolicy

class ReceptionFollowthroughTests(unittest.TestCase):
    def test_command_epoch_survives_browser_json_number_roundtrip(self):
        with patch('reception_mission.secrets.randbits',side_effect=lambda bits:(1<<bits)-1):
            m=Mission();m.start(fixture(),'tour',0)
        self.assertEqual(m.epoch,int(float(m.epoch)))
        self.assertLess(m.epoch,2**53)

    def mission(self):
        doc=fixture()
        for p in doc['points']:
            if p['id'] in ('b','c'): p.update(script='同一款产品的介绍。', guest_wait=True)
        doc['routes'][0]['loops']=2
        m=Mission(); m.start(doc,'tour',0.)
        return m

    def test_same_intro_at_four_visits_has_distinct_receipts_and_waits_for_continue(self):
        m=self.mission(); identities=set(); stale=None
        for i in range(4):
            cmd=m.tick(permitted(),i*10.); self.assertIsNotNone(cmd)
            m.dispatched(cmd['epoch'],i+1)
            # No speech before the navigation result (which includes heading).
            m.tick(permitted('executing',i+1),i*10.+1); self.assertEqual(m.phase,'navigating')
            m.tick(permitted('arrived',i+1),i*10.+2)
            identity=(m.epoch,m.speech_key)
            self.assertNotIn(identity,identities); identities.add(identity)
            if stale:
                m.tick(permitted('arrived',i+1),i*10.+3,stale)
                self.assertEqual(m.phase,'presenting')
            receipt=dict(epoch=m.epoch,key=m.speech_key,result='completed')
            m.tick(permitted('arrived',i+1),i*10.+4,receipt)
            self.assertEqual(m.phase,'guest_wait')
            m.tick(permitted('arrived',i+1),i*10.+100)
            self.assertEqual(m.phase,'guest_wait')
            m.continue_visit(i*10.+101); stale=receipt
        self.assertEqual(m.phase,'completed')
        self.assertEqual(len([e for e in m.events if e['kind']=='speech_completed']),4)

    def test_stale_matching_content_receipt_does_not_finish_new_presentation(self):
        m=self.mission(); cmd=m.tick(permitted(),0);m.dispatched(cmd['epoch'],1)
        m.tick(permitted('arrived'),1)
        m.tick(permitted('arrived'),2,dict(epoch=m.epoch-1,key=m.speech_key,result='completed'))
        self.assertEqual(m.phase,'presenting')

    def test_intro_then_dwell_then_next_leg(self):
        m=self.mission();p=m.steps[0]['point'];p.update(guest_wait=False,dwell=5.)
        cmd=m.tick(permitted(),0);m.dispatched(cmd['epoch'],1);m.tick(permitted('arrived'),1)
        m.tick(permitted('arrived'),3,dict(epoch=m.epoch,key=m.speech_key,result='completed'))
        self.assertEqual(m.phase,'dwell');self.assertEqual(m.deadline,8.)
        m.tick(permitted('arrived'),7.9);self.assertEqual(m.index,0)
        m.tick(permitted('arrived'),8.);self.assertEqual(m.index,1)
        self.assertEqual(m.phase,'dispatching')

    def service(self,points,start):
        s=MissionService.__new__(MissionService);s.engine=Mission();e=s.engine.start(fixture(),'tour',0)
        s.resume_command=None
        s.node=SimpleNamespace(gait_profile='p2-flat-model',reception_pose=lambda:start,set_goal=Mock(return_value={'generation':7}),confirm_goal=Mock())
        original=copy.deepcopy(points)
        s.dispatch(dict(epoch=e,point=points[-1],points=points,end_index=1))
        self.assertEqual(s.engine.phase,'navigating',s.engine.error)
        self.assertEqual(points,original,'Never rewrite saved route headings')
        return s.node.set_goal.call_args.args[0]

    def test_return_transit_uses_incoming_heading_not_zero_without_terminal_spin(self):
        points=[dict(id='via',x=0.,y=-2.,yaw=None),dict(id='home',x=0.,y=-4.,yaw=None)]
        d=self.service(points,[0.,0.,math.pi/2])
        self.assertEqual(d['heading_mode'],'position')
        for p in d['_route_goals']: self.assertAlmostEqual(p['yaw'],-math.pi/2)
        self.assertAlmostEqual(d['yaw'],d['_route_goals'][-1]['yaw'])

    def test_explicit_arrival_heading_preserved_with_turning_route(self):
        points=[dict(id='via',x=0.,y=2.,yaw=None),dict(id='end',x=-2.,y=2.,yaw=0.)]
        d=self.service(points,[0.,0.,0.])
        self.assertEqual(d['heading_mode'],'pose')
        self.assertAlmostEqual(d['_route_goals'][0]['yaw'],math.pi/2)
        self.assertEqual(d['_route_goals'][1]['yaw'],0.)

    def test_missing_initial_pose_and_duplicate_waypoint_use_next_nonzero_segment(self):
        points=[dict(id='a',x=2.,y=2.,yaw=None),dict(id='b',x=2.,y=2.,yaw=None),dict(id='c',x=0.,y=2.,yaw=None)]
        d=self.service(points,None)
        for p in d['_route_goals']: self.assertAlmostEqual(abs(p['yaw']),math.pi)

    def test_presentation_actions_select_approximate_arrival_without_rewriting_target(self):
        for change in [dict(script='欢迎参观'),dict(dwell=3),dict(guest_wait=True),dict(yaw=.2)]:
            point=dict(id='visit',x=2.,y=-5.,yaw=None,**{k:v for k,v in change.items()if k!='yaw'})
            if 'yaw'in change:point['yaw']=change['yaw']
            d=self.service([point],[0.,0.,0.])
            self.assertEqual(d['_arrival_profile'],'presentation')
            self.assertEqual((d['x'],d['y']),(2.,-5.))

    def test_single_position_goal_keeps_position_only(self):
        d=self.service([dict(id='end',x=-2.,y=0.,yaw=None)],[0.,0.,0.])
        self.assertNotIn('_route_goals',d); self.assertEqual(d['heading_mode'],'position')
        self.assertAlmostEqual(abs(d['yaw']),math.pi)

class PlayingCueTests(unittest.TestCase):
    def test_short_replan_does_not_cut_playing_start_but_cancel_still_does(self):
        p=CuePolicy()
        def data(phase,g=1):return dict(request_token='synthetic',session=dict(phase=phase,generation=g))
        p.observe(data('ready'),0);cue=p.observe(data('executing'),1)
        p.observe(data('replanning'),1.1)
        self.assertFalse(p.valid(cue,data('replanning'),1.2),'Do not queue an old start after a pause')
        self.assertTrue(p.valid(cue,data('replanning'),1.2,max_age=None),'Finish the sentence already playing')
        for state in (data('manual'),data('cancelled'),data('executing',2)):
            self.assertFalse(p.valid(cue,state,1.3,max_age=None))
