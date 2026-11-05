"""Actual mission and Console branches, with controlled native action receipts."""
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import test_position_goals as position_tests
from reception_catalog import blank
from reception_mission import Mission, MissionService
from reception_spins import initial_error,turn_initial


class NativeSpinConsoleTests(unittest.TestCase):
    setup_console=position_tests.PositionGoalTests.setup_console
    def test_directed_preview_retained_but_translation_does_not_pulse_final_yaw(self):
        c=self.setup_console()
        r=c.set_goal(dict(x=1.,y=1.,yaw=1.,heading_mode='pose',
                         _arrival_profile='presentation',_native_arrival_spin=True))
        self.assertEqual(c.planner.sent[0].planner_id,'GridBased')
        c.confirm_goal({'generation':r['generation']})
        import xml.etree.ElementTree as ET
        tree=ET.parse(c.nav.sent[0].behavior_tree)
        self.assertEqual(tree.find('.//ComputePathToPose').get('planner_id'),'GridBased')
        self.assertEqual(tree.find('.//FollowPath').get('goal_checker_id'),'presentation_position_goal_checker')


class ReceptionNativeSpinTests(unittest.TestCase):
    def fixture(self,error=.9):
        p=dict(id='product',name='Product',x=3.,y=0.,yaw=.9,kind='visit',
               product_id='',script='Product introduction.',dwell=0.,guest_wait=False)
        doc=blank('synthetic');doc['points']=[p]
        doc['routes']=[dict(id='route',name='route',mode='showcase',loops=1,
                           description='',steps=[dict(point_id=p['id'],speak=True)])]
        engine=Mission();epoch=engine.start(doc,'route',0.)
        engine.dispatched(epoch,7,0,[]);engine.pending_arrival_heading=True
        state=dict(phase='arrived',generation=7,navigation_permitted=True,
                   operator_mode='navigation_allowed',manual_activity=False)
        pose=[3.,0.,p['yaw']-error];session=NS(generation=7,replan_timeout=25.)
        node=NS(gait_profile='p2-flat-model',native_reception_spins=True,
                presentation_yaw_tolerance=math.radians(10),reception_pose=lambda:tuple(pose),
                session_state=lambda:dict(state),session=session,
                health_ready=lambda:True,execution_ready=lambda:True)
        def cancel(epoch):state['phase']='cancelled';return engine.valid(epoch)
        def prepare(data):
            session.generation+=1;state.update(phase='ready',generation=session.generation)
            return dict(generation=session.generation)
        def confirm(data):state['phase']='executing'
        node.cancel_reception_leg=Mock(side_effect=cancel)
        node.prepare_rotation=Mock(side_effect=prepare);node.confirm_goal=Mock(side_effect=confirm)
        service=MissionService.__new__(MissionService);service.engine=engine;service.node=node
        service.future=None;service.source_recovery_active=False;service.pending_source_command=None
        service.resume_command=None;service.tracking_turn_leg=None
        service.active_command=dict(epoch=epoch,point=p,points=[p],end_index=0,native_arrival_spin=True)
        return service,engine,state,pose

    def test_position_success_does_not_start_introduction_before_heading_receipt(self):
        s,e,state,pose=self.fixture()
        self.assertIsNone(e.tick(state,1.))
        self.assertEqual(e.phase,'navigating')
        self.assertFalse(any(x['kind']=='arrived'for x in e.events))
        command=s.arrival_heading_command(state);s.dispatch(command)
        s.node.prepare_rotation.assert_called_once_with({'radians':.9})
        self.assertEqual(e.goal_generation,8);self.assertTrue(e.pending_arrival_heading)
        self.assertIsNone(e.tick(state,2.));self.assertEqual(e.phase,'navigating')
        state['phase']='arrived';pose[2]=.85
        self.assertIsNone(s.arrival_heading_command(state));e.tick(state,3.)
        self.assertEqual(e.phase,'presenting')
        self.assertEqual(sum(x['kind']=='arrived'for x in e.events),1)
        self.assertEqual(sum(x['kind']=='presentation'for x in e.events),1)

    def test_ten_degree_window_does_not_request_fine_adjustment(self):
        for error in (-math.radians(9.99),math.radians(9.99)):
            s,e,state,_=self.fixture(error)
            self.assertIsNone(s.arrival_heading_command(state))
            e.tick(state,1.);self.assertEqual(e.phase,'presenting')
            s.node.prepare_rotation.assert_not_called()
        s,e,state,_=self.fixture(math.radians(10.01))
        self.assertIsNotNone(s.arrival_heading_command(state))

    def test_right_then_left_before_native_acceptance_never_replays_spin(self):
        s,e,state,_=self.fixture();command=s.arrival_heading_command(state)
        original=s.node.prepare_rotation.side_effect
        def takeover(data):
            result=original(data);state.update(navigation_permitted=False,operator_mode='manual')
            return result
        s.node.prepare_rotation.side_effect=takeover;s.dispatch(command)
        self.assertEqual(e.phase,'manual');s.node.confirm_goal.assert_not_called()
        state.update(navigation_permitted=True,operator_mode='navigation_allowed')
        s.dispatch(command);s.node.confirm_goal.assert_not_called()

    def test_fresh_source_recovery_keeps_original_point_and_requires_new_spin_receipt(self):
        s,e,state,pose=self.fixture();s.dispatch(s.arrival_heading_command(state))
        self.assertTrue(s.source_fault('original observation interrupted'))
        command=s.pending_source_command
        self.assertTrue(command['arrival_heading']);self.assertTrue(command['source_recovery'])
        state['phase']='cancelled';s.dispatch(command)
        self.assertEqual(e.phase,'navigating');self.assertEqual(e.goal_generation,9)
        self.assertTrue(e.pending_arrival_heading)
        # An old action success cannot advance this station.
        old=dict(state,phase='arrived',generation=8)
        self.assertIsNone(s.arrival_heading_command(old))
        self.assertEqual(e.phase,'navigating')
        state['phase']='arrived';pose[2]=.9
        s.arrival_heading_command(state);e.tick(state,3.)
        self.assertEqual(e.phase,'presenting')

    def test_blocked_native_turn_is_not_a_completed_visit(self):
        s,e,state,_=self.fixture();s.dispatch(s.arrival_heading_command(state))
        state.update(phase='blocked',notice='actual native collision')
        e.tick(state,3.);self.assertEqual(e.phase,'failed')
        self.assertFalse(any(x['kind']=='arrived'for x in e.events))

    def test_initial_curve_is_not_replaced_by_a_stationary_turn(self):
        node=NS(reception_pose=lambda:(0.,0.,0.),path=[[0.,0.],[.5,.2],[2.,1.]])
        self.assertEqual(initial_error(node),0.)
        node.path=[[0.,0.],[0.,0.],[-.5,0.],[-2.,0.]]
        self.assertAlmostEqual(abs(initial_error(node)),math.pi)

    def test_initial_spin_waits_for_native_success_then_releases_next_plan(self):
        s,e,state,_=self.fixture();e.phase='dispatching';clock=NS(now=0.)
        def sleep(dt):
            clock.now+=dt
            if clock.now>=1.:state['phase']='arrived'
        self.assertTrue(turn_initial(s.node,e,e.epoch,-1.8,
                                    now=lambda:clock.now,sleep=sleep))
        s.node.prepare_rotation.assert_called_once_with({'radians':-1.8})
        self.assertEqual(s.node.cancel_reception_leg.call_count,2)
        self.assertEqual(e.phase,'dispatching')
        self.assertFalse(any(x['kind']=='arrived'for x in e.events))

    def test_initial_spin_timeout_cancels_actual_action_and_never_claims_completion(self):
        s,e,state,_=self.fixture();e.phase='dispatching';clock=NS(now=0.)
        def sleep(dt):clock.now+=dt
        with self.assertRaisesRegex(ValueError,'超时'):
            turn_initial(s.node,e,e.epoch,2.5,now=lambda:clock.now,sleep=sleep)
        self.assertEqual(s.node.cancel_reception_leg.call_count,2)
        self.assertFalse(any(x.get('phase')=='arrived'for x in e.events))

    def test_intermediate_spin_success_does_not_announce_destination(self):
        from audio_feedback import CuePolicy
        policy=CuePolicy();base=dict(request_token='synthetic',session=dict(phase='executing',generation=1),
            reception=dict(mission=dict(run_id='run',phase='navigating',heading_pending=True)))
        self.assertIsNone(policy.observe(base,0.))
        base['session']=dict(phase='arrived',generation=1)
        self.assertIsNone(policy.observe(base,1.))
        base['session']=dict(phase='executing',generation=2);policy.observe(base,2.)
        base['session']=dict(phase='arrived',generation=2)
        base['reception']['mission']['heading_pending']=False
        cue=policy.observe(base,3.);self.assertEqual(cue.kind,'arrived')


if __name__=='__main__':unittest.main()
