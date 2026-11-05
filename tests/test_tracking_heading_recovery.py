"""Local action abort -> checked native turn -> same visit, never a new task."""
import copy
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from concurrent.futures import Future

from test_reception import fixture, permitted
from heading_recovery import plan_after_turn
from reception_mission import Mission, MissionService
from session import NavigationSession
from test_console_cancellation import fixture as console_fixture, Handle


class TrackingHeadingRecoveryTests(unittest.TestCase):
    def fixture(self):
        engine=Mission();epoch=engine.start(fixture(),'tour',0.)
        command=engine.tick(permitted('ready',4),0.)
        engine.dispatched(epoch,4,command['end_index'])
        state=permitted('blocked',4)
        state.update(manual_activity=False,last_stop=None,terminal_receipt=dict(generation=4,status=6,task_kind='navigation',succeeded=False))
        node=NS(gait_profile='p2-flat-model',session=NS(generation=4),feedback={},
                reception_pose=Mock(return_value=[0.,0.,0.]),session_state=lambda:dict(state),
                health_ready=Mock(return_value=True),execution_ready=Mock(return_value=True),
                cancel_reception_leg=Mock(return_value=True))
        node.prepare_rotation=Mock(side_effect=lambda d:state.update(generation=5,phase='ready')or dict(generation=5))
        def confirm(data):
            state['phase']='arrived'if data['generation']==5 else 'executing'
            if data['generation']==5:node.reception_pose.return_value=[0.,0.,math.pi/6]
        node.confirm_goal=Mock(side_effect=confirm)
        node.set_goal=Mock(side_effect=lambda d:state.update(generation=7,phase='ready')or dict(generation=7))
        service=MissionService.__new__(MissionService)
        service.engine=engine;service.node=node;service.active_command=copy.deepcopy(command)
        service.future=None;service.resume_command=None;service.source_recovery_active=False;service.tracking_turn_leg=None
        return service,state,command

    def test_valid_global_plan_local_abort_uses_turn_before_replanning_original_visit(self):
        s,state,original=self.fixture()
        command=s.tracking_recovery_command(state)
        self.assertEqual(s.engine.phase,'rerouting')
        s.dispatch(command)
        self.assertEqual(s.engine.phase,'navigating',s.engine.error)
        self.assertEqual(s.engine.goal_generation,7)
        self.assertEqual(s.node.set_goal.call_count,1,'No unchanged global preview before the turn')
        data=s.node.set_goal.call_args.args[0]
        self.assertEqual((data['x'],data['y']),(original['point']['x'],original['point']['y']))
        self.assertEqual(s.active_command['point'],original['point'],'Retain introduction, heading and dwell')
        self.assertNotIn('tracking_recovery',s.active_command)
        self.assertEqual(s.node.confirm_goal.call_count,2)
        self.assertFalse(any(e['kind']=='arrived'for e in s.engine.events),'A completed Spin is not visit arrival')

    def test_second_abort_of_same_leg_finishes_instead_of_looping_turns(self):
        s,state,_=self.fixture();s.dispatch(s.tracking_recovery_command(state))
        state.update(phase='blocked',generation=7,terminal_receipt=dict(generation=7,status=6,task_kind='navigation',succeeded=False))
        self.assertIsNone(s.tracking_recovery_command(state))
        s.engine.tick(state,1.)
        self.assertEqual(s.engine.phase,'failed');self.assertEqual(s.node.prepare_rotation.call_count,1)

    def test_cancelled_missing_stale_or_operator_stop_never_grants_rotation(self):
        changes=[dict(terminal_receipt=None),dict(last_stop={'reason':'sensor timeout'}),
                 dict(navigation_permitted=False),dict(manual_activity=True),dict(phase='manual'),dict(phase='paused'),
                 dict(generation=88)]
        for status in [None,2,4,5]:
            changes.append(dict(terminal_receipt=dict(generation=4,status=status,task_kind='navigation',succeeded=False)))
        for field,value in [('generation',3),('task_kind','rotation'),('succeeded',True)]:
            receipt=dict(generation=4,status=6,task_kind='navigation',succeeded=False);receipt[field]=value
            changes.append(dict(terminal_receipt=receipt))
        for change in changes:
            with self.subTest(change=change):
                s,state,_=self.fixture();state.update(change)
                self.assertIsNone(s.tracking_recovery_command(state))
                self.assertEqual(s.engine.phase,'navigating');s.node.prepare_rotation.assert_not_called()

    def test_existing_worker_or_replaced_command_is_not_recovered(self):
        for kind in ['worker','epoch','profile']:
            s,state,_=self.fixture()
            if kind=='worker':s.future=NS(done=lambda:False)
            elif kind=='epoch':s.active_command['epoch']+=1
            else:s.node.gait_profile='other'
            self.assertIsNone(s.tracking_recovery_command(state));s.node.prepare_rotation.assert_not_called()

    def test_manual_after_queue_invalidates_worker_without_turn_or_rearm(self):
        s,state,_=self.fixture();command=s.tracking_recovery_command(state)
        s.engine.cancel('manual');state.update(navigation_permitted=False,phase='manual')
        s.dispatch(command)
        self.assertEqual(s.engine.phase,'manual')
        s.node.prepare_rotation.assert_not_called();s.node.confirm_goal.assert_not_called();s.node.set_goal.assert_not_called()

    def test_passed_transit_points_are_not_reintroduced(self):
        s,state,_=self.fixture()
        final=copy.deepcopy(s.active_command['point'])
        s.active_command['points']=[dict(final,id='passed'),dict(final,id='still_ahead'),final]
        s.node.feedback=dict(generation=4,poses_remaining=2)
        command=s.tracking_recovery_command(state)
        self.assertEqual([p['id']for p in command['points']],['still_ahead',final['id']])
        self.assertEqual(command['point'],final)
        self.assertEqual([e['point']for e in s.engine.events if e['kind']=='transit_segment_completed'],['passed'])

    def test_zero_turn_abort_tries_opposite_direction_before_reusing_valid_global_plan(self):
        s,state,_=self.fixture();angles=[]
        def prepare(data):
            angles.append(data['radians']);state.update(generation=4+len(angles),phase='ready')
            return dict(generation=state['generation'])
        def confirm(_):
            state['phase']='blocked'
            if len(angles)==2:s.node.reception_pose.return_value=[0.,0.,-.12]
        s.node.prepare_rotation.side_effect=prepare;s.node.confirm_goal.side_effect=confirm
        result=plan_after_turn(s.node,s.engine,s.engine.epoch,dict(x=3.,y=0.,yaw=0.),ValueError('original tracking abort'),require_turn_progress=True)
        self.assertEqual(result,dict(generation=7));self.assertEqual(angles,[math.pi/6,-math.pi/6])
        s.node.set_goal.assert_called_once()
        self.assertEqual(len([e for e in s.engine.events if e['kind']=='heading_recovery_no_progress']),1)

    def test_both_stationary_rejections_preserve_obstacle_failure_and_end(self):
        s,state,_=self.fixture();s.node.confirm_goal.side_effect=lambda _:state.update(phase='blocked')
        with self.assertRaisesRegex(ValueError,'original tracking abort'):
            plan_after_turn(s.node,s.engine,s.engine.epoch,dict(x=3.,y=0.,yaw=0.),ValueError('original tracking abort'),require_turn_progress=True)
        self.assertEqual(s.node.prepare_rotation.call_count,2);s.node.set_goal.assert_not_called()

    def test_session_terminal_receipt_is_real_current_action_and_reset_on_next_goal(self):
        s=NavigationSession();s.phase='executing';s.generation=4
        self.assertTrue(s.finished(4,False,action_status=6))
        receipt=s.status(1.)['terminal_receipt']
        self.assertEqual(receipt,dict(generation=4,status=6,task_kind='navigation',succeeded=False))
        self.assertFalse(s.finished(3,True,action_status=4));self.assertEqual(s.status(1.)['terminal_receipt'],receipt)
        receipt['status']=4;self.assertEqual(s.terminal_receipt['status'],6)
        s.prepare(dict(x=1.,y=2.,yaw=0.),1.)
        self.assertIsNone(s.status(1.)['terminal_receipt'])

    def test_actual_console_callback_preserves_aborted_vs_cancelled_receipt(self):
        for status in [5,6]:
            with self.subTest(status=status):
                console,g=console_fixture();console.session.phase='starting'
                handle=Handle();accepted=Future();accepted.set_result(handle)
                console.track_acceptance('nav',g,accepted)
                handle.result.set_result(NS(status=status))
                self.assertEqual(console.session.terminal_receipt['status'],status)
                self.assertEqual(console.session.phase,'blocked')
                self.assertEqual(console.emitted[-1],dict(armed=False,paused=True))
