"""Mission turn->actual result->replan ordering, including takeover races."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from test_reception import fixture, permitted
from reception_mission import Mission, MissionService
from heading_recovery import plan_after_turn, preferred_side


class HeadingRecoveryTests(unittest.TestCase):
    def fixture(self):
        engine = Mission();epoch = engine.start(fixture(), 'tour', 0.)
        state = permitted('blocked', 4);state['manual_activity'] = False
        node = NS(session_state=lambda: dict(state), reception_pose=Mock(return_value=[0., 0., 0.]),
                  health_ready=Mock(return_value=True), execution_ready=Mock(return_value=True),
                  cancel_reception_leg=Mock(return_value=True), set_goal=Mock(return_value={'generation': 6}))
        turns = []
        def prepare(data):
            turns.append(data['radians']);state.update(generation=4+len(turns),phase='ready')
            return dict(generation=state['generation'])
        def confirm(data):
            self.assertTrue(engine.valid(data['_mission_epoch']))
            self.assertEqual(data['generation'], state['generation'])
            state['phase'] = 'arrived'
        node.prepare_rotation=Mock(side_effect=prepare);node.confirm_goal=Mock(side_effect=confirm)
        return engine,epoch,node,state,turns

    def run_recovery(self, node, engine, epoch, data=None, **kwargs):
        return plan_after_turn(node,engine,epoch,data or dict(x=3.,y=0.,yaw=1.,_mission_epoch=epoch),ValueError('未找到可通行路径'),**kwargs)

    def test_replans_original_yaw_and_route_only_after_real_turn_result(self):
        e,epoch,n,state,turns=self.fixture();data=dict(x=3.,y=0.,yaw=1.,_mission_epoch=epoch,_route_goals=[dict(x=1.,y=0.,yaw=.3)])
        order=[]
        n.cancel_reception_leg.side_effect=lambda _:order.append('cancel_receipt') or True
        original=n.confirm_goal.side_effect
        n.confirm_goal.side_effect=lambda d:order.append('turn_accepted') or original(d)
        n.set_goal.side_effect=lambda d:order.append('replan') or dict(generation=8)
        self.assertEqual(self.run_recovery(n,e,epoch,data),dict(generation=8))
        n.set_goal.assert_called_once_with(data);self.assertEqual(order,['cancel_receipt','turn_accepted','cancel_receipt','replan'])
        self.assertEqual(e.phase,'dispatching') # a turn is not an arrived visit
        self.assertAlmostEqual(turns[0],math.pi/6)

    def test_partial_collision_abort_is_recorded_then_current_map_is_replanned(self):
        e,epoch,n,state,_=self.fixture()
        def abort(_):state['phase']='blocked';n.reception_pose.return_value=[0.,0.,.12]
        n.confirm_goal.side_effect=abort
        self.assertEqual(self.run_recovery(n,e,epoch),dict(generation=6))
        receipt=next(x for x in e.events if x['kind']=='heading_recovery_finished')
        self.assertEqual(receipt['phase'],'blocked');self.assertAlmostEqual(receipt['measured_radians'],.12)

    def test_two_failures_do_not_restart_or_lose_original_failure(self):
        e,epoch,n,state,turns=self.fixture();n.set_goal.side_effect=ValueError('未找到可通行路径，原障碍保留')
        with self.assertRaisesRegex(ValueError,'原障碍保留'):self.run_recovery(n,e,epoch)
        self.assertEqual(len(turns),2);self.assertAlmostEqual(turns[0],-turns[1]);self.assertEqual(n.set_goal.call_count,2)

    def test_stale_source_does_not_spin(self):
        e,epoch,n,state,_=self.fixture();n.health_ready.return_value=False
        clock=[0.]
        with self.assertRaises(ValueError):self.run_recovery(n,e,epoch,now=lambda:clock[0],sleep=lambda _:clock.__setitem__(0,clock[0]+1.))
        n.prepare_rotation.assert_not_called();n.set_goal.assert_not_called()

    def test_transient_cancel_readiness_waits_at_zero_then_uses_real_reply(self):
        e,epoch,n,state,_=self.fixture();n.execution_ready.side_effect=[False,False,True,True,True]
        clock=[0.];sleeps=[]
        def sleep(dt):clock[0]+=dt;sleeps.append(dt)
        self.assertEqual(self.run_recovery(n,e,epoch,now=lambda:clock[0],sleep=sleep),dict(generation=6))
        self.assertEqual(sleeps,[.05,.05]);self.assertEqual(n.prepare_rotation.call_count,1)
        n.set_goal.assert_called_once()
    def test_manual_during_readiness_wait_never_spins_or_restarts(self):
        e,epoch,n,state,_=self.fixture();n.execution_ready.return_value=False
        def sleep(_):state.update(navigation_permitted=False,operator_mode='manual',phase='manual')
        self.assertIsNone(self.run_recovery(n,e,epoch,sleep=sleep))
        self.assertEqual(e.phase,'manual');n.prepare_rotation.assert_not_called();n.confirm_goal.assert_not_called();n.set_goal.assert_not_called()
    def test_manual_takeover_during_turn_never_replans_or_rearms(self):
        e,epoch,n,state,_=self.fixture()
        def takeover(_):state.update(navigation_permitted=False,operator_mode='manual',phase='manual');e.cancel('manual')
        n.confirm_goal.side_effect=takeover
        self.assertIsNone(self.run_recovery(n,e,epoch));n.set_goal.assert_not_called();self.assertEqual(n.confirm_goal.call_count,1)

    def test_cancelled_action_without_takeover_does_not_silently_redispatch(self):
        e,epoch,n,state,_=self.fixture();n.confirm_goal.side_effect=lambda _:state.update(phase='cancelled',notice='真实动作取消')
        with self.assertRaisesRegex(ValueError,'真实动作取消'):self.run_recovery(n,e,epoch)
        n.set_goal.assert_not_called()

    def test_observed_right_then_left_stays_cancelled_without_polling_the_right_edge(self):
        e,epoch,n,state,_=self.fixture()
        n.confirm_goal.side_effect=lambda _:state.update(navigation_permitted=False,operator_mode='manual')
        self.assertIsNone(self.run_recovery(n,e,epoch))
        state.update(navigation_permitted=True,operator_mode='navigation_allowed')
        self.assertEqual(e.phase,'manual');self.assertIsNone(e.tick(state,1.));n.set_goal.assert_not_called()

    def test_observed_middle_then_left_requires_explicit_resume(self):
        e,epoch,n,state,_=self.fixture()
        n.confirm_goal.side_effect=lambda _:state.update(navigation_permitted=False,operator_mode='pause')
        self.assertIsNone(self.run_recovery(n,e,epoch))
        state.update(navigation_permitted=True,operator_mode='navigation_allowed')
        self.assertEqual(e.phase,'paused');self.assertIsNone(e.tick(state,1.));n.set_goal.assert_not_called()

    def test_session_retains_intervention_even_when_left_is_already_fresh(self):
        for phase in ['manual','paused']:
            e,epoch,n,state,_=self.fixture();state.update(phase=phase,navigation_permitted=True,operator_mode='navigation_allowed')
            self.assertIsNone(self.run_recovery(n,e,epoch));self.assertEqual(e.phase,phase)
            n.prepare_rotation.assert_not_called();n.set_goal.assert_not_called()

    def test_action_wait_is_bounded_and_cancels_before_error(self):
        e,epoch,n,state,_=self.fixture();n.confirm_goal.side_effect=lambda _:state.update(phase='executing')
        t=[0.]
        def sleep(_):t[0]+=46.
        with self.assertRaisesRegex(ValueError,'动作期限'):self.run_recovery(n,e,epoch,now=lambda:t[0],sleep=sleep)
        self.assertEqual(n.cancel_reception_leg.call_count,2);n.set_goal.assert_not_called()

    def test_side_preference_uses_current_returns_without_refreshing_them(self):
        e,epoch,n,state,_=self.fixture();n.scan_preview=dict(frame_id='map',source_ns=900000000,points=[[.1,-.4],[.2,2.]])
        n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=1000000000))
        self.assertEqual(preferred_side(n,[0.,0.,0.],dict(x=3.,y=-.1)),1.)
        n.scan_preview['source_ns']=1
        self.assertEqual(preferred_side(n,[0.,0.,0.],dict(x=3.,y=-.1)),-1.)
        self.assertEqual(n.scan_preview['source_ns'],1)

    def test_started_mission_dispatch_recovers_then_starts_the_same_visit(self):
        e,epoch,n,state,_=self.fixture();service=MissionService.__new__(MissionService)
        service.engine=e;service.node=n;service.resume_command=None;service.active_command=None;service.source_recovery_active=False
        n.gait_profile='p2-flat-model';n.session=NS(generation=4)
        commands=[]
        def plan(data):
            commands.append(dict(data))
            if len(commands)==1:raise ValueError('未找到可通行路径，速度保持为零')
            state.update(generation=7,phase='ready');return dict(generation=7)
        n.set_goal.side_effect=plan
        def confirm(data):state['phase']='arrived' if len(commands)==1 else 'executing'
        n.confirm_goal.side_effect=confirm
        command=e.tick(permitted('blocked',4),0.)
        service.dispatch(command)
        self.assertEqual(e.phase,'navigating');self.assertEqual(e.goal_generation,7)
        self.assertEqual(commands[0],commands[1]);self.assertEqual(n.confirm_goal.call_count,2)
        self.assertEqual(service.active_command['point']['id'],command['point']['id'])

    def test_other_dispatch_errors_never_request_a_turn(self):
        e,epoch,n,state,_=self.fixture();service=MissionService.__new__(MissionService)
        service.engine=e;service.node=n;service.resume_command=None;service.active_command=None;service.source_recovery_active=False
        n.gait_profile='p2-flat-model';n.session=NS(generation=4);n.set_goal.side_effect=ValueError('目标在地图外')
        service.dispatch(e.tick(permitted('blocked',4),0.))
        self.assertEqual(e.phase,'failed');n.prepare_rotation.assert_not_called();n.confirm_goal.assert_not_called()
