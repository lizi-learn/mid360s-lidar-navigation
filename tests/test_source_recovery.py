"""Actual mission/console/gate methods; no network or robot commands."""
import copy,json,time,unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from test_reception import fixture,permitted
from reception_mission import Mission,MissionService
from test_console_cancellation import fixture as console_fixture
import test_collision_source_gate as source_gate_tests

class SourceRecoveryTests(unittest.TestCase):
    def service(self):
        s=MissionService.__new__(MissionService);s.engine=Mission();doc=fixture();doc['points'][1]['kind']='transit';epoch=s.engine.start(doc,'tour',0)
        command=s.engine.tick(permitted(),0);s.engine.dispatched(epoch,7,command['end_index']);s.active_command=command
        s.source_recovery_active=False;s.pending_source_command=None;s.resume_command=None
        s.node=NS(session=NS(generation=7,replan_timeout=8.,phase='executing'),feedback=dict(generation=7,poses_remaining=1),
            health_ready=Mock(return_value=True),execution_ready=Mock(return_value=True),cancel_reception_leg=Mock(return_value=True),
            session_state=lambda:permitted('cancelled',7),set_goal=Mock(return_value=dict(generation=8)),confirm_goal=Mock())
        s.catalog=NS(read=lambda _:doc);s.node.map_name=doc['map_id']
        return s
    def test_original_task_keeps_target_and_skips_only_receipted_passed_transit(self):
        s=self.service();run=s.engine.run_id;self.assertTrue(s.source_fault('synthetic source gap'))
        cmd=s.pending_source_command;self.assertEqual(s.engine.phase,'rerouting');self.assertEqual([q['id']for q in cmd['points']],['c'])
        self.assertEqual(s.engine.run_id,run);self.assertEqual(cmd['point']['id'],'c')
        deadline=cmd['source_deadline'];s.source_fault('same fault');self.assertEqual(s.pending_source_command['source_deadline'],deadline)
        s.dispatch(cmd);self.assertEqual(s.engine.phase,'navigating');self.assertEqual(s.engine.goal_generation,8)
        s.node.cancel_reception_leg.assert_called_once();s.node.confirm_goal.assert_called_once()
        self.assertFalse(s.source_recovery_active);self.assertNotIn('source_recovery',s.active_command)
    def test_pause_during_new_plan_keeps_remaining_route_and_discards_old_recovery_deadline(self):
        s=self.service();s.source_fault('source gap')
        s.node.feedback={};s.node.session.generation=8
        s.capture_resume()
        self.assertEqual([p['id']for p in s.resume_command['points']],['c'])
        for key in ['reroute','wait','source_recovery','source_deadline']:self.assertNotIn(key,s.resume_command)

    def test_waits_for_actual_health_before_planning(self):
        s=self.service();s.source_fault('source gap');s.node.health_ready.side_effect=[False,True,True]
        with patch('reception_mission.time.sleep')as sleep:s.dispatch(s.pending_source_command)
        sleep.assert_called_once_with(.05);self.assertEqual(s.engine.phase,'navigating')
    def test_manual_takeover_while_waiting_never_plans_or_arms(self):
        s=self.service();s.source_fault('source gap');s.node.health_ready.return_value=False
        with patch('reception_mission.time.sleep',side_effect=lambda _:s.engine.cancel('manual')):s.dispatch(s.pending_source_command)
        self.assertEqual(s.engine.phase,'manual');s.node.set_goal.assert_not_called();s.node.confirm_goal.assert_not_called()
    def test_expired_wait_ends_instead_of_late_restart(self):
        s=self.service();s.source_fault('source gap');s.pending_source_command['source_deadline']=time.monotonic()-1
        s.dispatch(s.pending_source_command);self.assertEqual(s.engine.phase,'failed');s.node.set_goal.assert_not_called()
    def test_geometry_failure_is_not_silently_retried(self):
        s=self.service();s.source_fault('source gap');s.node.set_goal.side_effect=ValueError('actual no path')
        s.dispatch(s.pending_source_command);self.assertEqual(s.engine.phase,'failed');self.assertIn('actual no path',s.engine.error)
    def test_plain_existing_goal_adoption_does_not_modify_catalog(self):
        s=self.service();s.engine=Mission();s.active_command=None;s.node.session.goal=dict(x=2.,y=3.,yaw=.6);s.node.goal_heading_mode='position'
        doc=s.catalog.read('x');original=copy.deepcopy(doc)
        self.assertTrue(s.source_fault('source gap'));self.assertEqual(s.pending_source_command['point']['x'],2.)
        self.assertIsNone(s.pending_source_command['point']['yaw']);self.assertEqual(doc,original)
    def test_lost_invalid_status_still_recovers_from_persistent_revocation(self):
        c,g=console_fixture();c.session.planned(g,3,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        c.reception=NS(source_fault=Mock(return_value=True));c.gait=None
        gate=dict(collision_source_valid=True,data_fresh=True,ground_ok=True,localized=True,scan_age=0.,odom_age=0.,armed=False,collision_fault_generation=g)
        c.on_gate(NS(data=json.dumps(gate)));c.reception.source_fault.assert_called_once();self.assertEqual(c.session.phase,'cancelled')
        self.assertIn('原任务',c.session.message);self.assertTrue(c.emitted[-1]['paused'])
    def test_native_health_revocation_race_can_recover_without_restarting_policy(self):
        c,g=console_fixture();c.session.planned(g,3,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        c.gate=dict(ground_ok=True,localized=True)
        c.reception=NS(source_fault=Mock(return_value=True));c.poll_gait_authority=lambda _:None
        c.gait=NS(status=lambda:dict(cancel_pending=True,phase='failed',native_reason='navigation_health_not_ready'),phase='failed',update=Mock(),stop=Mock(),acknowledge_cancel=Mock())
        c.poll_session();c.reception.source_fault.assert_called_once();self.assertEqual(c.source_revoked_generation,g)
        self.assertIn('原任务',c.session.message)
    def test_native_policy_fault_does_not_enter_observation_recovery(self):
        c,g=console_fixture();c.session.planned(g,3,time.monotonic());c.session.start(g,time.monotonic(),True,True)
        c.gate=dict(ground_ok=True,localized=True)
        c.reception=NS(source_fault=Mock(return_value=True));c.poll_gait_authority=lambda _:None
        c.gait=NS(status=lambda:dict(cancel_pending=True,phase='failed',native_reason='policy_not_ready'),phase='failed',update=Mock(),stop=Mock(),acknowledge_cancel=Mock())
        c.poll_session();c.reception.source_fault.assert_not_called();self.assertEqual(c.session.phase,'cancelled')

    def test_stale_evidence_can_only_select_wait_not_claim_fresh_health(self):
        c,_=console_fixture();c.gate=dict(ground_ok=False,localized=False,ground_evidence=True,localization_evidence_ok=True,data_fresh=False)
        self.assertTrue(c.source_recovery_eligible());self.assertFalse(c.health_ready())
        c.gate['ground_evidence']=False;self.assertFalse(c.source_recovery_eligible())
        c.gate.update(ground_evidence=True,localization_evidence_ok=False);self.assertFalse(c.source_recovery_eligible())

    def test_gate_status_keeps_revocation_evidence_after_boolean_recovers(self):
        n=source_gate_tests.CollisionSourceGateTests().fixture();n.on_collision_source(NS(data=False));n.tick();n.on_collision_source(NS(data=True));n.tick()
        d=json.loads(n.statuses[-1].data);self.assertEqual(d['collision_fault_generation'],4);self.assertTrue(d['collision_source_valid']);self.assertFalse(d['armed'])
