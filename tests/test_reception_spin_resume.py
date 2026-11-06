import math,unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import test_reception_native_spin as fixtures
from reception_spins import wait_after_interruption,turn_initial

class SpinWaitTests(unittest.TestCase):
 def fixture(self):
  s,e,state,pose=fixtures.ReceptionNativeSpinTests().fixture()
  s.node.session.replan_timeout=25.;s.node.collision_stop=False
  s.dispatch(s.arrival_heading_command(state))
  state.update(phase='blocked',terminal_receipt=dict(generation=e.goal_generation,status=6,task_kind='rotation',succeeded=False),notice='native Spin aborted for collision')
  return s,e,state,pose
 def test_abort_waits_then_recomputes_remaining_heading_and_requires_new_result(self):
  s,e,state,pose=self.fixture();original=e.run_id;command=s.interrupted_arrival_spin_command(state)
  self.assertEqual(e.phase,'rerouting');self.assertIsNone(e.tick(state,1.))
  self.assertFalse(any(x['kind']=='arrived'for x in e.events))
  pose[2]=.4
  with patch('reception_spins.wait_after_interruption',return_value=True):s.dispatch(command)
  self.assertEqual(s.node.prepare_rotation.call_count,2)
  self.assertAlmostEqual(s.node.prepare_rotation.call_args.args[0]['radians'],.5)
  self.assertEqual(e.run_id,original);self.assertEqual(e.phase,'navigating')
  self.assertEqual(e.index,0);self.assertEqual(e.goal_generation,9)
  self.assertEqual(e.tick(state,2.),None)
  state.update(phase='arrived');pose[2]=.9;s.arrival_heading_command(state);e.tick(state,3.)
  self.assertEqual(e.phase,'presenting');self.assertEqual(sum(x['kind']=='arrived'for x in e.events),1)
 def test_repeated_abort_keeps_the_original_wait_deadline(self):
  s,e,state,_=self.fixture();c=s.interrupted_arrival_spin_command(state);deadline=c['turn_wait_deadline']
  with patch('reception_spins.wait_after_interruption',return_value=True):s.dispatch(c)
  state.update(phase='blocked',terminal_receipt=dict(generation=e.goal_generation,status=6,task_kind='rotation',succeeded=False))
  c=s.interrupted_arrival_spin_command(state);self.assertEqual(c['turn_wait_deadline'],deadline)
 def test_operator_cancel_and_old_generation_are_not_retried(self):
  for change in [dict(navigation_permitted=False),dict(manual_activity=True),dict(last_stop={'reason':'cancel'}),dict(generation=1)]:
   s,e,state,_=self.fixture();state.update(change)
   self.assertIsNone(s.interrupted_arrival_spin_command(state));self.assertEqual(s.node.prepare_rotation.call_count,1)
 def test_wait_requires_continuous_clear_monitor_and_original_health(self):
  s,e,state,_=self.fixture();clock=NS(t=0.)
  s.node.health_ready=lambda:clock.t>=.5
  def sleep(dt):
   clock.t+=dt;s.node.collision_stop=1.<clock.t<2.
  self.assertTrue(wait_after_interruption(s.node,e,e.epoch,10.,now=lambda:clock.t,sleep=sleep))
  self.assertGreaterEqual(clock.t,3.5);self.assertEqual(s.node.prepare_rotation.call_count,1)
 def test_monitor_stop_has_its_own_evidence_not_a_fabricated_action_abort(self):
  s,e,state,_=self.fixture();state['terminal_receipt']['status']=None
  self.assertIsNone(s.interrupted_arrival_spin_command(state))
  s.node.rotation_collision_generation=state['generation']
  self.assertIsNotNone(s.interrupted_arrival_spin_command(state))
  self.assertIsNone(state['terminal_receipt']['status'])
 def test_persistent_obstacle_times_out_without_an_action_or_fake_arrival(self):
  s,e,state,_=self.fixture();clock=NS(t=0.);s.node.collision_stop=True
  def sleep(dt):clock.t+=dt
  with self.assertRaisesRegex(ValueError,'持续受阻'):
   wait_after_interruption(s.node,e,e.epoch,3.,now=lambda:clock.t,sleep=sleep)
  self.assertEqual(s.node.prepare_rotation.call_count,1)
  self.assertFalse(any(x['kind']=='arrived'for x in e.events))
 def test_right_during_wait_latches_manual_even_after_return_to_left(self):
  s,e,state,_=self.fixture();clock=NS(t=0.)
  def sleep(dt):clock.t+=dt;state.update(navigation_permitted=False,operator_mode='manual')
  self.assertFalse(wait_after_interruption(s.node,e,e.epoch,10.,now=lambda:clock.t,sleep=sleep))
  self.assertEqual(e.phase,'manual');state.update(navigation_permitted=True,operator_mode='navigation_allowed')
  self.assertFalse(wait_after_interruption(s.node,e,e.epoch,10.,now=lambda:clock.t,sleep=sleep))
 def test_initial_partial_turn_resumes_remaining_angle_not_full_original_angle(self):
  s,e,state,pose=self.fixture();e.phase='dispatching';pose[2]=0.;clock=NS(t=0.);s.node.prepare_rotation.reset_mock();s.node.confirm_goal.reset_mock()
  original=s.node.confirm_goal.side_effect
  def confirm(data):original(data)
  s.node.confirm_goal.side_effect=confirm
  def sleep(dt):
   clock.t+=dt
   if state['phase']=='executing':
    if s.node.prepare_rotation.call_count==1:
     pose[2]=.4;state.update(phase='blocked',terminal_receipt=dict(generation=state['generation'],status=6,task_kind='rotation',succeeded=False))
    else:pose[2]=1.8;state['phase']='arrived'
  self.assertTrue(turn_initial(s.node,e,e.epoch,1.8,now=lambda:clock.t,sleep=sleep))
  self.assertAlmostEqual(s.node.prepare_rotation.call_args.args[0]['radians'],1.4)
  self.assertEqual(e.phase,'dispatching');self.assertFalse(any(x['kind']=='arrived'for x in e.events))

if __name__=='__main__':unittest.main()
