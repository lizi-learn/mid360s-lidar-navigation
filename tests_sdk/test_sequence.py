import copy,json,math,tempfile,threading,time,unittest
from pathlib import Path
import test_contract as fixtures
from navigation_sdk import NavigationError
from navigation_sdk.mock import MockBackend
from navigation_sdk.store import TaskStore
from navigation_sdk.validation import task_request

STEPS=[dict(kind='rotate',radians=-math.pi/2),dict(kind='forward',meters=5.)]
def until(test,predicate):
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:
        if predicate():return
        time.sleep(.01)
    test.fail('bounded sequence condition did not occur')

class OrderedStore(unittest.TestCase):
    def setUp(self):
        self.backend=MockBackend();self.store=TaskStore(self.backend);self.addCleanup(self.store.close)
    def submit(self):return self.store.submit(dict(kind='sequence',steps=STEPS),'sequence-1')
    def test_no_next_step_until_success_and_runs_without_get_polling(self):
        t=self.submit();self.backend.advance('running');time.sleep(.22);self.assertEqual(self.backend.submissions,1)
        self.backend.advance('succeeded')
        until(self,lambda:self.backend.submissions==2)
        self.assertEqual(self.backend.current['request'],STEPS[1]);self.backend.advance('succeeded')
        until(self,lambda:self.store.get(t['id'])['state']=='succeeded')
        final=self.store.get(t['id']);self.assertEqual(final['sequence']['completed'],2)
        self.assertEqual(len(final['sequence']['receipts']),2)
    def test_failed_or_cancelled_or_unknown_first_leg_never_starts_next(self):
        for state in ('failed','cancelled'):
            with self.subTest(state=state):
                b=MockBackend();s=TaskStore(b);self.addCleanup(s.close)
                t=s.submit(dict(kind='sequence',steps=STEPS));b.advance(state)
                until(self,lambda:s.get(t['id'])['state']==state);self.assertEqual(b.submissions,1)
    def test_complete_validation_and_idempotency_apply_to_entire_sequence(self):
        with self.assertRaises(NavigationError):self.store.submit(dict(kind='sequence',steps=[STEPS[0],dict(kind='forward',meters=-1)]))
        self.assertEqual(self.backend.submissions,0)
        t=self.submit();self.assertEqual(self.submit()['id'],t['id']);self.assertEqual(self.backend.submissions,1)
        with self.assertRaises(NavigationError):self.store.submit(dict(kind='sequence',steps=list(reversed(STEPS))),'sequence-1')
        with self.assertRaises(NavigationError):self.store.submit(dict(kind='forward',meters=1))
    def test_status_reports_parent_until_all_steps_finish(self):
        t=self.submit();status=self.store.status()
        self.assertTrue(status['task_active']);self.assertEqual(status['current_task']['task_id'],t['id'])
        self.assertEqual(status['current_task']['total'],2)
    def test_global_stop_and_task_cancel_prevent_later_steps(self):
        t=self.submit();self.store.stop();time.sleep(.2)
        self.assertEqual(self.store.get(t['id'])['state'],'cancelled');self.assertEqual(self.backend.submissions,1)
    def test_pause_requires_explicit_resume(self):
        t=self.submit();self.store.control(t['id'],'pause');time.sleep(.2)
        self.assertEqual(self.store.get(t['id'])['state'],'paused');self.assertEqual(self.backend.submissions,1)
        self.store.control(t['id'],'resume');self.backend.advance('succeeded')
        until(self,lambda:self.backend.submissions==2)
    def test_restart_marks_unknown_and_does_not_replay_or_continue(self):
        with tempfile.TemporaryDirectory()as folder:
            b=MockBackend();s=TaskStore(b,Path(folder)/'tasks.db');t=s.submit(dict(kind='sequence',steps=STEPS));s.close()
            b.advance('succeeded');s=TaskStore(b,Path(folder)/'tasks.db');self.addCleanup(s.close)
            time.sleep(.2);self.assertEqual(s.get(t['id'])['state'],'unknown');self.assertEqual(b.submissions,1)
    def test_cancel_during_late_transition_never_replays(self):
        entered=threading.Event();release=threading.Event();original=self.backend.submit
        def submit(request,**kw):
            if kw.get('after'):entered.set();release.wait(3)
            return original(request,**kw)
        self.backend.submit=submit;t=self.submit();self.backend.advance('succeeded');self.assertTrue(entered.wait(2))
        self.store.stop();release.set();time.sleep(.25)
        self.assertEqual(self.backend.submissions,1);self.assertEqual(self.store.get(t['id'])['state'],'cancelled')
    def test_lost_transition_reply_is_unknown_and_never_retried(self):
        original=self.backend.submit
        def submit(request,**kw):
            ref=original(request,**kw)
            if kw.get('after'):raise NavigationError('UPSTREAM_UNCERTAIN','Synthetic lost accepted reply',status=503)
            return ref
        self.backend.submit=submit;t=self.submit();self.backend.advance('succeeded')
        until(self,lambda:self.store.get(t['id'])['state']=='unknown');time.sleep(.2)
        self.assertEqual(self.backend.submissions,2)
    def test_external_new_task_is_never_overwritten_by_pending_sequence(self):
        entered=threading.Event();release=threading.Event();original=self.backend.submit
        def submit(request,**kw):
            if kw.get('after'):entered.set();release.wait(3)
            return original(request,**kw)
        self.backend.submit=submit;t=self.submit();self.backend.advance('succeeded');self.assertTrue(entered.wait(2))
        replacement=original(dict(kind='forward',meters=1));release.set()
        until(self,lambda:self.store.get(t['id'])['state']=='interrupted')
        self.assertEqual(self.backend.current['id'],replacement['id']);self.assertEqual(self.backend.submissions,2)

class OrderedP2(unittest.TestCase):
    def setUp(self):
        fixtures.P2.setUp(self);self.store=TaskStore(self.backend);self.addCleanup(self.store.close)
    def test_second_goal_uses_measured_post_turn_position_and_heading(self):
        self.server.state['pose']=[1.,2.,0.]
        t=self.store.submit(dict(kind='sequence',steps=STEPS))
        self.assertEqual([p for p,b in self.server.posts],['/api/rotate','/api/confirm'])
        self.server.state['pose']=[1.1,2.2,-math.pi/2+.08]
        self.server.state['session']['phase']='arrived'
        until(self,lambda:len(self.server.posts)==4)
        goal=self.server.state['session']['goal'];yaw=-math.pi/2+.08
        self.assertAlmostEqual(goal['x'],1.1+5*math.cos(yaw));self.assertAlmostEqual(goal['y'],2.2+5*math.sin(yaw))
        self.assertEqual(self.server.posts[2][1]['_sdk_expected_phase'],'arrived')
        self.assertNotEqual(self.store.get(t['id'])['state'],'succeeded')
        self.server.state['session']['phase']='arrived';until(self,lambda:self.store.get(t['id'])['state']=='succeeded')
    def test_manual_takeover_or_failure_ends_remainder(self):
        t=self.store.submit(dict(kind='sequence',steps=STEPS));self.server.state['session']['phase']='manual'
        until(self,lambda:self.store.get(t['id'])['state']=='cancelled')
        self.server.state['session'].update(phase='manual',navigation_permitted=True,operator_mode='navigation_allowed')
        time.sleep(.2);self.assertEqual(len(self.server.posts),2)
    def test_takeover_between_snapshot_and_prepare_cannot_be_armed(self):
        first=self.backend.submit(STEPS[0]);self.server.state['session']['phase']='arrived'
        original=self.backend._post
        def change(path,body):
            if path=='/api/relative-goal':self.server.state['session']['phase']='manual'
            return original(path,body)
        self.backend._post=change
        with self.assertRaises(NavigationError):self.backend.submit(STEPS[1],after=first)
        self.assertEqual([p for p,b in self.server.posts].count('/api/confirm'),1)
    def test_forward_resume_keeps_original_world_target(self):
        ref=self.backend.submit(STEPS[1]);goal=copy.deepcopy(self.server.state['session']['goal'])
        self.server.state['session']['phase']='paused';self.server.state['pose']=[2.,3.,.7]
        new=self.backend.resume_relative(ref,STEPS[1]);body=self.server.posts[-2][1]
        self.assertEqual({k:body[k]for k in('x','y','yaw')},goal)
        self.assertGreater(new['generation'],ref['generation'])
    def test_rotation_resume_uses_remaining_angle_not_full_original_angle(self):
        self.server.state['pose']=[0.,0.,0.];ref=self.backend.submit(STEPS[0]);self.server.state['session']['phase']='paused'
        self.server.state['pose']=[0.,0.,-math.pi/4]
        self.backend.resume_relative(ref,STEPS[0]);self.assertAlmostEqual(self.server.posts[-2][1]['radians'],-math.pi/4)
    def test_middle_during_action_waits_until_explicit_resume(self):
        self.server.state['pose']=[0.,0.,0.]
        t=self.store.submit(dict(kind='sequence',steps=STEPS));self.server.state['pose']=[0.,0.,-.5]
        self.server.state['session'].update(phase='paused',operator_mode='paused',navigation_permitted=False)
        until(self,lambda:self.store.get(t['id'])['state']=='paused')
        self.server.state['session'].update(operator_mode='navigation_allowed',navigation_permitted=True)
        time.sleep(.22);self.assertEqual(len(self.server.posts),2)
        self.store.control(t['id'],'resume');self.assertAlmostEqual(self.server.posts[2][1]['radians'],-math.pi/2+.5)
        self.server.state['session']['phase']='arrived';until(self,lambda:len(self.server.posts)==6)
    def test_middle_after_completed_step_keeps_receipt_and_does_not_repeat_turn(self):
        t=self.store.submit(dict(kind='sequence',steps=STEPS));ss=self.server.state['session']
        ss.update(phase='paused',operator_mode='paused',navigation_permitted=False,
                  terminal_receipt=dict(generation=ss['generation'],succeeded=True))
        until(self,lambda:self.store.get(t['id'])['state']=='paused')
        ss.update(operator_mode='navigation_allowed',navigation_permitted=True)
        self.store.control(t['id'],'resume');until(self,lambda:len(self.server.posts)==4)
        self.assertEqual(self.server.posts[2][0],'/api/relative-goal')
    def test_middle_between_success_read_and_next_prepare_stays_paused(self):
        t=self.store.submit(dict(kind='sequence',steps=STEPS));original=self.backend.submit
        def submit(request,**kw):
            if kw.get('after'):
                ss=self.server.state['session'];ss.update(phase='paused',operator_mode='paused',navigation_permitted=False,
                    terminal_receipt=dict(generation=ss['generation'],succeeded=True))
                self.backend.submit=original
            return original(request,**kw)
        self.backend.submit=submit;self.server.state['session']['phase']='arrived'
        until(self,lambda:self.store.get(t['id'])['state']=='paused');self.assertEqual(len(self.server.posts),2)
        self.server.state['session'].update(operator_mode='navigation_allowed',navigation_permitted=True)
        self.store.control(t['id'],'resume');until(self,lambda:len(self.server.posts)==4)
    def test_cancel_between_prepare_and_confirm_sends_no_new_motion(self):
        ref=self.backend.submit(STEPS[0]);self.server.state['session']['phase']='arrived';checks=[]
        def cancelled():checks.append(True);return len(checks)>1
        with self.assertRaises(NavigationError):self.backend.submit(STEPS[1],after=ref,cancelled=cancelled)
        self.assertEqual([p for p,b in self.server.posts].count('/api/confirm'),1)
        self.assertEqual(self.server.state['session']['phase'],'cancelled')
