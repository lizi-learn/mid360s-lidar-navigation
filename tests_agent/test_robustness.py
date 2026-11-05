"""Adversarial model outputs exercise actual RAI/HTTP task handling offline."""
import tempfile
import threading
import time
from pathlib import Path
import unittest
from langchain_core.messages import AIMessage
from test_agent import ScriptedModel
from navigation_agent.app import NavigationAgent,requested_controls
from navigation_sdk import NavigationClient
from navigation_sdk.gateway import GatewayServer
from navigation_sdk.mock import MockBackend
from navigation_sdk.store import TaskStore

class ObservedBackend(MockBackend):
    def __init__(self):super().__init__();self.actions=[];self.finish_on_submit=False
    def submit(self,body):
        self.actions.append(('submit',body.copy()));ref=super().submit(body)
        if self.finish_on_submit:super().advance('succeeded')
        return ref
    def stop(self):self.actions.append(('stop',{}));return super().stop()
    def control(self,ref,operation):self.actions.append((operation,{}));return super().control(ref,operation)

class Robustness(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.backend=ObservedBackend();self.store=TaskStore(self.backend)
        self.server=GatewayServer(('127.0.0.1',0),self.store)
        self.thread=threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.02),daemon=True);self.thread.start()
        self.nav=NavigationClient('http://127.0.0.1:'+str(self.server.server_port),timeout=2);self.agents=[]
    def tearDown(self):
        for a in self.agents:a.executor.shutdown(wait=True);a.journal.db.close()
        self.server.shutdown();self.server.server_close();self.store.close();self.tmp.cleanup()
    def agent(self,script):
        a=NavigationAgent(self.nav,Path(self.tmp.name)/('agent'+str(len(self.agents))+'.sqlite3'),ScriptedModel(script=script));self.agents.append(a);return a
    def wait(self,a,ident='r'):
        end=time.monotonic()+8
        while time.monotonic()<end:
            job=a.journal.job(ident)
            if job['state']not in('queued','working'):return job
            time.sleep(.01)
        self.fail('Offline graph timed out')
    def test_readonly_question_cannot_start_even_when_model_calls_visit(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已到达'])
        a.submit('r','c','只查询当前状态，不要启动导航');j=self.wait(a)
        self.assertEqual(self.backend.submissions,0)
        self.assertNotIn('已到达',j['reply'])
    def test_explicit_target_cannot_be_replaced_by_different_model_point(self):
        a=self.agent([('visit_point',{'point_id':'display'}),'已开始'])
        a.submit('r','c','去接待起点');self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertEqual(self.backend.current['request']['marker'],'welcome')
    def test_one_navigation_instruction_cannot_dispatch_visit_then_another_route(self):
        self.backend.finish_on_submit=True
        a=self.agent([('visit_point',{'point_id':'welcome'}),('run_route',{'route_id':'sample-tour'}),'已完成'])
        a.submit('r','c','去接待起点');self.wait(a)
        self.assertEqual(self.backend.submissions,1)
    def test_missing_action_receipt_cannot_be_reported_as_arrival(self):
        a=self.agent(['已经到达接待起点'])
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertTrue(self.backend.submissions==1 or '已经到达'not in j['reply'])
        if self.backend.submissions:self.assertEqual(j['task']['state'],'accepted')
    def test_query_about_pause_does_not_authorize_pause(self):
        for text in ('暂停了吗？','恢复了吗？','继续了吗？','停止了吗？','能查询暂停状态吗？'):
            with self.subTest(text=text):self.assertEqual(requested_controls(text),set())
    def test_explicit_cancel_comma_go_does_not_cancel_its_own_new_start(self):
        previous=self.nav.visit('接待起点')
        a=self.agent([('control_navigation',{'operation':'cancel'}),('visit_point',{'point_id':'display'}),'已开始'])
        a.journal.save_conversation('c',dict(history=[],task_id=previous['id']))
        a.submit('r','c','取消当前任务，去产品展示');j=self.wait(a)
        self.assertEqual(self.backend.submissions,2)
        self.assertEqual(j['task']['request']['marker'],'display')
    def test_model_exception_after_acceptance_keeps_real_task_receipt(self):
        a=self.agent([('visit_point',{'point_id':'welcome'})])
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertEqual(j.get('task',{}).get('state'),'accepted')
        self.assertEqual(self.backend.current['state'],'accepted')
    def test_unknown_target_cannot_be_silently_substituted(self):
        a=self.agent([('visit_point',{'point_id':'display'}),'已开始'])
        a.submit('r','c','去不存在的月球展厅');j=self.wait(a)
        self.assertEqual(self.backend.submissions,0)
    def test_repeated_delivery_same_uuid_remains_single_dispatch_under_threads(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已开始'])
        threads=[threading.Thread(target=lambda:a.submit('r','c','去接待起点'))for _ in range(12)]
        for t in threads:t.start()
        for t in threads:t.join()
        self.wait(a);self.assertEqual(self.backend.submissions,1)
    def test_failure_query_does_not_restart_completed_or_failed_task(self):
        previous=self.nav.visit('接待起点');self.backend.advance('failed');self.nav.task(previous['id'])
        a=self.agent([('visit_point',{'point_id':'welcome'}),'正在重试'])
        a.journal.save_conversation('c',dict(history=[],task_id=previous['id']))
        a.submit('r','c','为什么刚才导航失败了？只查原因');self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertEqual(self.backend.current['state'],'failed')

    def test_catalogue_query_after_arrival_is_not_replaced_by_task_completion_reply(self):
        t=self.nav.visit('接待起点');self.backend.advance('succeeded');self.nav.task(t['id'])
        a=self.agent(['目前有两个点位'])
        a.journal.save_conversation('c',dict(history=[],task_id=t['id']))
        a.submit('r','c','只查询：现在有多少个点位');j=self.wait(a)
        self.assertEqual(j['reply'],'目前有两个点位');self.assertEqual(self.backend.submissions,1)
    def test_model_failure_before_dispatch_completes_the_single_bound_command(self):
        a=self.agent([])
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1);self.assertEqual(j['task']['state'],'accepted')
    def test_observation_failure_after_acceptance_keeps_receipt_marked_stale(self):
        from navigation_sdk.errors import NavigationError
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已开始'])
        a.nav.task=lambda _:(_ for _ in()).throw(NavigationError('UPSTREAM_UNAVAILABLE','offline',status=503))
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1);self.assertEqual(j['task']['state'],'accepted')
        self.assertTrue(j['task']['observation_stale'])
    def test_uncertain_submission_is_not_replayed_even_if_model_changes_tool_kind(self):
        from navigation_sdk.errors import NavigationError
        original=self.backend.submit
        def lose_reply(body):original(body);raise NavigationError('UPSTREAM_UNCERTAIN','lost reply',status=503)
        self.backend.submit=lose_reply
        a=self.agent([('visit_point',{'point_id':'welcome'}),('run_route',{'route_id':'sample-tour'}),'已完成'])
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1);self.assertNotIn('任务已完成',j['reply'])
    def test_context_outage_does_not_issue_a_blind_goal(self):
        from navigation_sdk.errors import NavigationError
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已到达'])
        a.nav.status=lambda:(_ for _ in()).throw(NavigationError('UPSTREAM_UNAVAILABLE','status offline',status=503))
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,0);self.assertIn('无法读取',j['reply'])
    def test_unpolled_completed_task_is_saved_before_next_sdk_dispatch(self):
        previous=self.nav.visit('接待起点');self.backend.advance('succeeded')
        self.nav.visit('产品展示')
        self.assertEqual(self.nav.task(previous['id'])['state'],'succeeded')
    def test_deleted_target_between_context_and_dispatch_never_fakes_start(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已开始'])
        original=a.nav.markers;count=[0]
        def live_markers():
            count[0]+=1
            return original()if count[0]==1 else []
        a.nav.markers=live_markers
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,0);self.assertNotIn('已开始',j['reply'])
    def test_pause_model_error_is_not_retried_without_a_new_request(self):
        from navigation_sdk.errors import NavigationError
        t=self.nav.visit('接待起点');calls=[]
        a=self.agent([('control_navigation',{'operation':'pause'}),'未确认'])
        a.journal.save_conversation('c',dict(history=[],task_id=t['id']))
        def unknown(_):calls.append(1);raise NavigationError('UPSTREAM_UNCERTAIN','lost control reply',status=503)
        a.nav.pause=unknown
        a.submit('r','c','暂停');self.wait(a)
        self.assertEqual(calls,[1])
    def test_stop_cancels_queued_request_and_no_later_fallback_dispatches(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已开始'])
        entered=threading.Event();release=threading.Event();a.llm._entered=entered;a.llm._release=release
        a.submit('r','c','去接待起点');self.assertTrue(entered.wait(3))
        a.submit('queued','c','去产品展示');a.stop();release.set()
        self.wait(a);self.wait(a,'queued');self.assertEqual(self.backend.submissions,0)
    def test_repeat_stop_is_not_held_behind_cloud_model(self):
        a=self.agent(['只读回答']);entered=threading.Event();release=threading.Event()
        a.llm._entered=entered;a.llm._release=release
        a.submit('r','c','查询当前状态');self.assertTrue(entered.wait(3))
        start=time.monotonic();a.submit('stop','c','停一下');elapsed=time.monotonic()-start
        self.assertLess(elapsed,.5);self.assertEqual(a.journal.job('stop')['state'],'done')
        release.set();self.wait(a)
    def test_malformed_reply_after_native_acceptance_is_unknown_not_not_started(self):
        original=self.nav.visit
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已到达'])
        def malformed(*args,**kwargs):original(*args,**kwargs);return {'state':'accepted'}
        a.nav.visit=malformed;a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertNotIn('未启动',j['reply']);self.assertIn('未确认',j['reply'])
    def test_later_local_tool_error_cannot_hide_an_unknown_dispatch_outcome(self):
        from navigation_sdk.errors import NavigationError
        original=self.backend.submit
        def lost(body):original(body);raise NavigationError('UPSTREAM_UNCERTAIN','lost reply',status=503)
        self.backend.submit=lost
        a=self.agent([('visit_point',{'point_id':'welcome'}),('run_route',{'route_id':'sample-tour'}),'已开始'])
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertIn('未确认',j['reply']);self.assertNotIn('未启动',j['reply'])
    def test_malformed_observation_keeps_the_original_accepted_receipt(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已开始']);a.nav.task=lambda _:{}
        a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(self.backend.submissions,1)
        self.assertEqual(j.get('task',{}).get('state'),'accepted');self.assertTrue(j['task']['observation_stale'])
    def test_latest_status_tool_receipt_overrides_start_of_turn_state(self):
        t=self.nav.visit('接待起点')
        a=self.agent([('navigation_status',{}),'已完成']);a.journal.save_conversation('c',dict(history=[],task_id=t['id']))
        original=a.nav.status;calls=[0]
        def current():
            calls[0]+=1
            if calls[0]==2:self.backend.advance('succeeded')
            return original()
        a.nav.status=current;a.submit('r','c','查询刚才任务是否完成');j=self.wait(a)
        self.assertIn('已完成',j['reply'])
    def test_repeated_model_tool_calls_share_one_sdk_submission(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),('visit_point',{'point_id':'welcome'}),'已完成'])
        original=a.nav.visit;calls=[]
        def submit(*args,**kwargs):calls.append(1);return original(*args,**kwargs)
        a.nav.visit=submit;a.submit('r','c','去接待起点');j=self.wait(a)
        self.assertEqual(calls,[1]);self.assertEqual(self.backend.submissions,1)
        self.assertEqual(j['task']['state'],'accepted')
