"""Actual upstream RAI graph against the SDK HTTP synthetic backend; no ROS/hardware."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from typing import Any
from langchain_core.language_models import BaseChatModel
from langchain_core.messages import AIMessage
from langchain_core.outputs import ChatResult,ChatGeneration
from pydantic import PrivateAttr
from navigation_agent.app import NavigationAgent,Journal
from navigation_sdk import NavigationClient
from navigation_sdk.gateway import GatewayServer
from navigation_sdk.mock import MockBackend
from navigation_sdk.store import TaskStore

class ScriptedModel(BaseChatModel):
    script:list[Any]
    _index:int=PrivateAttr(default=0)
    _entered:Any=PrivateAttr(default=None)
    _release:Any=PrivateAttr(default=None)
    @property
    def _llm_type(self):return 'scripted-fixture'
    def bind_tools(self,tools,**kwargs):return self
    def _generate(self,messages,stop=None,run_manager=None,**kwargs):
        if self._entered:
            self._entered.set();self._release.wait(8)
        entry=self.script[self._index];self._index+=1
        message=AIMessage(content=entry)if isinstance(entry,str)else AIMessage(content='',tool_calls=[dict(name=entry[0],args=entry[1],id='call-'+str(self._index),type='tool_call')])
        return ChatResult(generations=[ChatGeneration(message=message)])

class AgentTest(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.backend=MockBackend();self.store=TaskStore(self.backend)
        self.server=GatewayServer(('127.0.0.1',0),self.store)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.nav=NavigationClient('http://127.0.0.1:'+str(self.server.server_port),timeout=3)
        self.agents=[]
    def tearDown(self):
        for a in self.agents:a.executor.shutdown(wait=True);a.journal.db.close()
        self.server.shutdown();self.server.server_close();self.store.close();self.tmp.cleanup()
    def agent(self,script):
        a=NavigationAgent(self.nav,Path(self.tmp.name)/('agent'+str(len(self.agents))+'.sqlite3'),ScriptedModel(script=script))
        self.agents.append(a);return a
    def wait(self,a,ident='request-1'):
        end=time.monotonic()+10
        while time.monotonic()<end:
            job=a.journal.job(ident)
            if job['state']not in('queued','working'):return job
            time.sleep(.02)
        self.fail('Actual RAI graph did not finish')
    def test_actual_graph_visit_is_accepted_not_falsely_arrived(self):
        a=self.agent([('list_destinations',{}),('visit_point',{'point_id':'welcome'}),'已经到达'])
        a.submit('request-1','test','去接待起点');job=self.wait(a)
        self.assertEqual(job['state'],'done');self.assertEqual(self.backend.submissions,1)
        self.assertEqual(job['task']['state'],'accepted');self.assertNotIn('已经到达',job['reply'])
        self.assertEqual([t['name']for t in job['tools']],['list_destinations','visit_point'])
    def test_duplicate_request_survives_journal_reopen_without_dispatch(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已提交'])
        a.submit('request-1','test','去接待起点');original=self.wait(a)
        self.assertEqual(a.submit('request-1','test','去接待起点')['task']['id'],original['task']['id'])
        reopened=Journal(Path(self.tmp.name)/'agent0.sqlite3')
        _,new=reopened.reserve('request-1','test','去接待起点',10);reopened.db.close()
        self.assertFalse(new);self.assertEqual(self.backend.submissions,1)
        with self.assertRaises(ValueError):a.submit('request-1','test','改成另一个目的地')
    def test_stop_during_model_call_prevents_late_dispatch(self):
        a=self.agent([('visit_point',{'point_id':'welcome'}),'已处理'])
        entered=threading.Event();release=threading.Event();a.llm._entered=entered;a.llm._release=release
        a.submit('request-1','test','去接待起点');self.assertTrue(entered.wait(3))
        begin=time.monotonic();a.stop();self.assertLess(time.monotonic()-begin,1)
        release.set();job=self.wait(a)
        self.assertEqual(self.backend.submissions,0)
        self.assertIn('已被停止',json.dumps(job,ensure_ascii=False))
    def test_unknown_point_id_never_generates_motion(self):
        a=self.agent([('visit_point',{'point_id':'not-real'}),'目录没有这个点位'])
        a.submit('request-1','test','去一个不存在的位置');job=self.wait(a)
        self.assertEqual(self.backend.submissions,0);self.assertIn('不存在',job['tools'][0]['result']['error'])
    def test_route_and_cancel_use_existing_sdk_task_lifecycle(self):
        a=self.agent([('run_route',{'route_id':'sample-tour'}),'已提交'])
        a.submit('request-1','test','参观一遍');job=self.wait(a)
        self.assertEqual(job['task']['request']['kind'],'route')
        a.submit('request-stop','test','停止导航');self.assertEqual(a.journal.job('request-stop')['state'],'done')
        self.assertEqual(self.backend.current['state'],'cancelled');self.assertEqual(self.backend.submissions,1)
    def test_graph_status_and_pause_resume_continue(self):
        task=self.nav.visit('接待起点',idempotency_key='fixture')
        a=self.agent([('control_navigation',{'operation':'pause'}),('navigation_status',{}),'已暂停'])
        a.journal.save_conversation('test',dict(history=[],task_id=task['id']))
        a.submit('request-1','test','暂停');job=self.wait(a)
        self.assertEqual(self.backend.current['state'],'paused')
        self.assertEqual(job['tools'][1]['result']['task']['state'],'paused')
        b=self.agent([('control_navigation',{'operation':'resume'}),'已恢复'])
        b.journal.save_conversation('test',dict(history=[],task_id=task['id']))
        b.submit('request-1','test','继续导航');self.wait(b)
        self.assertEqual(self.backend.current['state'],'running')
    def test_restart_never_replays_working_request(self):
        p=Path(self.tmp.name)/'crash.sqlite3';j=Journal(p)
        job,_=j.reserve('working','test','去接待起点',0);job['state']='working';j.put(job);j.db.close()
        j=Journal(p);self.assertEqual(j.job('working')['state'],'unknown');j.db.close()
        self.assertEqual(self.backend.submissions,0)

if __name__=='__main__':unittest.main()
