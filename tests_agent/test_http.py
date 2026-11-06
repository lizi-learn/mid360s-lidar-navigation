"""Exercise the real Agent HTTP handler against the isolated SDK fixture."""
import json,tempfile,threading,time,unittest
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError
from navigation_agent.app import NavigationAgent
from navigation_agent.server import AgentServer
from navigation_sdk import NavigationClient
from navigation_sdk.gateway import GatewayServer
from navigation_sdk.mock import MockBackend
from navigation_sdk.store import TaskStore
from test_agent import ScriptedModel

class HTTPContract(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.backend=MockBackend();self.store=TaskStore(self.backend)
        self.sdk=GatewayServer(('127.0.0.1',0),self.store);self.start(self.sdk)
        self.agent=NavigationAgent(NavigationClient('http://127.0.0.1:'+str(self.sdk.server_port)),Path(self.tmp.name)/'agent.sqlite3',ScriptedModel(script=['已提交']))
        self.web=AgentServer(('127.0.0.1',0),self.agent,'offline-scripted');self.start(self.web)
        self.base='http://127.0.0.1:'+str(self.web.server_port)
    def start(self,server):threading.Thread(target=lambda:server.serve_forever(poll_interval=.02),daemon=True).start()
    def tearDown(self):
        self.web.shutdown();self.web.server_close();self.agent.executor.shutdown(wait=True);self.agent.journal.db.close()
        self.sdk.shutdown();self.sdk.server_close();self.store.close();self.tmp.cleanup()
    def request(self,path,body=None,token=True,origin=None):
        headers={}
        if token:headers['X-Agent-Token']=self.web.token
        if origin:headers['Origin']=origin
        data=None if body is None else json.dumps(body).encode()
        if data is not None:headers['Content-Type']='application/json'
        try:
            with urlopen(Request(self.base+path,data=data,headers=headers),timeout=4)as r:return r.status,json.load(r)
        except HTTPError as e:return e.code,json.load(e)
    def wait(self,ident):
        end=time.monotonic()+6
        while time.monotonic()<end:
            _,j=self.request('/api/agent/jobs/'+ident)
            if j['state']not in('working','queued'):return j
            time.sleep(.01)
        self.fail('HTTP job timed out')
    def test_same_request_id_returns_same_task_without_new_dispatch(self):
        body={'id':'same','conversation':'c','text':'去接待起点'}
        self.assertEqual(self.request('/api/agent/chat',body)[0],202);first=self.wait('same')
        self.assertEqual(self.request('/api/agent/chat',body)[1]['task']['id'],first['task']['id'])
        self.assertEqual(self.backend.submissions,1)
    def test_same_id_with_different_command_is_rejected(self):
        self.request('/api/agent/chat',{'id':'same','conversation':'c','text':'去接待起点'});self.wait('same')
        self.assertEqual(self.request('/api/agent/chat',{'id':'same','conversation':'c','text':'去产品展示'})[0],400)
        self.assertEqual(self.backend.submissions,1)
    def test_missing_token_does_not_queue_any_action(self):
        status,_=self.request('/api/agent/chat',{'id':'x','conversation':'c','text':'去接待起点'},token=False)
        self.assertEqual(status,403);self.assertEqual(self.backend.submissions,0)
        self.assertEqual(self.agent.journal.db.execute('select count(*)from jobs').fetchone()[0],0)
    def test_foreign_origin_cannot_control_even_with_a_valid_token(self):
        status,_=self.request('/api/agent/stop',{},origin='http://untrusted.invalid')
        self.assertEqual(status,403);self.assertEqual(self.agent.epoch,0)
    def test_invalid_fields_and_nonstring_commands_have_no_side_effect(self):
        for body in ({'id':'x','conversation':'c','text':123},{'id':'x','conversation':'c','text':'去接待起点','goal':{}},{'id':'x','conversation':'c','text':'x'*2001}):
            with self.subTest(body=str(body)[:60]):self.assertEqual(self.request('/api/agent/chat',body)[0],400)
        self.assertEqual(self.backend.submissions,0)
    def test_stop_request_is_direct_and_prevents_late_model_action(self):
        entered=threading.Event();release=threading.Event();self.agent.llm._entered=entered;self.agent.llm._release=release
        self.request('/api/agent/chat',{'id':'x','conversation':'c','text':'去接待起点'});self.assertTrue(entered.wait(2))
        start=time.monotonic();status,_=self.request('/api/agent/stop',{});elapsed=time.monotonic()-start
        self.assertEqual(status,200);self.assertLess(elapsed,.5)
        release.set();self.wait('x');self.assertEqual(self.backend.submissions,0)
    def test_live_page_and_status_reads_do_not_invoke_the_model(self):
        self.assertEqual(self.request('/api/agent/status?conversation=c')[0],200)
        self.assertEqual(self.agent.llm._index,0);self.assertEqual(self.backend.submissions,0)
