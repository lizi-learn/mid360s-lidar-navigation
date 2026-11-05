"""Agent -> SDK HTTP -> P2 adapter -> actual Session/Mission engines.

Goal completion and audio are injected receipts. This does not simulate walking,
Nav2 geometry, sensors or real obstacle clearance.
"""
import copy
from pathlib import Path
import sys,tempfile,threading,time,unittest
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'navigation'))
from reception_catalog import blank,validate
from reception_mission import Mission,BUSY
from session import NavigationSession
import sdk_control
from navigation_agent.app import NavigationAgent
from navigation_sdk import NavigationClient,NavigationError
from navigation_sdk.p2 import P2Backend
from navigation_sdk.store import TaskStore
from navigation_sdk.gateway import GatewayServer
from test_agent import ScriptedModel
EVIDENCE=[]

class EngineProvider(P2Backend):
    def __init__(self):
        self.now=1000.;self.mode='navigation_allowed';self.sequence=0;self.posts=[];self.events=[]
        self.session=NavigationSession();self.engine=Mission(self.events.append);self.map_id='synthetic_20'
        self.doc=blank(self.map_id);self.doc['revision']=1
        self.doc['points']=[dict(id='p'+str(n),name='点位 '+str(n),x=float(n),y=0.,yaw=.2 if n%2 else None,
            product_id='',script='合成介绍'+str(n)if n in(1,5)else '',dwell=.3 if n==15 else 0.,guest_wait=False,kind='visit')for n in range(1,21)]
        self.doc['routes']=[dict(id=rid,name=name,mode='showcase',loops=1,description='',steps=[dict(point_id='p'+str(n),speak=True)for n in nums])for rid,name,nums in [('forward','20点顺序挑战 · 1至20',range(1,21)),('reverse','20点逆序挑战 · 20至1',range(20,0,-1))]]
        self.doc=validate(self.doc);self.operator('navigation_allowed')
        self.node=NS(lock=threading.RLock(),session=self.session)
        self.node.reception=NS(engine=self.engine,map_id=lambda:self.map_id,command=self.command)
        self.node.stop=self.stop_native
    def operator(self,mode):
        self.mode=mode;self.sequence+=1
        self.session.operator(mode,True,True,False,1,self.sequence,self.now,self.now)
        self.engine.tick(self.session.status(self.now),self.now)
    def _extension(self):return sdk_control.version()
    def _document(self):return copy.deepcopy(self.doc)
    def raw_status(self):
        self.operator(self.mode)
        return dict(request_token='synthetic-only',robot_name='synthetic-only',pose=[0,0,0],gate={'data_fresh':True,'localized':True},
          session=self.session.status(self.now),reception={'map_id':self.map_id,'mission':self.engine.status()},maps=[self.map_id])
    def capabilities(self):
        c=super().capabilities();c.update(hardware=False,verification='actual state engines with synthetic goal/audio receipts only');return c
    def _post(self,path,body):
        self.posts.append((path,copy.deepcopy(body)))
        try:
            if path=='/api/confirm':return sdk_control.mission(self.node,body)
            if path=='/api/stop':return sdk_control.stop(self.node,body)
            raise ValueError('unsupported test path')
        except ValueError as exc:raise NavigationError('UPSTREAM_REJECTED',str(exc),status=409)from None
    def stop_native(self):
        if self.engine.phase in BUSY:self.engine.cancel()
        g=self.session.cancel();self.session.acknowledge_cancel(g)
    def command(self,data):
        op=data['command']
        if op=='start':
            if not self.session.status(self.now)['navigation_permitted']:raise ValueError('manual priority')
            d=copy.deepcopy(self.doc);rid=data.get('route_id')
            if data.get('point_ids'):
                from relative_intent import visit_sequence
                rid=visit_sequence(d,data['point_ids'])
            if data.get('point_id'):
                rid='single';d['routes'].append(dict(id=rid,name='单点',mode='showcase',loops=1,description='',steps=[dict(point_id=data['point_id'],speak=True)]))
            self.engine.start(d,rid,self.now)
        elif op=='pause':self.engine.cancel('paused');self.stop_native()
        elif op=='resume':
            if not self.session.status(self.now)['navigation_permitted']:raise ValueError('manual priority')
            self.engine.resume(self.now)
        elif op=='continue':self.engine.continue_visit(self.now)
        else:raise ValueError('unsupported control')
        return self.engine.status()
    def leg(self):
        command=self.engine.tick(self.session.status(self.now),self.now)
        if not command:return False
        p=command['point'];g=self.session.prepare(dict(x=p['x'],y=p['y'],yaw=p['yaw']or 0.),self.now)
        self.session.planned(g,2,self.now);self.session.start(g,self.now,True,True);self.session.started(g,self.now)
        self.engine.dispatched(command['epoch'],g,command['end_index'])
        self.now+=.02;self.operator(self.mode)
        self.session.finished(g,True,action_status=4);self.engine.tick(self.session.status(self.now),self.now)
        return True
    def drain(self):
        for _ in range(200):
            if self.engine.phase=='completed':return
            if self.engine.phase=='dispatching':self.leg()
            elif self.engine.phase=='presenting':
                self.engine.tick(self.session.status(self.now),self.now,{'epoch':self.engine.epoch,'key':self.engine.speech_key,'result':'completed'})
            elif self.engine.phase=='dwell':
                self.now+=.31;self.operator(self.mode)
            else:raise AssertionError('unexpected drain phase '+self.engine.phase)
        raise AssertionError('bounded engine sequence did not complete')

class MissionIntegration(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.backend=EngineProvider();self.store=TaskStore(self.backend)
        self.server=GatewayServer(('127.0.0.1',0),self.store)
        threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.02),daemon=True).start()
        self.nav=NavigationClient('http://127.0.0.1:'+str(self.server.server_port),timeout=3);self.agents=[]
    def tearDown(self):
        for a in self.agents:a.executor.shutdown(wait=True);a.journal.db.close()
        self.server.shutdown();self.server.server_close();self.store.close();self.tmp.cleanup()
    def agent(self,script=()):
        a=NavigationAgent(self.nav,Path(self.tmp.name)/('agent'+str(len(self.agents))+'.sqlite3'),ScriptedModel(script=list(script)));self.agents.append(a);return a
    def request(self,a,text,ident='r',conversation='c'):
        a.submit(ident,conversation,text);end=time.monotonic()+8
        while time.monotonic()<end:
            j=a.journal.job(ident)
            if j['state']not in('working','queued'):return j
            time.sleep(.01)
        self.fail('agent request timeout')
    def test_empty_paused_session_starts_without_a_phantom_cancel(self):
        self.backend.operator('paused');self.backend.operator('navigation_allowed')
        a=self.agent([('control_navigation',{'operation':'cancel'}),('visit_point',{'point_id':'p1'}),'已开始'])
        j=self.request(a,'去点位1')
        self.assertEqual(j['task']['state'],'accepted')
        self.assertEqual([p for p,b in self.backend.posts],['/api/confirm'])
    def test_two_complete_twenty_point_orders_keep_arrival_and_speech_receipts(self):
        a=self.agent(['可以','可以'])
        for i,(text,order)in enumerate([('按20到1的路线参观',list(range(20,0,-1))),('按1到20的路线参观',list(range(1,21)))]):
            j=self.request(a,text,str(i));self.assertEqual(j['task']['state'],'accepted');start=len(self.backend.events)
            self.backend.drain();t=self.nav.task(j['task']['id']);self.assertEqual(t['state'],'succeeded')
            events=self.backend.events[start:]
            self.assertEqual([e['point']for e in events if e['kind']=='arrived'],['p'+str(n)for n in order])
            self.assertEqual(sum(e['kind']=='speech_completed'for e in events),2)
            EVIDENCE.append({'scenario':text,'arrived':order,'speech_completed':2,'task':'succeeded','native_events':len(events),'initial_dispatches':1,'injected_receipts':True})
        self.assertEqual([p for p,b in self.backend.posts],['/api/confirm','/api/confirm'])
    def test_guest_wait_requires_continue_and_then_completes(self):
        p=self.backend.doc['points'][2];p['guest_wait']=True
        a=self.agent(['提交','继续']);j=self.request(a,'去点位3')
        self.backend.leg();self.assertEqual(self.nav.task(j['task']['id'])['state'],'waiting')
        self.backend.engine.tick(self.backend.session.status(self.backend.now),self.backend.now+1)
        self.assertEqual(self.backend.engine.phase,'guest_wait')
        self.request(a,'继续下一站','continue');self.assertEqual(self.nav.task(j['task']['id'])['state'],'succeeded')
    def test_manual_takeover_then_left_does_not_resurrect_task(self):
        a=self.agent(['提交','继续']);j=self.request(a,'去点位4');self.backend.leg()
        # Use a new in-flight leg rather than a completed single point.
        j=self.request(a,'按20到1的路线参观','route')
        self.backend.operator('manual');self.assertEqual(self.nav.task(j['task']['id'])['state'],'cancelled')
        count=len(self.backend.posts);self.backend.operator('navigation_allowed')
        self.request(a,'继续导航','continue');self.assertEqual(len(self.backend.posts),count)
        self.assertEqual(self.backend.engine.phase,'manual')
    def test_operator_goes_manual_before_dispatch_and_no_start_is_sent(self):
        self.backend.operator('manual');a=self.agent(['已经到达'])
        j=self.request(a,'去点位1')
        self.assertEqual(self.backend.posts,[]);self.assertNotIn('已经到达',j['reply'])
    def test_late_old_reference_cannot_stop_a_new_run(self):
        old=self.nav.visit('p4');self.backend.drain();new=self.nav.visit('p2')
        count=len(self.backend.posts)
        with self.assertRaises(NavigationError):self.backend.control(old['native_ref'],'cancel')
        self.assertEqual(len(self.backend.posts),count);self.assertEqual(self.nav.task(new['id'])['state'],'accepted')
    def test_named_products_complete_speech_and_wait_in_requested_order(self):
        self.backend.doc['points'][0].update(name='A产品',yaw=.7,dwell=.3)
        self.backend.doc['points'][4].update(name='B产品',yaw=-1.)
        original=copy.deepcopy(self.backend.doc);a=self.agent(['提交'])
        j=self.request(a,'去介绍A产品，B产品');self.assertEqual(j['task']['request']['kind'],'tour')
        self.assertEqual([s['point']['yaw']for s in self.backend.engine.steps],[.7,-1.])
        self.backend.drain();self.assertEqual(self.nav.task(j['task']['id'])['state'],'succeeded')
        self.assertEqual([e['point']for e in self.backend.events if e['kind']=='arrived'],['p1','p5'])
        self.assertEqual(sum(e['kind']=='speech_completed'for e in self.backend.events),2)
        self.assertEqual(self.backend.doc,original)
        self.assertEqual([p for p,b in self.backend.posts],['/api/confirm'])
        EVIDENCE.append(dict(scenario='A产品→B产品',arrived=['p1','p5'],speech_completed=2,task='succeeded',injected_receipts=True,initial_dispatches=1))
