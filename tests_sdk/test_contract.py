import copy
import json
import math
from pathlib import Path
import socket
import socketserver
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from navigation_sdk import NavigationClient, NavigationError
from navigation_sdk.cloud_signing import signature
from navigation_sdk.compat import WaterServer, WaterFacade
from navigation_sdk.gateway import GatewayServer
from navigation_sdk.http import JsonHTTP
from navigation_sdk.mock import MockBackend
from navigation_sdk.p2 import P2Backend
from navigation_sdk.protocol import JSONStream, command, parse_command, quaternion, yaw
from navigation_sdk.store import TaskStore
from navigation_sdk.validation import task_request
from navigation_sdk.water import WaterClient, WaterBackend

class Running:
    def __init__(self, server):self.server=server
    def __enter__(self):
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start();return self.server
    def __exit__(self,*args):self.server.shutdown();self.server.server_close();self.thread.join(3)

class Protocol(unittest.TestCase):
    def test_json_every_byte_split_and_utf8(self):
        objects=[{'type':'response','uuid':'u','text':'中文 {"} 😊'},{'type':'callback','results':{}},{'type':'notification','code':'01002'}]
        for separator in ('','\n','\r\n'):
            raw=separator.join(json.dumps(o,ensure_ascii=False)for o in objects).encode()
            for cut in range(len(raw)+1):
                s=JSONStream();self.assertEqual(s.feed(raw[:cut])+s.feed(raw[cut:],final=True),objects)
    def test_json_single_byte_chunks(self):
        raw=json.dumps({'中文':'\"{}\n','type':'response'},ensure_ascii=False).encode();stream=JSONStream();out=[]
        for c in raw:out+=stream.feed(bytes([c]))
        self.assertEqual(out,[json.loads(raw)])
    def test_bad_utf8_truncation_and_bounds(self):
        for data in (b'[]',b'{"x":',b'\xff'):
            with self.assertRaises(NavigationError):JSONStream().feed(data,final=True)
        with self.assertRaises(NavigationError):JSONStream(3).feed(b'{"x":1}')
    def test_url_encoding_and_duplicate_rejection(self):
        name='中文 &? +,/%';p,params=parse_command(command('/api/move',{'marker':name}))
        self.assertEqual(params['marker'],name)
        with self.assertRaises(NavigationError):parse_command('/api/move?marker=A&marker=B')
    def test_quaternion_sign_and_wrap(self):
        for theta in (-math.pi,-1.,0.,2.9,math.pi):
            q=quaternion(theta)
            for qq in (q,{k:-v for k,v in q.items()}):self.assertAlmostEqual(math.remainder(yaw(qq)-theta,2*math.pi),0.)
    def test_cloud_official_vector(self):
        p={'accessKeyId':'testid','uidKey':'0802','signatureNonce':'53c593e7-766d-4646-8b58-0b795ded0ed6','timestamp':'2019-10-10T08:26:01Z'}
        self.assertEqual(signature(p,'testsecret'),'VQAdXELyv7rl/6E4bj1VdV4X6vI=')
        self.assertEqual(signature({**p,'signature':'ignored','blank':''},'testsecret'),signature(p,'testsecret'))
        with self.assertRaises(ValueError):signature({'nested':{}},'x')
        self.assertNotEqual(signature({**p,'accessKeyId':'testId'},'testsecret'),signature(p,'testsecret'))
    def test_strict_targets(self):
        for payload in ({'kind':'move','target':{'x':True,'y':0}}, {'kind':'move','target':{'x':float('nan'),'y':0}},
                        {'kind':'move','target':{'x':0,'y':0,'theta':180}}, {'kind':'move','target':{'marker':'a','x':0}},
                        {'kind':'move','target':{'marker':' '}}, {'kind':'visit','marker':'a','junk':1}):
            with self.assertRaises(NavigationError):task_request(payload)

class HTTP(unittest.TestCase):
    def setUp(self):
        self.backend=MockBackend();self.store=TaskStore(self.backend)
        self.server=GatewayServer(('127.0.0.1',0),self.store)
        self.running=Running(self.server);self.running.__enter__();self.addCleanup(self.running.__exit__)
        self.addCleanup(self.store.close);self.url='http://127.0.0.1:'+str(self.server.server_port);self.client=NavigationClient(self.url)
    def advance(self,state):JsonHTTP(self.url).request('POST','/mock/advance',{'state':state})
    def test_accepted_not_arrived_and_real_terminal(self):
        t=self.client.move(marker='产品展示',idempotency_key='order-1')
        self.assertEqual(t['state'],'accepted');self.assertEqual(self.client.task(t['id'])['state'],'accepted')
        self.advance('running');self.assertEqual(self.client.task(t['id'])['state'],'running')
        self.advance('succeeded');self.assertEqual(self.client.wait(t['id'],timeout=.1)['state'],'succeeded')
    def test_presentation_wait_dwell_and_continue(self):
        t=self.client.visit('产品展示')
        for state in ('running','presenting','waiting'):
            self.advance(state);self.assertEqual(self.client.task(t['id'])['state'],state)
        self.assertEqual(self.client.continue_visit(t['id'])['state'],'running')
        self.advance('dwelling');self.advance('succeeded');self.assertEqual(self.client.task(t['id'])['state'],'succeeded')
    def test_cancel_pause_resume_stop(self):
        t=self.client.route('sample-tour');self.assertEqual(self.client.pause(t['id'])['state'],'paused')
        self.assertEqual(self.client.resume(t['id'])['state'],'running')
        self.assertEqual(self.client.cancel(t['id'])['state'],'cancelled')
        t2=self.client.move(x=1,y=0);self.client.stop();self.assertEqual(self.client.task(t2['id'])['state'],'cancelled')
    def test_wait_timeout_does_not_cancel(self):
        t=self.client.move(x=1,y=0)
        with self.assertRaises(NavigationError)as e:self.client.wait(t['id'],timeout=.01,interval=.01)
        self.assertEqual(e.exception.code,'WAIT_TIMEOUT');self.assertEqual(self.client.task(t['id'])['state'],'accepted')
    def test_idempotency_parallel_single_dispatch_and_conflict(self):
        with ThreadPoolExecutor(8)as ex:
            results=list(ex.map(lambda _:self.client.move(x=1,y=0,idempotency_key='same'),range(12)))
        self.assertEqual(len({t['id']for t in results}),1);self.assertEqual(self.backend.submissions,1)
        with self.assertRaises(NavigationError)as e:self.client.move(x=2,y=0,idempotency_key='same')
        self.assertEqual(e.exception.code,'IDEMPOTENCY_CONFLICT')
    def test_marker_crud_and_null_capabilities(self):
        p=self.client.save_marker({'name':'测试 &?','x':1.,'y':2.,'theta':.3})
        self.assertEqual(next(x for x in self.client.markers()if x['name']==p['name'])['x'],1.)
        self.client.delete_marker(p['name']);self.assertFalse(any(x['name']==p['name']for x in self.client.markers()))
        self.assertIsNone(self.client.status()['battery_percent']);self.assertFalse(self.client.capabilities()['hardware'])
    def test_old_cancel_does_not_cancel_replacement(self):
        t=self.client.move(x=1,y=0);self.advance('succeeded');self.client.task(t['id'])
        newer=self.client.move(x=2,y=0)
        with self.assertRaises(NavigationError):self.client.cancel(t['id'])
        self.assertEqual(self.client.task(newer['id'])['state'],'accepted')
    def test_interruption_is_not_success_and_no_restart(self):
        t=self.client.move(x=1,y=0);self.advance('interrupted')
        self.assertEqual(self.client.task(t['id'])['state'],'interrupted')
    def test_auth_and_origin(self):
        self.server.api_key='local-test'
        with self.assertRaises(NavigationError)as e:self.client.status()
        self.assertEqual(e.exception.status,401)
        c=NavigationClient(self.url,api_key='local-test');self.assertEqual(c.status()['backend'],'mock')
        with self.assertRaises(NavigationError):c.http.request('GET','/v1/status',headers={'Origin':'https://foreign.invalid'})
    def test_invalid_json_and_unknown_route(self):
        with self.assertRaises(NavigationError):self.client.route('missing')
        with self.assertRaises(NavigationError):self.client.http.request('POST','/v1/tasks',{'kind':'move','target':{'x':1,'y':0},'hidden':True})

class Journal(unittest.TestCase):
    def test_restart_does_not_redispatch(self):
        with tempfile.TemporaryDirectory()as d:
            backend=MockBackend();p=Path(d)/'tasks.sqlite';s=TaskStore(backend,p)
            first=s.submit({'kind':'move','target':{'x':1,'y':0}},'one');s.close()
            s=TaskStore(backend,p)
            try:
                same=s.submit({'kind':'move','target':{'x':1,'y':0}},'one')
                self.assertEqual(same['id'],first['id']);self.assertEqual(same['state'],'unknown');self.assertEqual(backend.submissions,1)
            finally:s.close()
    def test_lost_dispatch_reply_not_replayed(self):
        backend=MockBackend();calls=[]
        def lost(_):calls.append(1);raise NavigationError('UPSTREAM_UNCERTAIN','lost',status=503)
        backend.submit=lost;s=TaskStore(backend)
        try:
            with self.assertRaises(NavigationError):s.submit({'kind':'move','target':{'x':1,'y':0}},'lost')
            self.assertEqual(s.submit({'kind':'move','target':{'x':1,'y':0}},'lost')['state'],'unknown')
            self.assertEqual(len(calls),1)
        finally:s.close()
    def test_poll_failure_preserves_last_state_with_stale_flag(self):
        b=MockBackend();s=TaskStore(b);self.addCleanup(s.close);t=s.submit({'kind':'move','target':{'x':1,'y':0}})
        def offline(_):raise NavigationError('UPSTREAM_UNAVAILABLE','offline',status=503)
        b.observe=offline;observed=s.get(t['id']);self.assertEqual(observed['state'],'accepted');self.assertTrue(observed['observation_stale'])

class TCP(unittest.TestCase):
    def setUp(self):
        self.backend=MockBackend();self.store=TaskStore(self.backend);self.addCleanup(self.store.close)
        self.server=WaterServer(('127.0.0.1',0),self.store);self.running=Running(self.server);self.running.__enter__();self.addCleanup(self.running.__exit__)
        self.port=self.server.server_address[1]
    def client(self):
        c=WaterClient('127.0.0.1',self.port,timeout=2.,terminator='\n');self.addCleanup(c.close);return c
    def test_tcp_move_query_and_cancel(self):
        c=self.client();response=c.move(marker='产品展示');self.assertEqual(response['status'],'OK')
        self.assertEqual(c.status()['move_status'],'running');self.assertEqual(c.status()['task_id'],response['task_id'])
        c.cancel();self.assertEqual(c.status()['move_status'],'canceled')
    def test_fragmented_request_never_executes_before_delimiter(self):
        with socket.create_connection(('127.0.0.1',self.port))as s:
            raw='/api/move?marker=产品展示&uuid=a'.encode()
            for byte in raw:s.sendall(bytes([byte]))
            time.sleep(.1);self.assertEqual(self.backend.submissions,0)
            s.sendall(b'\r\n');response=JSONStream().feed(s.recv(65536))[0]
            self.assertEqual(response['uuid'],'a');self.assertEqual(self.backend.submissions,1)
    def test_eof_fragment_does_not_dispatch(self):
        with socket.create_connection(('127.0.0.1',self.port))as s:s.sendall(b'/api/move?location=1,0,0')
        time.sleep(.05);self.assertEqual(self.backend.submissions,0)
    def test_sticky_commands_in_one_write(self):
        with socket.create_connection(('127.0.0.1',self.port))as s:
            s.sendall(b'/api/robot_status?uuid=a\n/api/software/get_version?uuid=b\n');s.settimeout(2);out=[];stream=JSONStream()
            while len(out)<2:out+=stream.feed(s.recv(65536))
            self.assertEqual([o['uuid']for o in out],['a','b'])
    def test_subscription_and_notification(self):
        c=self.client();c.request('/api/request_data',topic='robot_status',frequency=2)
        t=c.move(marker='产品展示');time.sleep(.6);c.status()
        self.assertTrue(any(o['type']=='callback'for o in c.events))
        self.backend.advance('succeeded');time.sleep(.6);c.status()
        self.assertTrue(any(o.get('code')=='01002'for o in c.events))
    def test_unsupported_not_silently_ignored(self):
        c=self.client()
        for path,params in (('/api/joy_control',{'linear_velocity':.3}),('/api/estop',{'flag':True}),
                            ('/api/move',{'markers':'A,B','count':1}),('/api/move',{'marker':'产品展示','distance_tolerance':.5})):
            with self.assertRaises(NavigationError):c.request(path,**params)
        self.assertEqual(self.backend.submissions,0)
    def test_insert_same_name_updates_to_actual_current_pose(self):
        c=self.client();c.request('/api/markers/insert_by_pose',name='同名',x=9,y=8,theta=.7)
        self.backend.pose.update(x=2.,y=3.,theta=.2)
        c.request('/api/markers/insert',name='同名')
        p=c.markers()['同名'];self.assertEqual(p['pose']['position']['x'],2.)
        self.assertAlmostEqual(yaw(p['pose']['orientation']),.2)

    def test_water_provider_uses_wire_and_matches_task_id_extension(self):
        b=WaterBackend('127.0.0.1',self.port,terminator='\n');self.addCleanup(b.client.close)
        ref=b.submit({'kind':'move','target':{'marker':'产品展示'}})
        self.assertEqual(b.observe(ref)['state'],'running');self.backend.advance('succeeded');self.assertEqual(b.observe(ref)['state'],'succeeded')
    def test_legacy_missing_task_id_does_not_invent_success(self):
        b=WaterBackend('127.0.0.1',self.port,terminator='\n');b.client.status=lambda:{'move_status':'succeeded'}
        self.assertEqual(b.observe({'task_id':'abc'})['state'],'unknown')
    def test_raw_marker_shape_and_upsert(self):
        c=self.client();c.request('/api/markers/insert_by_pose',name='中文新点',x=1,y=2,theta=.4)
        p=c.markers()['中文新点'];self.assertEqual(p['marker_name'],'中文新点');self.assertEqual(p['key'],0);self.assertIsNone(p['floor'])
        self.assertAlmostEqual(yaw(p['pose']['orientation']),.4)
        c.request('/api/markers/delete',name='中文新点');self.assertNotIn('中文新点',c.markers())

class P2Fixture(BaseHTTPRequestHandler):
    def log_message(self,*_):pass
    def send(self,obj,status=200):
        raw=json.dumps(obj,ensure_ascii=False).encode();self.send_response(status);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
    def do_GET(self):
        if self.path=='/api/status?summary=1':return self.send(self.server.state)
        if self.path=='/api/reception':return self.send({'document':self.server.document})
        if self.path=='/api/sdk/version':return self.send({'conditional_task_control':True,'relative_forward':True,'relative_rotation':True,'visit_sequence':True,'conditional_relative_chaining':True} if self.server.extension else {'error':'missing'},200 if self.server.extension else 404)
        self.send({},404)
    def do_POST(self):
        body=json.loads(self.rfile.read(int(self.headers['Content-Length'])));s=self.server.state;m=s['reception']['mission'];ss=s['session']
        if self.headers.get('X-Nav-Token')!='ephemeral-test':return self.send({'error':'token'},403)
        self.server.posts.append((self.path,body))
        expected=body.get('_sdk_expected')
        if expected and expected!={'generation':ss['generation'],'run_id':m['run_id'],'epoch':m['epoch'],'map_id':s['reception']['map_id']}:return self.send({'error':'changed'},409)
        if self.path in('/api/goal','/api/relative-goal','/api/rotate'):
            if '_sdk_expected_phase'in body and body['_sdk_expected_phase']!=ss['phase']:return self.send({'error':'phase changed'},409)
            if self.path=='/api/goal'and body.get('_sdk_expected_generation')!=ss['generation']:return self.send({'error':'changed'},409)
            if self.path=='/api/relative-goal':
                x,y,yaw=s['pose'];ss['goal']={'x':x+body['meters']*math.cos(yaw),'y':y+body['meters']*math.sin(yaw),'yaw':yaw}
            elif self.path=='/api/rotate':ss['goal']={'x':s['pose'][0],'y':s['pose'][1],'yaw':math.remainder(s['pose'][2]+body['radians'],math.tau)}
            else:ss['goal']={k:body[k]for k in('x','y','yaw')}
            ss['generation']+=1;ss['phase']='ready';return self.send({'generation':ss['generation']})
        if self.path=='/api/confirm':
            if 'reception_command'in body:
                command=body['reception_command']['command']
                if command=='start':m.update(run_id='new-run',epoch=m['epoch']+1,phase='navigating')
                elif command=='pause':m['phase']='paused';m['epoch']+=1
                elif command in ('resume','continue'):m['phase']='navigating';m['epoch']+=1
                return self.send(m)
            if body['generation']!=ss['generation']:return self.send({'error':'changed'},409)
            ss['phase']='executing';return self.send({'started':True})
        if self.path=='/api/stop':ss['phase']='cancelled';m['phase']='cancelled';return self.send({'stopped':True})
        if self.path=='/api/save':
            reception=body['reception']
            if reception['expected_revision']!=self.server.document['revision']:return self.send({'error':'revision'},409)
            self.server.document=reception['document'];self.server.document['revision']+=1;return self.send(self.server.document)
        self.send({},404)

class P2(unittest.TestCase):
    def setUp(self):
        self.server=ThreadingHTTPServer(('127.0.0.1',0),P2Fixture);self.server.posts=[];self.server.extension=True
        self.server.state={'request_token':'ephemeral-test','robot_name':'test','pose':[1.,2.,.3],
            'gate':{'data_fresh':True,'localized':True},'session':{'phase':'idle','generation':0,'navigation_permitted':True,'operator_mode':'navigation_allowed'},
            'reception':{'map_id':'example','mission':{'phase':'idle','run_id':None,'epoch':0}},'maps':['example']}
        self.server.document={'map_id':'example','revision':1,'points':[{'id':'p','name':'接待','x':3.,'y':4.,'yaw':.5,'script':'你好','dwell':3.,'guest_wait':True}],
                              'routes':[{'id':'r','steps':[{'point_id':'p','speak':True}]}],'edges':[],'policy':{},'doors':[],'annotations':[]}
        running=Running(self.server);running.__enter__();self.addCleanup(running.__exit__)
        self.backend=P2Backend('http://127.0.0.1:'+str(self.server.server_port))
    def test_manual_priority_does_not_prepare_or_dispatch_a_goal(self):
        self.server.state['session'].update(navigation_permitted=False,operator_mode='manual',notice='E右档手动优先')
        with self.assertRaises(NavigationError)as exc:self.backend.submit({'kind':'visit','marker':'接待'})
        self.assertEqual(exc.exception.code,'MANUAL_PRIORITY')
        self.assertEqual(self.server.posts,[])
    def test_empty_paused_switch_state_is_idle_and_accepts_visit_without_stop(self):
        self.server.state['session'].update(phase='paused',goal=None,cancel_pending=False)
        status=self.backend.status()
        self.assertEqual(status['navigation_state'],'idle')
        self.assertFalse(status['task_active']);self.assertIsNone(status['current_task'])
        ref=self.backend.submit({'kind':'visit','marker':'接待'})
        self.assertEqual(ref['type'],'mission')
        self.assertEqual([path for path,body in self.server.posts],['/api/confirm'])
        self.assertEqual(self.backend.status()['current_task']['run_id'],ref['run_id'])
        self.assertEqual(self.backend.status()['navigation_state'],'running')
    def test_real_paused_goal_and_cancel_ack_pending_are_still_busy(self):
        for changes in (
            dict(phase='paused',goal={'x':1,'y':2,'yaw':0},last_stop={'phase':'paused'},terminal_receipt=None),
            dict(phase='paused',goal=None,cancel_pending=True),
            dict(phase='executing',goal={'x':1,'y':2,'yaw':0},cancel_pending=False),
        ):
            with self.subTest(changes=changes):
                self.server.state['session'].update(changes)
                with self.assertRaises(NavigationError)as exc:self.backend.submit({'kind':'visit','marker':'接待'})
                self.assertEqual(exc.exception.code,'TASK_BUSY')
                self.assertTrue(exc.exception.details['current_task'])
                self.assertEqual(self.server.posts,[])
    def test_retained_terminal_goal_does_not_become_busy_on_switch_pause(self):
        for changes in (
            dict(terminal_receipt={'succeeded':True},last_stop=None),
            dict(terminal_receipt=None,last_stop={'phase':'manual'}),
            dict(terminal_receipt=None,last_stop={'phase':'cancelled'}),
        ):
            with self.subTest(changes=changes):
                self.server.state['session'].update(phase='paused',goal={'x':1,'y':2,'yaw':0},**changes)
                self.assertFalse(self.backend.status()['task_active'])
                self.assertEqual(self.backend.status()['navigation_state'],'idle')
    def test_paused_mission_still_owns_task_even_when_session_has_no_goal(self):
        self.server.state['session'].update(phase='paused',goal=None)
        self.server.state['reception']['mission'].update(phase='paused',run_id='existing-tour')
        self.assertTrue(self.backend.status()['task_active'])
        with self.assertRaises(NavigationError)as exc:self.backend.submit({'kind':'visit','marker':'接待'})
        self.assertEqual(exc.exception.details['current_task']['run_id'],'existing-tour')
        self.assertEqual(self.server.posts,[])
    def test_navigation_move_not_double_speech(self):
        ref=self.backend.submit(task_request({'kind':'move','target':{'marker':'接待'}}))
        self.assertEqual(ref['type'],'navigation');self.assertEqual([p for p,b in self.server.posts],['/api/goal','/api/confirm'])
        self.assertEqual(self.backend.observe(ref)['state'],'running')
        self.server.state['session']['phase']='arrived';self.assertEqual(self.backend.observe(ref)['state'],'succeeded')
    def test_visit_and_route_full_workflow(self):
        ref=self.backend.submit({'kind':'visit','marker':'接待'});self.assertEqual(ref['type'],'mission')
        self.assertEqual(self.server.posts[-1][1]['reception_command']['point_id'],'p')
        self.assertEqual(self.backend.control(ref,'pause')['epoch'],2)
        self.server.state['reception']['mission']['phase']='completed';self.assertEqual(self.backend.observe(ref)['state'],'succeeded')
    def test_no_unknown_battery_estop_faked(self):
        s=self.backend.status();self.assertIsNone(s['battery_percent']);self.assertIsNone(s['hardware_estop'])
    def test_old_reference_cancel_rejected(self):
        ref=self.backend.submit({'kind':'route','route_id':'r'});self.server.state['reception']['mission']['run_id']='app-replacement'
        with self.assertRaises(NavigationError):self.backend.control(ref,'cancel')
        self.assertNotIn('/api/stop',[p for p,b in self.server.posts])
    def test_map_switch_interrupts(self):
        ref=self.backend.submit({'kind':'route','route_id':'r'});self.server.state['reception']['map_id']='other'
        self.assertEqual(self.backend.observe(ref)['state'],'interrupted')
    def test_map_change_between_snapshot_and_dispatch_never_starts_old_coordinates(self):
        original=self.backend._post
        def change_map(path,body):
            if path=='/api/goal':self.server.state['reception']['map_id']='new-map'
            return original(path,body)
        self.backend._post=change_map
        with self.assertRaises(NavigationError):self.backend.submit({'kind':'move','target':{'marker':'接待'}})
        self.assertNotIn('/api/confirm',[path for path,body in self.server.posts])
        self.assertEqual(self.server.state['session']['phase'],'idle')
    def test_catalog_upsert_preserves_routes_introductions(self):
        routes=copy.deepcopy(self.server.document['routes']);self.backend.save_marker({'name':'接待','x':5.})
        self.assertEqual(self.server.document['routes'],routes);self.assertEqual(self.server.document['points'][0]['script'],'你好')
        self.assertEqual(self.server.document['points'][0]['x'],5.)
        with self.assertRaises(NavigationError)as e:self.backend.delete_marker('接待')
        self.assertEqual(e.exception.code,'MARKER_IN_USE')
    def test_old_console_truthfully_read_only_until_extension_installed(self):
        self.server.extension=False;self.assertFalse(self.backend.capabilities()['features']['move_pose'])
        self.assertEqual(self.backend.status()['robot_id'],'test')
        with self.assertRaises(NavigationError):self.backend.submit({'kind':'move','target':{'x':1,'y':2,'theta':None}})
        self.assertEqual(self.server.posts,[])
    def test_busy_not_overwritten(self):
        self.server.state['reception']['mission']['phase']='presenting'
        with self.assertRaises(NavigationError):self.backend.submit({'kind':'route','route_id':'r'})
        self.assertEqual(self.server.posts,[])


class Correlation(unittest.TestCase):
    def run_server(self, handler):
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),handler);server.daemon_threads=True
        run=Running(server);run.__enter__();self.addCleanup(run.__exit__)
        return server.server_address[1]
    def test_callback_and_wrong_uuid_never_complete_request(self):
        class Wire(socketserver.StreamRequestHandler):
            def handle(self):
                path,p=parse_command(self.rfile.readline().decode().strip())
                objects=[{'type':'callback','topic':'robot_status','results':{'move_status':'succeeded'}},
                         {'type':'response','command':path,'uuid':'wrong','status':'OK','results':{'wrong':True}},
                         {'type':'notification','code':'01002','description':'其他任务'},
                         {'type':'response','command':path,'uuid':p['uuid'],'status':'OK','results':{'correct':True}}]
                raw=''.join(json.dumps(o,ensure_ascii=False)for o in objects).encode()
                for byte in raw:self.request.sendall(bytes([byte]))
        c=WaterClient('127.0.0.1',self.run_server(Wire),terminator='\n',timeout=2);self.addCleanup(c.close)
        response=c.request('/api/robot_status');self.assertEqual(response['results'],{'correct':True})
        self.assertEqual(len(c.events),3)
    def test_truncated_mutation_reply_is_uncertain_not_replayed(self):
        calls=[]
        class Wire(socketserver.StreamRequestHandler):
            def handle(self):calls.append(self.rfile.readline());self.request.sendall(b'{"type":"response"')
        c=WaterClient('127.0.0.1',self.run_server(Wire),terminator='\n',timeout=2);self.addCleanup(c.close)
        with self.assertRaises(NavigationError)as exc:c.move(marker='x')
        self.assertEqual(exc.exception.code,'UPSTREAM_UNCERTAIN');self.assertEqual(len(calls),1)

class HTTPOutcome(unittest.TestCase):
    def server(self):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*_):pass
            def do_GET(self):
                self.server.calls.append(self.path)
                self.send_response(302);self.send_header('Location','/must-not-follow');self.end_headers()
            def do_POST(self):
                self.server.calls.append(self.path)
                self.send_response(500);self.end_headers();self.wfile.write(b'{"error":"failure after dispatch"}')
        s=ThreadingHTTPServer(('127.0.0.1',0),Handler);s.calls=[]
        run=Running(s);run.__enter__();self.addCleanup(run.__exit__)
        return s,JsonHTTP('http://127.0.0.1:'+str(s.server_port),api_key='test-only')
    def test_redirect_is_not_followed(self):
        s,c=self.server()
        with self.assertRaises(NavigationError)as exc:c.request('GET','/redirect')
        self.assertEqual(exc.exception.code,'UPSTREAM_REDIRECT');self.assertEqual(s.calls,['/redirect'])
    def test_http_500_mutation_outcome_is_unknown(self):
        s,c=self.server()
        with self.assertRaises(NavigationError)as exc:c.request('POST','/dispatch',{})
        self.assertEqual(exc.exception.code,'UPSTREAM_UNCERTAIN');self.assertEqual(s.calls,['/dispatch'])

if __name__=='__main__':unittest.main()
