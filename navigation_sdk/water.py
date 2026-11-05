"""Documented WATER TCP client. No implicit reconnect/retry of mutations."""
from collections import deque
import socket
import threading
import time
import uuid
from .errors import NavigationError
from .protocol import JSONStream, command, yaw

class WaterClient:
    def __init__(self, host, port=31001, *, timeout=10., terminator=''):
        if terminator not in ('','\n','\r\n'):raise ValueError('Choose bare, LF or CRLF explicitly')
        self.host=host;self.port=port;self.timeout=timeout;self.terminator=terminator
        self.socket=None;self.stream=JSONStream();self.lock=threading.RLock();self.events=deque(maxlen=128)
    def close(self):
        with self.lock:
            if self.socket:self.socket.close()
            self.socket=None;self.stream=JSONStream()
    def request(self, path, **params):
        with self.lock:
            ident=uuid.uuid4().hex;params={**params,'uuid':ident};sent=False
            try:
                if self.socket is None:
                    self.socket=socket.create_connection((self.host,self.port),self.timeout)
                self.socket.settimeout(self.timeout)
                sent=True;self.socket.sendall((command(path,params)+self.terminator).encode())
                deadline=time.monotonic()+self.timeout
                while time.monotonic()<deadline:
                    self.socket.settimeout(max(.01,deadline-time.monotonic()))
                    data=self.socket.recv(65536)
                    if not data:
                        self.stream.feed(b'',final=True);raise OSError('EOF')
                    response=None
                    for obj in self.stream.feed(data):
                        if obj.get('type')=='response' and obj.get('uuid')==ident and obj.get('command')==path:
                            response=obj
                        else:self.events.append(obj)
                    if response is not None:
                        if response.get('status')!='OK':
                            raise NavigationError('WATER_'+str(response.get('status','UNKNOWN_ERROR')),
                                                  response.get('error_message') or 'WATER rejected command',details={'response':response})
                        return response
                raise TimeoutError()
            except (OSError,TimeoutError):
                self.close()
                raise NavigationError('UPSTREAM_UNCERTAIN' if sent and path in ('/api/move','/api/move/cancel') or sent and '/insert' in path or sent and '/delete' in path else 'UPSTREAM_UNAVAILABLE',
                                      'TCP request outcome unavailable; no mutation was replayed',status=503) from None
            except NavigationError as exc:
                if exc.code in ('INVALID_RESPONSE','RESPONSE_TOO_LARGE'):
                    self.close()
                    raise NavigationError('UPSTREAM_UNCERTAIN','Malformed reply; outcome unknown and not replayed',status=503) from None
                raise
    def move(self, *, marker=None, x=None, y=None, theta=0.):
        return self.request('/api/move',**({'marker':marker} if marker is not None else {'location':f'{x},{y},{theta}'}))
    def status(self):return self.request('/api/robot_status').get('results',{})
    def cancel(self):return self.request('/api/move/cancel')
    def markers(self):return self.request('/api/markers/query_list').get('results') or {}

class WaterBackend:
    name='water'
    def __init__(self, host, port=31001, *, timeout=10., terminator=''):
        self.client=WaterClient(host,port,timeout=timeout,terminator=terminator)
    def capabilities(self):
        return {'backend':'water','hardware':True,'api_version':'1.0',
                'features':{'move_pose':True,'move_marker':True,'marker_crud':True,'global_cancel':True,
                            'saved_routes':False,'visit_introduction':False,'conditional_task_control':False},
                'task_correlation':'requires task_id in status; legacy v1.8.7 lacks this guarantee',
                'verification':'loopback protocol fixtures, not colleague firmware','units':{'position':'m','theta':'rad'}}
    def status(self):
        s=self.client.status();return {'backend':'water','pose':s.get('current_pose'),'floor':s.get('current_floor'),
            'map_id':None,'robot_id':None,'navigation_state':s.get('move_status','unknown'),
            'battery_percent':s.get('power_percent'),'charging':s.get('charge_state'),
            'hardware_estop':s.get('hard_estop_state'),'software_estop':s.get('soft_estop_state'),
            'vendor_status':s,'unknown_fields':[k for k in ('power_percent','charge_state','hard_estop_state','soft_estop_state') if k not in s]}
    def maps(self):raise NavigationError('UNSUPPORTED_CAPABILITY','Map enumeration not implemented in this WATER profile')
    def routes(self):raise NavigationError('UNSUPPORTED_CAPABILITY','Use native WATER multi-goal API or a P2 saved route; no silent emulation')
    def markers(self):
        return [{'id':name,'name':p.get('marker_name',name),'x':p['pose']['position']['x'],
                 'y':p['pose']['position']['y'],'theta':yaw(p['pose']['orientation']),
                 'floor':p.get('floor'),'marker_type':p.get('key'),'map_id':None} for name,p in self.client.markers().items()]
    def submit(self, request):
        if request['kind']!='move':raise NavigationError('UNSUPPORTED_CAPABILITY','Only move tasks are implemented by this WATER provider')
        t=request['target']
        if 'marker' not in t and t.get('theta') is None:
            raise NavigationError('UNSUPPORTED_CAPABILITY','WATER location requires an explicit theta; position-only semantics are not assumed')
        response=self.client.move(**t)
        return {'task_id':response.get('task_id'),'target':t}
    def observe(self, ref):
        s=self.client.status()
        if not ref.get('task_id') or not s.get('task_id'):
            return {'state':'unknown','reason':'legacy_status_has_no_task_id','vendor_state':s.get('move_status'),
                    'note':'Use raw WaterClient status under the application\'s existing ownership contract'}
        if s['task_id']!=ref['task_id']:return {'state':'interrupted','reason':'task_replaced'}
        return {'state':{'running':'running','succeeded':'succeeded','failed':'failed','canceled':'cancelled'}.get(s.get('move_status'),'unknown')}
    def control(self, ref, operation):
        raise NavigationError('UNSUPPORTED_CAPABILITY','Legacy WATER cancel is global; use stop() explicitly')
    def stop(self):self.client.cancel();return {'stop_requested':True,'scope':'current_vendor_navigation'}
    def save_marker(self, data):
        if set(data)-{'name','x','y','theta'}:raise NavigationError('UNSUPPORTED_CAPABILITY','WATER markers do not store P2 presentation fields')
        params=dict(data)
        path='/api/markers/insert_by_pose' if any(k in params for k in ('x','y','theta')) else '/api/markers/insert'
        if path.endswith('insert_by_pose') and not all(k in params for k in ('x','y','theta')):
            raise NavigationError('INVALID_ARGUMENT','Explicit WATER pose needs x,y,theta',status=400)
        self.client.request(path,**params)
        return next((p for p in self.markers() if p['name']==data['name']),{'name':data['name'],'write_accepted':True,'readback':False})
    def delete_marker(self,name):self.client.request('/api/markers/delete',name=name);return {'deleted':True,'name':name}
