"""WATER command subset over the unified task registry. LF/CRLF framing only."""
import json
import math
import select
import socketserver
import threading
import time
from .errors import NavigationError
from .protocol import parse_command, quaternion
from .validation import finite, task_request

MOVE_STATE={'dispatching':'running','accepted':'running','running':'running','blocked':'running',
            'presenting':'running','waiting':'running','dwelling':'running','paused':'running',
            'succeeded':'succeeded','failed':'failed','cancelled':'canceled','interrupted':'canceled','unknown':None}
class WaterFacade:
    def __init__(self, store):self.store=store;self.backend=store.backend;self.lock=threading.RLock();self.current=None
    def status(self):
        s=self.backend.status()
        with self.lock:t=self.store.get(self.current) if self.current else None
        marker=t['request'].get('target',{}).get('marker','') if t else ''
        state=t['state'] if t else 'idle'
        return {'move_target':marker,'move_status':MOVE_STATE.get(state,'idle'),
                'running_status':'running' if state in ('accepted','running','blocked','dispatching','paused','presenting','waiting','dwelling') else 'idle',
                'current_pose':s.get('pose'),'current_floor':s.get('floor'),
                'power_percent':s.get('battery_percent'),'charge_state':s.get('charging'),
                'hard_estop_state':s.get('hardware_estop'),'soft_estop_state':s.get('software_estop'),
                'estop_state':None,'task_id':t['id'] if t else None,
                'extensions':{'profile':'navbridge-water-lf/1','state':state,'backend':s['backend'],
                              'unknown_fields':s.get('unknown_fields',[]),
                              'navigation_permitted':s.get('navigation_permitted')}}
    def execute(self, line):
        path='';params={}
        try:
            path,params=parse_command(line);ident=params.pop('uuid','')
            response={'type':'response','command':path,'uuid':ident,'status':'OK','error_message':''}
            allowed={
                '/api/move':{'marker','location'}, '/api/move/cancel':set(), '/api/robot_status':set(),
                '/api/markers/query_list':set(), '/api/markers/query_brief':set(), '/api/markers/count':set(),
                '/api/markers/insert':{'name'}, '/api/markers/insert_by_pose':{'name','x','y','theta'},
                '/api/markers/delete':{'name'}, '/api/robot_info':set(), '/api/software/get_version':set(),
                '/api/request_data':{'topic','frequency'}}
            if path not in allowed:raise NavigationError('UNSUPPORTED_CAPABILITY','Command not implemented by this profile')
            if set(params)-allowed[path]:raise NavigationError('UNSUPPORTED_CAPABILITY','Unsupported parameters: '+','.join(sorted(set(params)-allowed[path])))
            if path=='/api/move':
                if 'location' in params:
                    parts=params['location'].split(',')
                    if len(parts)!=3:raise NavigationError('INVALID_ARGUMENT','location=x,y,theta',status=400)
                    target={k:float(v) for k,v in zip(('x','y','theta'),parts)}
                elif 'marker' in params:target={'marker':params['marker']}
                else:raise NavigationError('INVALID_ARGUMENT','marker or location required',status=400)
                t=self.store.submit({'kind':'move','target':target})
                with self.lock:self.current=t['id']
                response['task_id']=t['id']
            elif path=='/api/move/cancel':self.store.stop()
            elif path=='/api/robot_status':response['results']=self.status()
            elif path=='/api/request_data':
                if params.get('topic')!='robot_status':raise NavigationError('UNSUPPORTED_CAPABILITY','Only robot_status subscription implemented')
                frequency=float(params.get('frequency','2'))
                if not math.isfinite(frequency) or not 0<=frequency<=2:raise NavigationError('INVALID_ARGUMENT','Profile frequency range 0..2 Hz',status=400)
                response['subscription']={'topic':'robot_status','frequency':frequency}
            elif path=='/api/software/get_version':response['results']='navbridge/0.1.0; profile=navbridge-water-lf/1'
            elif path=='/api/robot_info':response['results']={'product_id':'navbridge-'+self.backend.name,'vendor_emulation':False}
            elif path.startswith('/api/markers/'):
                if path.endswith('insert') or path.endswith('insert_by_pose'):
                    if 'name' not in params:raise NavigationError('INVALID_ARGUMENT','name required',status=400)
                    data={'name':params['name']}
                    if path.endswith('insert_by_pose'):
                        if set(params)!={'name','x','y','theta'}:raise NavigationError('INVALID_ARGUMENT','name,x,y,theta required',status=400)
                        data.update({k:finite(float(params[k]),k) for k in ('x','y','theta')})
                        task_request({'kind':'move','target':{k:data[k]for k in ('x','y','theta')}})
                    else:
                        pose=self.backend.status().get('pose')
                        if not isinstance(pose,dict) or any(pose.get(k) is None for k in ('x','y','theta')):
                            raise NavigationError('POSE_UNAVAILABLE','Current measured pose is unavailable')
                        data.update({k:finite(pose[k],k) for k in ('x','y','theta')})
                    self.backend.save_marker(data)
                elif path.endswith('delete'):
                    if set(params)!={'name'}:raise NavigationError('INVALID_ARGUMENT','name required',status=400)
                    self.backend.delete_marker(params['name'])
                else:
                    points=self.backend.markers()
                    if path.endswith('count'):response['results']={'count':len(points)}
                    elif path.endswith('query_brief'):
                        # A floor string cannot be truthfully generated when floor is unknown.
                        raise NavigationError('UNSUPPORTED_CAPABILITY','P2 has no configured WATER floor/type brief mapping')
                    else:
                        response['results']={p['name']:{'marker_name':p['name'],'floor':p.get('floor'),'key':p.get('marker_type',0),
                          'pose':{'position':{'x':p['x'],'y':p['y'],'z':0.},'orientation':quaternion(p.get('theta') or 0.)},
                          'extensions':{'map_id':p.get('map_id'),'heading_unspecified':p.get('theta') is None}}
                          for p in points} or None
            return response
        except (NavigationError,ValueError,UnicodeError) as exc:
            code=getattr(exc,'code','INVALID_ARGUMENT')
            return {'type':'response','command':path,'uuid':params.get('uuid',locals().get('ident','')),
                    'status':'INVALID_REQUEST' if code in ('INVALID_ARGUMENT','MARKER_NOT_FOUND') else 'REQUEST_DENIED',
                    'error_message':str(exc),'extensions':{'error_code':code}}

class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        facade=self.server.facade;buffer=b'';frequency=0.;next_poll=0.;last_event=None
        self.request.settimeout(2.)
        def send(value):self.request.sendall(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode()+b'\n')
        try:
            while not self.server.stopping.is_set():
                readable,_,_=select.select([self.request],[],[],.1)
                if readable:
                    data=self.request.recv(4096)
                    if not data:return
                    buffer+=data
                    if len(buffer)>65536:return
                    while b'\n' in buffer:
                        raw,buffer=buffer.split(b'\n',1)
                        # Requests are delimited explicitly. EOF/idle does not execute a fragment.
                        response=facade.execute(raw.rstrip(b'\r').decode('utf-8','strict'));send(response)
                        if response.get('status')=='OK' and 'subscription'in response:
                            frequency=response['subscription']['frequency'];next_poll=0.
                if frequency and time.monotonic()>=next_poll:
                    next_poll=time.monotonic()+1/frequency
                    try:status=facade.status()
                    except NavigationError as exc:
                        send({'type':'notification','code':'NAVBRIDGE_BACKEND_UNAVAILABLE','level':'warning','description':str(exc)});continue
                    send({'type':'callback','topic':'robot_status','results':status})
                    event=(status['task_id'],status['move_status'])
                    if event!=last_event and event[0]:
                        code={'running':'01001','succeeded':'01002','failed':'01003','canceled':'01004'}.get(event[1])
                        if code:send({'type':'notification','code':code,'level':'info','description':'Navigation state changed',
                                      'data':{'target':status['move_target']},'extensions':{'task_id':event[0]}})
                    last_event=event
        except (OSError,UnicodeError,ValueError):return

class WaterServer(socketserver.ThreadingTCPServer):
    allow_reuse_address=True;daemon_threads=True
    def __init__(self,address,store):
        self.facade=WaterFacade(store);self.stopping=threading.Event();super().__init__(address,_Handler)
    def server_close(self):self.stopping.set();super().server_close()
