"""Deterministic contract fixture. No clock-based arrival or physical motion."""
import copy
import threading
import uuid
from .errors import NavigationError
from .validation import finite, text

class MockBackend:
    name = 'mock'
    def __init__(self):
        self.lock = threading.RLock()
        self.points = [{'id':'welcome','name':'接待起点','x':0.,'y':0.,'theta':0.,'map_id':'synthetic',
                        'script':'欢迎参观。','dwell':2.,'guest_wait':False,'kind':'visit'},
                       {'id':'display','name':'产品展示','x':4.,'y':2.,'theta':1.57,'map_id':'synthetic',
                        'script':'这里是示例产品。','dwell':3.,'guest_wait':True,'kind':'visit'}]
        self.current = None
        self.pose = {'x':0.,'y':0.,'theta':0.,'frame':'map'}
        self.submissions = 0
    def capabilities(self):
        return {'backend':'mock','api_version':'1.0','hardware':False,'verification':'synthetic contract fixture only',
                'features':{k:True for k in ('move_pose','move_marker','marker_crud','saved_routes','visit_introduction','conditional_task_control','global_cancel','relative_forward','relative_rotation','visit_sequence')},
                'units':{'position':'m','theta':'rad'}}
    def status(self):
        with self.lock:
            return {'backend':'mock','robot_id':'synthetic-only','map_id':'synthetic','pose':dict(self.pose),
                    'navigation_state':self.current['state'] if self.current else 'idle',
                    'navigation_permitted':True,'operator_mode':'simulation','battery_percent':None,
                    'charging':None,'hardware_estop':None,'unknown_fields':['battery_percent','charging','hardware_estop']}
    def maps(self): return [{'id':'synthetic','active':True}]
    def markers(self):
        with self.lock: return copy.deepcopy(self.points)
    def routes(self): return [{'id':'sample-tour','name':'合成接待路线','steps':[{'point_id':'welcome'},{'point_id':'display'}]}]
    def submit(self, request):
        with self.lock:
            if self.current and self.current['state'] not in ('succeeded','failed','cancelled','interrupted'):
                raise NavigationError('TASK_BUSY','Synthetic task already active')
            if request['kind'] == 'route' and request['route_id'] != 'sample-tour':
                raise NavigationError('ROUTE_NOT_FOUND','Unknown synthetic route',status=404)
            marker = request.get('marker') or request.get('target',{}).get('marker')
            if marker and not any(marker in (p['id'],p['name']) for p in self.points):
                raise NavigationError('MARKER_NOT_FOUND','Unknown synthetic marker',status=404)
            if request['kind']=='tour'and any(not any(m in(p['id'],p['name'])for p in self.points)for m in request['markers']):
                raise NavigationError('MARKER_NOT_FOUND','Unknown synthetic tour marker',status=404)
            ident = uuid.uuid4().hex
            self.current = {'id':ident,'state':'accepted','arrival_reached':False,'request':copy.deepcopy(request)}
            self.submissions += 1
            return {'id':ident,'type':request['kind']}
    def observe(self, ref):
        with self.lock:
            if not self.current or self.current['id'] != ref['id']:
                return {'state':'interrupted','reason':'task_replaced'}
            return {k:v for k,v in self.current.items() if k != 'request'}
    def control(self, ref, operation):
        with self.lock:
            if self.observe(ref)['state'] in ('succeeded','failed','cancelled','interrupted'):
                raise NavigationError('TASK_TERMINAL','Task no longer active')
            if operation == 'cancel': self.current['state'] = 'cancelled'
            elif operation == 'pause': self.current['state'] = 'paused'
            elif operation in ('resume','continue'): self.current['state'] = 'running'
            return ref
    def stop(self):
        with self.lock:
            if self.current and self.current['state'] not in ('succeeded','failed','interrupted'):
                self.current['state'] = 'cancelled'
            return {'stop_requested':True,'scope':'synthetic_navigation_only'}
    def advance(self, state):
        if state not in ('running','blocked','presenting','waiting','dwelling','succeeded','failed','cancelled','interrupted'):
            raise NavigationError('INVALID_ARGUMENT','Unknown synthetic state',status=400)
        with self.lock:
            if not self.current: raise NavigationError('TASK_NOT_FOUND','No synthetic task',status=404)
            if self.current['state'] in ('succeeded','failed','cancelled','interrupted'):
                raise NavigationError('TASK_TERMINAL','Synthetic terminal state is immutable')
            self.current['state'] = state
            self.current['arrival_reached'] = state in ('presenting','waiting','dwelling','succeeded')
            return self.observe({'id':self.current['id']})
    def save_marker(self, data):
        name=text(data.get('name'),'name')
        if set(data)-{'name','x','y','theta','script','dwell','guest_wait','kind'}:
            raise NavigationError('INVALID_ARGUMENT','Unsupported marker fields',status=400)
        with self.lock:
            p=next((p for p in self.points if p['name']==name),None)
            if p is None:
                p={'id':uuid.uuid4().hex,'name':name,'map_id':'synthetic',**self.pose};p.pop('frame',None);self.points.append(p)
            for k,v in data.items():p[k]=finite(v,k) if k in ('x','y','theta','dwell') and v is not None else v
            return dict(p)
    def delete_marker(self, name):
        with self.lock:
            p=next((p for p in self.points if name in (p['id'],p['name'])),None)
            if p is None:raise NavigationError('MARKER_NOT_FOUND','Unknown marker',status=404)
            if p['id'] in ('welcome','display'):raise NavigationError('MARKER_IN_USE','Marker belongs to sample-tour')
            self.points.remove(p);return {'deleted':True,'name':p['name']}
