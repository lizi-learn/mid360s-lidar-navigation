"""Ordered reception task engine and bounded console adapter.

One explicit Start owns a snapshot. Manual input/cancel invalidates that run;
returning E to left never resumes it. No motor commands or velocity synthesis.
"""
import copy
import math
import threading
import secrets
import time
from concurrent.futures import ThreadPoolExecutor
from reception_catalog import introduction, content_key
from planning_requests import RouteReplyTimeout

def route_poses(points, start=None):
    """Seed unspecified orientations with the incoming route direction.

    These headings constrain the lattice search, not arrival actions. Explicit
    point headings remain authoritative; position-only final goals stay so.
    Missing start poses and coincident points use the next nonzero segment,
    then the last usable heading. Saved catalog points are never changed.
    """
    previous=start
    heading=start[2] if start is not None else 0.
    result=[]
    for i,point in enumerate(points):
        x,y=point['x'],point['y']
        if point['yaw'] is not None:
            heading=point['yaw']
        elif previous is not None and math.hypot(x-previous[0],y-previous[1])>1e-6:
            heading=math.atan2(y-previous[1],x-previous[0])
        else:
            for following in points[i+1:]:
                dx,dy=following['x']-x,following['y']-y
                if math.hypot(dx,dy)>1e-6:
                    heading=math.atan2(dy,dx);break
        result.append(dict(x=x,y=y,yaw=heading))
        previous=(x,y,heading)
    return result

BUSY = {'dispatching', 'navigating', 'presenting', 'guest_wait', 'dwell', 'rerouting', 'waiting_route'}

class Mission:
    def __init__(self,event_sink=None):
        self.event_sink=event_sink
        # Keep command/receipt IDs exact through JSON and JavaScript Number.
        self.lock = threading.RLock(); self.epoch = secrets.randbits(40)*256; self.reset()
    def reset(self):
        self.run_id='';self.paused_from=None;self.pause_remaining=0.
        self.phase = 'idle'; self.steps = []; self.index = 0; self.document = None
        self.route_id = ''; self.route_mode='showcase';self.actual_transit=[];self.waiting_since=None;self.route_reason=''
        self.route_name = ''; self.error = ''; self.goal_generation = None; self.batch_end = None
        self.presentation_started = None; self.deadline = None; self.speech_key = ''; self.events = []; self.event_seq = 0
    def event(self, kind, **data):
        self.event_seq += 1
        self.events.append({'sequence': self.event_seq, 'kind': kind, **data})
        self.events = self.events[-100:]
        if self.event_sink and self.run_id:
            point_id=data.get('point');point=next((p for p in self.document['points']if p['id']==point_id),{})
            self.event_sink(dict(self.events[-1],point_name=point.get('name',''),run_id=self.run_id,map_id=self.document['map_id'],route_name=self.route_name,wall_time=time.time(),epoch=self.epoch))
    def start(self, document, route_id, now):
        with self.lock:
            if self.phase in BUSY: raise ValueError('当前任务正在执行，请先取消或暂停')
            route = next((r for r in document['routes'] if r['id'] == route_id), None)
            if route is None: raise ValueError('路线不存在')
            if not route['steps']: raise ValueError('这条路线还没有点位，请先加入点位再开始巡游')
            self.reset(); self.run_id=secrets.token_hex(12);self.epoch += 1; self.document = copy.deepcopy(document)
            points = {p['id']: p for p in document['points']}
            def make_step(s):
                point=copy.deepcopy(points[s['point_id']])
                if s.get('_transit'):point.update(kind='transit',yaw=None,script='',product_id='',dwell=0.,guest_wait=False)
                return dict(point=point,speak=s['speak'])
            self.steps = [make_step(s)
                          for _ in range(route['loops']) for s in route['steps']]
            self.route_id=route_id;self.route_mode=route['mode']
            self.route_name = route['name']; self.phase = 'dispatching'
            self.event('started', at=now, route=route_id, revision=document['revision'])
            return self.epoch
    def valid(self, epoch):
        with self.lock: return epoch == self.epoch and self.phase in BUSY
    def cancel(self, reason='cancelled'):
        with self.lock:
            if self.phase==reason:return self.epoch
            if reason=='paused' and self.phase in BUSY:
                self.paused_from=self.phase;self.pause_remaining=max(0.,(self.deadline or time.monotonic())-time.monotonic())
            self.epoch += 1; self.phase = reason; self.speech_key = ''
            self.event(reason); return self.epoch
    def resume(self,now):
        with self.lock:
            if self.phase!='paused' or not self.document or not self.paused_from:raise ValueError('当前没有可继续的暂停任务')
            previous=self.paused_from;self.epoch+=1;self.error=''
            if previous in ('dispatching','navigating','rerouting','waiting_route'):
                self.phase='dispatching'
            elif previous=='presenting':
                self.phase='presenting';self.presentation_started=now
                self.speech_key=content_key(introduction(self.document,self.steps[self.index]['point']))
            elif previous=='dwell':self.phase='dwell';self.deadline=now+self.pause_remaining
            else:self.phase=previous
            self.paused_from=None;self.event('resumed',previous_phase=previous)
            return self.epoch
    def batch(self):
        """Only action-free transit points can be crossed without a stop."""
        end=self.index
        while end+1<len(self.steps):
            step=self.steps[end];point=step['point']
            if (point['kind']!='transit' or point['yaw'] is not None or point['dwell'] or point['guest_wait'] or
                    step['speak'] and introduction(self.document,point)):break
            end+=1
        return end,[copy.deepcopy(s['point'])for s in self.steps[self.index:end+1]]
    def dispatched(self, epoch, generation, end_index=None,transit_ids=None):
        with self.lock:
            if epoch != self.epoch or self.phase not in ('dispatching','rerouting'): return False
            self.actual_transit=list(transit_ids)if transit_ids is not None else [s['point']['id']for s in self.steps[self.index:(self.index if end_index is None else end_index)]]
            self.waiting_since=None
            self.goal_generation = generation; self.batch_end = self.index if end_index is None else end_index; self.phase = 'navigating'
            self.event('leg_started', point=self.steps[self.batch_end]['point']['id'],
                       via=self.actual_transit)
            return True
    def begin_reroute(self,reason):
        with self.lock:
            if self.phase not in ('navigating','waiting_route'):return False
            self.phase='rerouting';self.route_reason=reason;self.event('route_replanning',reason=reason)
            return True
    def wait_route(self,epoch,now,reason):
        with self.lock:
            if epoch!=self.epoch or self.phase!='rerouting':return
            self.phase='waiting_route';self.route_reason=reason
            if self.waiting_since is None:self.waiting_since=now
            self.event('route_waiting',reason=reason)
    def failed(self, epoch, error):
        with self.lock:
            if epoch != self.epoch or self.phase not in BUSY: return
            self.error = str(error); self.phase = 'failed'; self.speech_key = ''
            self.event('failed', error=self.error)
    def _after_speech(self, now):
        p = self.steps[self.index]['point']; self.speech_key = ''
        if p['guest_wait']: self.phase = 'guest_wait'; self.event('guest_wait')
        elif p['dwell'] > 0: self.phase = 'dwell'; self.deadline = now+p['dwell']
        else: self._next(now)
    def _next(self, now):
        self.index += 1; self.goal_generation = None; self.batch_end = None
        if self.index >= len(self.steps): self.phase = 'completed'; self.event('completed', at=now)
        else: self.phase = 'dispatching'
    def continue_visit(self, now):
        with self.lock:
            if self.phase != 'guest_wait': raise ValueError('当前没有等待访客继续的讲解点')
            self._next(now)
    def skip(self, now):
        with self.lock:
            if self.phase not in ('guest_wait', 'presenting', 'dwell'): raise ValueError('仅能跳过当前已到达点位的讲解')
            self.event('presentation_skipped'); self._next(now)
    def tick(self, session, now, speech_result=None):
        with self.lock:
            if self.phase not in BUSY: return None
            # Stay sensitive to takeover while parked for a presentation too.
            if not session.get('navigation_permitted', False):
                self.cancel('manual' if session.get('operator_mode') == 'manual' else 'paused'); return None
            if self.phase=='waiting_route' and self.waiting_since is not None and now-self.waiting_since>=self.document['policy']['max_wait']:
                self.failed(self.epoch,'通道持续受阻，已结束本次导览');return None
            if self.phase == 'dispatching':
                end,points=self.batch()
                return {'epoch': self.epoch, 'point': points[-1], 'points':points, 'end_index':end}
            if self.phase == 'navigating':
                if session.get('generation') != self.goal_generation:
                    self.failed(self.epoch, '导航目标已被其他操作替换'); return None
                phase = session.get('phase')
                if phase in ('blocked', 'cancelled', 'stopped', 'manual', 'paused'):
                    self.failed(self.epoch, session.get('notice', phase)); return None
                if phase == 'arrived':
                    if self.batch_end is not None:
                        for point_id in self.actual_transit:
                            self.event('transit_segment_completed',point=point_id,receipt='navigate_through_poses_succeeded')
                        self.index=self.batch_end
                    p = self.steps[self.index]['point']; script = introduction(self.document, p) if self.steps[self.index]['speak'] else ''
                    self.event('arrived', point=p['id'], at=now)
                    if script:
                        # The content cache key can repeat at another stop or loop.
                        # A new presentation must have its own playback receipt.
                        self.epoch += 1
                        self.phase = 'presenting'; self.presentation_started = now; self.speech_key = content_key(script)
                        self.event('presentation', key=self.speech_key, text=script, point=p['name'])
                    else: self._after_speech(now)
            elif self.phase == 'presenting':
                if (speech_result and speech_result.get('epoch') == self.epoch and
                        speech_result.get('key') == self.speech_key):
                    result = speech_result.get('result')
                    if result == 'completed': self.event('speech_completed'); self._after_speech(now)
                    elif result in ('missing', 'failed'):
                        self.phase = 'guest_wait'; self.event('speech_unavailable', result=result)
                        # Display text and allow Continue; never claim unheard audio.
            elif self.phase == 'dwell' and now >= self.deadline: self._next(now)
            return None
    def status(self):
        with self.lock:
            target=self.batch_end if self.batch_end is not None else self.index
            point = self.steps[target]['point'] if target < len(self.steps) else None
            return {'phase': self.phase, 'epoch': self.epoch, 'run_id':self.run_id, 'route_name': self.route_name,
                    'route_id':self.route_id,'route_mode':self.route_mode,'route_reason':self.route_reason,'via':list(self.actual_transit),
                    'index': self.index, 'total': len(self.steps), 'point': copy.deepcopy(point),
                    'revision': self.document['revision'] if self.document else None,
                    'error': self.error, 'speech_key': self.speech_key,
                    'event_sequence': self.event_seq, 'events': copy.deepcopy(self.events[-12:])}

class MissionService:
    def session_snapshot(self):
        # Console owns the current original-source observation and its clock.
        # Background task management must not re-evaluate an older UI view.
        read = getattr(self.node, 'session_state', None)
        if callable(read):
            return read()
        return self.node.session.status(time.monotonic())
    """One worker, at most one dispatch. ROS/control threads never wait on TTS."""
    def __init__(self, node, catalog):
        self.node = node; self.catalog = catalog
        from reception_history import History
        self.history=History(node.runtime/'reception-history/events.sqlite3');self.engine = Mission(self.history.append)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='reception-leg')
        from reception_decisions import RouteDecisions
        from reception_live_routes import LiveRouter
        self.live_router=LiveRouter();self.active_command=None;self.resume_command=None
        self.decisions=RouteDecisions(); self.route_report=[]
        self.pending_source_command=None;self.source_recovery_active=False
        self.tracking_turn_leg=None
        self.future = None; self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='reception-task'); self.thread.start()
    def map_id(self):
        return self.node.map_name if getattr(self.node, 'map_name', '') else 'unselected'
    def observation_document(self):
        with self.engine.lock:
            doc=self.engine.document if self.engine.phase in BUSY else None
            if doc:return {'map_id':doc['map_id'],'revision':doc['revision'],'doors':copy.deepcopy(doc['doors']),
                           'annotations':copy.deepcopy(doc.get('annotations',[])),'policy':dict(doc['policy'])}
        return self.catalog.summary(self.map_id())
    def command(self, data):
        command = data.get('command')
        if command == 'start':
            with self.node.lock:
                doc = self.catalog.read(self.map_id())
                if data.get('revision') != doc['revision']: raise ValueError('路线已更新，请刷新任务列表')
                state = self.session_snapshot()
                if not state['navigation_permitted']: raise ValueError('当前 E 档位或摇杆未允许导航')
                if self.node.session.phase in ('starting', 'executing', 'replanning'): raise ValueError('已有导航正在执行')
                if self.future is not None and not self.future.done(): raise ValueError('上一任务正在退出，请稍候')
                route_id=data.get('route_id')
                if data.get('target') is not None:
                    target=data['target']
                    if not isinstance(target,dict) or any(type(target.get(k)) not in (int,float) or not math.isfinite(target[k]) or abs(target[k])>1e5 for k in ('x','y')):
                        raise ValueError('地图目标坐标无效')
                    point=dict(id='temporary_visit',name='地图临时目的地',x=float(target['x']),y=float(target['y']),yaw=None,kind='visit',product_id='',script='',dwell=0.,guest_wait=False)
                    doc['points']=[p for p in doc['points'] if p['id']!=point['id']]+[point]
                    data=dict(data,point_id=point['id'])
                if data.get('point_id'):
                    point=next((p for p in doc['points']if p['id']==data['point_id']),None)
                    if point is None:raise ValueError('点位不存在')
                    if data.get('dock_pose')is not None:
                        dock=data['dock_pose']
                        if not isinstance(dock,dict)or any(type(dock.get(k))not in(int,float)or not math.isfinite(dock[k])for k in ('x','y','yaw')):
                            raise ValueError('附近停靠坐标无效')
                        if math.hypot(dock['x']-point['x'],dock['y']-point['y'])>3.:raise ValueError('附近停靠超出原目标3米')
                        point.update(x=dock['x'],y=dock['y'])
                        if point['yaw']is not None:point['yaw']=math.remainder(dock['yaw'],2*math.pi)
                    route_id='single_visit';doc['routes']=[r for r in doc['routes']if r['id']!=route_id]+[dict(id=route_id,name=point['name'],mode='showcase',loops=1,description='',steps=[dict(point_id=point['id'],speak=True)])]
                doc,self.route_report=self.decisions.expand(doc,route_id,time.monotonic())
                self.node.session.configure_obstruction(doc['policy'])
                self.engine.start(doc, route_id, time.monotonic())
                self.active_command=None;self.resume_command=None;self.pending_source_command=None;self.source_recovery_active=False;self.live_router.reset()
                self.tracking_turn_leg=None
        elif command=='resume':
            with self.node.lock:
                if data.get('epoch')!=self.engine.epoch:raise ValueError('任务已更新，请刷新后继续')
                if not self.session_snapshot()['navigation_permitted']:raise ValueError('当前 E 档位或摇杆未允许导航')
                if not self.engine.document or self.engine.document['map_id']!=self.map_id():raise ValueError('当前地图已切换，请重新开始任务')
                if self.future is not None and not self.future.done():raise ValueError('上一导航正在退出，请稍候')
                self.engine.resume(time.monotonic());self.live_router.reset()
        elif command in ('cancel', 'pause'):
            with self.node.lock:
                self.pending_source_command=None;self.source_recovery_active=False
                if command=='pause':self.capture_resume()
                self.engine.cancel('paused' if command == 'pause' else 'cancelled'); self.node.stop()
        elif command in ('continue', 'skip'):
            if data.get('epoch')!=self.engine.epoch:raise ValueError('讲解任务已更新，请刷新后继续')
            state = self.session_snapshot()
            if not state['navigation_permitted']: raise ValueError('当前已交回人工，请重新开始任务')
            if command == 'continue': self.engine.continue_visit(time.monotonic())
            else: self.engine.skip(time.monotonic())
        else: raise ValueError('任务命令无效')
        return self.engine.status()
    def capture_resume(self):
        self.resume_command=None
        if self.engine.phase not in ('navigating','rerouting','waiting_route') or not self.active_command:return
        command=copy.deepcopy(self.active_command);feedback=getattr(self.node,'feedback',{})
        remaining=feedback.get('poses_remaining')if feedback.get('generation')==self.engine.goal_generation else None
        if type(remaining)is int and 0<remaining<=len(command['points']):command['points']=command['points'][-remaining:]
        for key in ('reroute','wait','source_recovery','source_deadline'):command.pop(key,None)
        self.resume_command=command
    def source_fault(self,reason):
        """Recover an authorized task through real cancel, plan and arm receipts.

        Manual/cancel invalidates the same mission epoch. Recovery changes the
        navigation goal generation, never the sensor/source timestamps.
        """
        with self.engine.lock:
            if self.source_recovery_active and self.engine.phase=='rerouting':return True
            if self.engine.phase=='navigating':
                end=self.engine.batch_end if self.engine.batch_end is not None else self.engine.index
                command=copy.deepcopy(self.active_command)if self.active_command else dict(epoch=self.engine.epoch,point=self.engine.steps[end]['point'],points=[x['point']for x in self.engine.steps[self.engine.index:end+1]],end_index=end)
                feedback=getattr(self.node,'feedback',{})
                remaining=feedback.get('poses_remaining')if feedback.get('generation')==self.engine.goal_generation else None
                if type(remaining)is int and 0<=remaining<=len(command['points']):
                    remaining=max(1,remaining)
                    passed=command['points'][:-remaining]
                    for p in passed:self.engine.event('transit_segment_completed',point=p['id'],receipt='native_poses_remaining_before_source_recovery')
                    command['points']=command['points'][-remaining:]
            elif self.engine.phase not in BUSY and self.node.session.phase in ('starting','executing','replanning'):
                # Legacy single-point navigation was already explicitly started.
                # Adopt that exact target in RAM; do not save a new catalog route.
                doc=copy.deepcopy(self.catalog.read(self.map_id()));g=self.node.session.goal
                if not g:return False
                point=dict(id='source_recovery',name='继续原目的地',x=g['x'],y=g['y'],yaw=None if getattr(self.node,'goal_heading_mode','pose')=='position'else g['yaw'],kind='visit',product_id='',script='',dwell=0.,guest_wait=False)
                via=[];raw=getattr(self.node,'goal_route',[])
                if raw:
                    remaining=getattr(self.node,'feedback',{}).get('poses_remaining')
                    if type(remaining)is int and 0<=remaining<=len(raw):raw=raw[-max(1,remaining):]
                    via=[dict(point,id='source_via_'+str(i),name='原路线途经点',x=q.pose.position.x,y=q.pose.position.y,yaw=None,kind='transit')for i,q in enumerate(raw[:-1])]
                doc['edges']=[]
                doc['points']=via+[point];doc['routes']=[dict(id='source_recovery',name='原导航观测恢复',mode='showcase',loops=1,description='',steps=[dict(point_id=q['id'],speak=False)for q in doc['points']])]
                doc['policy']['max_wait']=self.node.session.replan_timeout
                epoch=self.engine.start(doc,'source_recovery',time.monotonic());self.engine.dispatched(epoch,self.node.session.generation,len(via))
                command=dict(epoch=epoch,point=point,points=doc['points'],end_index=len(via))
                self.engine.event('adopted_authorized_goal',generation=self.node.session.generation)
            else:return False
            self.engine.begin_reroute(reason)
            command.update(epoch=self.engine.epoch,reroute=True,source_recovery=True,reason=reason,
                           source_deadline=time.monotonic()+self.node.session.replan_timeout)
            self.pending_source_command=command;self.active_command=copy.deepcopy(command);self.source_recovery_active=True
            self.engine.event('source_recovery_wait',generation=self.node.session.generation,reason=reason)
            return True
    def dispatch(self, command):
        epoch = command['epoch']
        if self.resume_command and not command.get('reroute'):
            command=dict(self.resume_command,epoch=epoch);self.resume_command=None
        p = command['point'];starting_generation=None
        try:
            if not self.engine.valid(epoch): return
            if command.get('reroute'):
                if not self.node.cancel_reception_leg(epoch):return
                if command.get('wait'):
                    self.engine.wait_route(epoch,time.monotonic(),command['reason']);return
            if command.get('source_recovery'):
                if time.monotonic()>=command['source_deadline']:raise ValueError('观测未在本次等待时间内恢复，已结束任务')
                while not self.node.health_ready() or not self.node.execution_ready():
                    if not self.engine.valid(epoch):return
                    state=self.session_snapshot()
                    if not state['navigation_permitted']:return
                    if time.monotonic()>=command['source_deadline']:raise ValueError('观测未在本次等待时间内恢复，已结束任务')
                    time.sleep(.05)
            points=command.get('points',[p])
            read_pose=getattr(self.node,'reception_pose',None)
            poses=route_poses(points,read_pose() if callable(read_pose) else None)
            data=dict(poses[-1],_mission_epoch=epoch,heading_mode='position' if p['yaw'] is None else 'pose')
            if getattr(self.node,'gait_profile',None)=='p2-flat-model'and (p.get('script')or p.get('product_id')or p.get('dwell')or p.get('guest_wait')or p.get('yaw')is not None):data['_arrival_profile']='presentation'
            if len(points)>1:
                data['_route_goals']=poses
            starting_generation=self.node.session.generation if command.get('source_recovery')else None
            try:
                if command.get('tracking_recovery'):
                    from heading_recovery import plan_after_turn
                    result=plan_after_turn(self.node,self.engine,epoch,data,ValueError(command['reason']),require_turn_progress=True)
                    if result is None:return
                else:
                    result = self.node.set_goal(data)
            except ValueError as plan_error:
                from heading_recovery import no_route, plan_after_turn
                if (getattr(self.node, 'gait_profile', None) != 'p2-flat-model' or
                        command.get('source_recovery') or command.get('tracking_recovery') or not no_route(plan_error)):
                    raise
                result = plan_after_turn(self.node, self.engine, epoch, data, plan_error)
                if result is None:return
            # Allow the existing observations/lifecycle to finish initializing.
            # Do not change freshness requirements or resume after takeover.
            deadline=time.monotonic()+8.
            while ((hasattr(self.node,'health_ready') and not self.node.health_ready()) or
                   (hasattr(self.node,'execution_ready') and not self.node.execution_ready())):
                if not self.engine.valid(epoch):return
                state=self.session_snapshot()
                if not state['navigation_permitted'] or state['generation']!=result['generation']:return
                if time.monotonic()>=deadline:
                    gait=getattr(self.node,'gait',None)
                    reason=gait.status().get('blockers','') if gait else ''
                    raise ValueError(reason or '定位或避障观测尚未收到有效数据，请查看实时状态')
                time.sleep(.05)
            if not self.engine.valid(epoch): return
            self.node.confirm_goal({'generation': result['generation'], '_mission_epoch': epoch})
            if self.engine.dispatched(epoch, result['generation'],command.get('end_index'),[q['id']for q in points[:-1]]):
                self.source_recovery_active=False
                if command.get('source_recovery'):self.engine.event('source_recovery_resumed',generation=result['generation'])
                self.active_command=copy.deepcopy(command)
                self.active_command.pop('source_recovery',None);self.active_command.pop('source_deadline',None)
                for key in ('tracking_recovery','tracking_generation','reroute','reason'):
                    self.active_command.pop(key,None)
        except Exception as error:
            if not self.engine.valid(epoch):return
            if (command.get('source_recovery') and self.engine.valid(epoch) and
                    time.monotonic()<command['source_deadline'] and
                    (isinstance(error,RouteReplyTimeout) or not self.node.health_ready() or starting_generation is not None and self.node.session.generation>starting_generation and getattr(self.node,'source_revoked_generation',None)==self.node.session.generation)):
                self.pending_source_command=command
                self.engine.event('source_recovery_retry',reason=str(error))
            else:
                self.source_recovery_active=False;self.engine.failed(epoch,error)
    def tracking_recovery_command(self,state):
        """One checked turn/replan after a real aborted navigation action.

        A global preview can be valid while local forward tracking has no arc.
        This preserves the current visit and actions instead of ending the tour
        before the existing native Spin recovery can inspect the other side.
        Operator/source stops and cancelled actions are never recovery grants.
        """
        receipt=state.get('terminal_receipt') or {}
        if (getattr(self.node,'gait_profile',None)!='p2-flat-model' or
                state.get('phase')!='blocked' or not state.get('navigation_permitted') or
                state.get('manual_activity') or state.get('last_stop') or
                receipt.get('status')!=6 or receipt.get('task_kind')!='navigation' or
                receipt.get('succeeded')is not False or receipt.get('generation')!=state.get('generation') or
                (self.future is not None and not self.future.done())):
            return None
        with self.engine.lock:
            end=self.engine.batch_end if self.engine.batch_end is not None else self.engine.index
            key=(self.engine.epoch,self.engine.index,end)
            if (self.engine.phase!='navigating' or state['generation']!=self.engine.goal_generation or
                    not self.active_command or self.active_command['epoch']!=self.engine.epoch or
                    getattr(self,'tracking_turn_leg',None)==key):
                return None
            command=copy.deepcopy(self.active_command)
            feedback=getattr(self.node,'feedback',{})
            remaining=feedback.get('poses_remaining')if feedback.get('generation')==self.engine.goal_generation else None
            if type(remaining)is int and 0<remaining<=len(command['points']):
                passed=command['points'][:-remaining]
                for p in passed:self.engine.event('transit_segment_completed',point=p['id'],receipt='native_poses_remaining_before_heading_recovery')
                command['points']=command['points'][-remaining:]
            reason=state.get('notice')or '本段导航动作中止，尝试原地转向后重规划'
            if not self.engine.begin_reroute(reason):return None
            self.tracking_turn_leg=key
            command.update(epoch=self.engine.epoch,end_index=end,reroute=True,tracking_recovery=True,
                           tracking_generation=state['generation'],reason=reason)
            self.engine.event('tracking_heading_recovery',generation=state['generation'],point=command['point']['id'])
            return command
    def update_live_route(self):
        snapshot=self.engine.status()
        if snapshot['route_mode']!='adaptive' or snapshot['phase']not in ('navigating','waiting_route') or not self.active_command:return
        if self.future is not None and not self.future.done():return
        document=self.engine.document;pose=self.node.reception_pose()if hasattr(self.node,'reception_pose')else getattr(self.node,'pose',None)
        states=self.decisions.states(document,time.monotonic())
        feedback=getattr(self.node,'feedback',{})
        remaining=feedback.get('poses_remaining')if feedback.get('generation')==self.engine.goal_generation else None
        previous=self.engine.steps[self.engine.index-1]['point']['id']if self.engine.index else None
        decision=self.live_router.choose(document,states,pose,self.active_command,previous,remaining,time.monotonic(),snapshot['phase']=='waiting_route')
        if not decision or not self.engine.begin_reroute(decision['reason']):return
        command=dict(epoch=self.engine.epoch,point=self.engine.steps[self.engine.batch_end]['point'],end_index=self.engine.batch_end,
                     reroute=True,**decision)
        if not decision['wait']:command['point']=decision['points'][-1]
        self.route_report=(self.route_report+[{'kind':'live_update','reason':decision['reason'],'old_cost':decision.get('old_cost'),'new_cost':decision.get('new_cost')}])[-20:]
        self.future=self.executor.submit(self.dispatch,command)
    def run(self):
        while not self.closed.wait(.1):
            try:
                state = self.session_snapshot()
                recovery = self.tracking_recovery_command(state)
                # Speech acknowledgements are local, bounded files; no cloud here.
                speech = self.node.reception_speech_result()
                snapshot=self.engine.status()
                if snapshot['phase']=='presenting':
                    from pathlib import Path
                    import json
                    clip=self.node.reception_voice.root/(snapshot['speech_key']+'.wav')
                    if not clip.is_file():speech={'epoch':snapshot['epoch'],'key':snapshot['speech_key'],'result':'missing'}
                    elif time.monotonic()-(self.engine.presentation_started or time.monotonic())>3:
                        try: worker=json.loads((self.node.runtime/'audio-feedback-state.json').read_text())
                        except (OSError,ValueError):worker={}
                        if not 0<=time.time()-worker.get('updated_at',0)<=3:
                            speech={'epoch':snapshot['epoch'],'key':snapshot['speech_key'],'result':'failed'}
                if not state.get('navigation_permitted') and snapshot['phase'] in BUSY:self.capture_resume()
                command = self.engine.tick(state, time.monotonic(), speech) or recovery
                current = self.engine.status()
                if snapshot['phase'] in BUSY and current['phase'] in ('manual', 'paused'):
                    # E pause and manual takeover end physical navigation too;
                    # a task banner alone is not a stop receipt.
                    self.source_recovery_active=False;self.pending_source_command=None
                    self.node.stop()
                if (self.future is None or self.future.done()) and getattr(self,'pending_source_command',None) is not None:
                    command=self.pending_source_command;self.pending_source_command=None
                if command and (self.future is None or self.future.done()): self.future = self.executor.submit(self.dispatch, command)
                self.update_live_route()
            except Exception as error: self.engine.failed(self.engine.epoch, error)
    def close(self):
        self.closed.set()
        if self.engine.phase in BUSY:self.engine.cancel('interrupted')
        self.thread.join(timeout=1)
        self.executor.shutdown(wait=False, cancel_futures=True);self.history.close()
