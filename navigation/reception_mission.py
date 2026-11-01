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

BUSY = {'dispatching', 'navigating', 'presenting', 'guest_wait', 'dwell'}

class Mission:
    def __init__(self):
        self.lock = threading.RLock(); self.epoch = secrets.randbits(48)*256; self.reset()
    def reset(self):
        self.phase = 'idle'; self.steps = []; self.index = 0; self.document = None
        self.route_name = ''; self.error = ''; self.goal_generation = None
        self.presentation_started = None; self.deadline = None; self.speech_key = ''; self.events = []; self.event_seq = 0
    def event(self, kind, **data):
        self.event_seq += 1
        self.events.append({'sequence': self.event_seq, 'kind': kind, **data})
        self.events = self.events[-100:]
    def start(self, document, route_id, now):
        with self.lock:
            if self.phase in BUSY: raise ValueError('当前任务正在执行，请先取消或暂停')
            route = next((r for r in document['routes'] if r['id'] == route_id), None)
            if route is None: raise ValueError('路线不存在')
            self.reset(); self.epoch += 1; self.document = copy.deepcopy(document)
            points = {p['id']: p for p in document['points']}
            self.steps = [dict(point=copy.deepcopy(points[s['point_id']]), speak=s['speak'])
                          for _ in range(route['loops']) for s in route['steps']]
            self.route_name = route['name']; self.phase = 'dispatching'
            self.event('started', at=now, route=route_id, revision=document['revision'])
            return self.epoch
    def valid(self, epoch):
        with self.lock: return epoch == self.epoch and self.phase in BUSY
    def cancel(self, reason='cancelled'):
        with self.lock:
            self.epoch += 1; self.phase = reason; self.speech_key = ''
            self.event(reason); return self.epoch
    def dispatched(self, epoch, generation):
        with self.lock:
            if epoch != self.epoch or self.phase != 'dispatching': return False
            self.goal_generation = generation; self.phase = 'navigating'
            self.event('leg_started', point=self.steps[self.index]['point']['id'])
            return True
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
        self.index += 1; self.goal_generation = None
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
            if self.phase == 'dispatching':
                return {'epoch': self.epoch, 'point': copy.deepcopy(self.steps[self.index]['point'])}
            if self.phase == 'navigating':
                if session.get('generation') != self.goal_generation:
                    self.failed(self.epoch, '导航目标已被其他操作替换'); return None
                phase = session.get('phase')
                if phase in ('blocked', 'cancelled', 'stopped', 'manual', 'paused'):
                    self.failed(self.epoch, session.get('notice', phase)); return None
                if phase == 'arrived':
                    p = self.steps[self.index]['point']; script = introduction(self.document, p) if self.steps[self.index]['speak'] else ''
                    self.event('arrived', point=p['id'], at=now)
                    if script:
                        self.phase = 'presenting'; self.presentation_started = now; self.speech_key = content_key(script)
                        self.event('presentation', key=self.speech_key, text=script, point=p['name'])
                    else: self._after_speech(now)
            elif self.phase == 'presenting':
                if speech_result and speech_result.get('key') == self.speech_key:
                    result = speech_result.get('result')
                    if result == 'completed': self.event('speech_completed'); self._after_speech(now)
                    elif result in ('missing', 'failed'):
                        self.phase = 'guest_wait'; self.event('speech_unavailable', result=result)
                        # Display text and allow Continue; never claim unheard audio.
            elif self.phase == 'dwell' and now >= self.deadline: self._next(now)
            return None
    def status(self):
        with self.lock:
            point = self.steps[self.index]['point'] if self.index < len(self.steps) else None
            return {'phase': self.phase, 'epoch': self.epoch, 'route_name': self.route_name,
                    'index': self.index, 'total': len(self.steps), 'point': copy.deepcopy(point),
                    'revision': self.document['revision'] if self.document else None,
                    'error': self.error, 'speech_key': self.speech_key,
                    'event_sequence': self.event_seq, 'events': copy.deepcopy(self.events[-12:])}

class MissionService:
    """One worker, at most one dispatch. ROS/control threads never wait on TTS."""
    def __init__(self, node, catalog):
        self.node = node; self.catalog = catalog; self.engine = Mission()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='reception-leg')
        from reception_decisions import RouteDecisions
        self.decisions=RouteDecisions(); self.route_report=[]
        self.future = None; self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='reception-task'); self.thread.start()
    def map_id(self):
        return self.node.map_name if getattr(self.node, 'map_name', '') else 'unselected'
    def command(self, data):
        command = data.get('command')
        if command == 'start':
            with self.node.lock:
                doc = self.catalog.read(self.map_id())
                if data.get('revision') != doc['revision']: raise ValueError('路线已更新，请刷新任务列表')
                state = self.node.session.status(time.monotonic())
                if not state['navigation_permitted']: raise ValueError('当前 E 档位或摇杆未允许导航')
                if self.node.session.phase in ('starting', 'executing', 'replanning'): raise ValueError('已有导航正在执行')
                if self.future is not None and not self.future.done(): raise ValueError('上一任务正在退出，请稍候')
                doc,self.route_report=self.decisions.expand(doc,data['route_id'],time.monotonic())
                self.node.session.configure_obstruction(doc['policy'])
                self.engine.start(doc, data['route_id'], time.monotonic())
        elif command in ('cancel', 'pause'):
            with self.node.lock:
                self.engine.cancel('paused' if command == 'pause' else 'cancelled'); self.node.stop()
        elif command in ('continue', 'skip'):
            if data.get('epoch')!=self.engine.epoch:raise ValueError('讲解任务已更新，请刷新后继续')
            state = self.node.session.status(time.monotonic())
            if not state['navigation_permitted']: raise ValueError('当前已交回人工，请重新开始任务')
            if command == 'continue': self.engine.continue_visit(time.monotonic())
            else: self.engine.skip(time.monotonic())
        else: raise ValueError('任务命令无效')
        return self.engine.status()
    def dispatch(self, command):
        epoch = command['epoch']; p = command['point']
        data = {'x': p['x'], 'y': p['y'], 'yaw': p['yaw'] or 0.,
                'heading_mode': 'position' if p['yaw'] is None else 'pose'}
        try:
            if not self.engine.valid(epoch): return
            result = self.node.set_goal(data)
            if not self.engine.valid(epoch): return
            self.node.confirm_goal({'generation': result['generation'], '_mission_epoch': epoch})
            self.engine.dispatched(epoch, result['generation'])
        except Exception as error: self.engine.failed(epoch, error)
    def run(self):
        while not self.closed.wait(.1):
            try:
                state = self.node.session.status(time.monotonic())
                # Speech acknowledgements are local, bounded files; no cloud here.
                speech = self.node.reception_speech_result()
                snapshot=self.engine.status()
                if snapshot['phase']=='presenting':
                    from pathlib import Path
                    import json
                    clip=self.node.reception_voice.root/(snapshot['speech_key']+'.wav')
                    if not clip.is_file():speech={'key':snapshot['speech_key'],'result':'missing'}
                    elif time.monotonic()-(self.engine.presentation_started or time.monotonic())>3:
                        try: worker=json.loads((self.node.runtime/'audio-feedback-state.json').read_text())
                        except (OSError,ValueError):worker={}
                        if not 0<=time.time()-worker.get('updated_at',0)<=3:
                            speech={'key':snapshot['speech_key'],'result':'failed'}
                command = self.engine.tick(state, time.monotonic(), speech)
                if command and (self.future is None or self.future.done()): self.future = self.executor.submit(self.dispatch, command)
            except Exception as error: self.engine.failed(self.engine.epoch, error)
    def close(self):
        self.closed.set(); self.engine.cancel(); self.thread.join(timeout=1)
        self.executor.shutdown(wait=False, cancel_futures=True)
