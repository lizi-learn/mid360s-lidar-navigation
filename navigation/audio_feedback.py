"""Quiet navigation cues. This module has no motion/control client or ROS imports."""
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import threading
import time

PHRASES = {
    'started': '开始导航。',
    'stopped': '导航已停止。',
    'paused': '导航已暂停。',
    'manual': '已切换为手动控制。',
    'obstacle': '前方有障碍，正在调整路线。',
    'continued': '路线已更新，继续导航。',
    'failed': '暂时无法继续，导航已停止。',
    'no_path': '暂时没有找到可通行路线。',
    'arrived': '已到达目的地。',
    'test': '这是导航提示音。',
}
ACTIVE = {'starting', 'executing', 'replanning'}
TERMINAL = {'stopped', 'cancelled', 'paused', 'manual', 'blocked', 'arrived'}
PRIORITY = {'failed': 40, 'no_path': 40, 'stopped': 35, 'paused': 35,
            'manual': 35, 'arrived': 30, 'started': 15, 'continued': 10,
            'obstacle': 5, 'test': 1}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + f'.{os.getpid()}.{threading.get_ident()}.tmp')
    try:
        temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False))
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


class AudioSettings:
    """Per-navigation stream settings; never changes the system mixer."""
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.Lock()

    def read(self):
        with self.lock:
            return self._read()

    def _read(self):
        default = {'enabled': True, 'volume': 55, 'test_generation': 0}
        try:
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict):
                raise ValueError('Invalid audio settings')
            if type(data.get('enabled')) is not bool or type(data.get('volume')) is not int or not 0 <= data['volume'] <= 100:
                raise ValueError('Invalid audio settings')
            generation = data.get('test_generation', 0)
            if type(generation) is not int or generation < 0:
                raise ValueError('Invalid audio test generation')
            return {**default, 'enabled': data['enabled'], 'volume': data['volume'], 'test_generation': generation}
        except FileNotFoundError:
            return default
        except (OSError, ValueError, TypeError):
            return {**default, 'enabled': False, 'error': 'audio_settings_invalid'}

    def update(self, data):
        if not isinstance(data, dict) or set(data) - {'enabled', 'volume', 'test'}:
            raise ValueError('声音设置字段无效')
        if 'enabled' in data and type(data['enabled']) is not bool:
            raise ValueError('声音开关无效')
        if 'volume' in data and (type(data['volume']) is not int or not 0 <= data['volume'] <= 100):
            raise ValueError('音量范围为0到100')
        if 'test' in data and type(data['test']) is not bool:
            raise ValueError('试听参数无效')
        with self.lock:
            result = {k:v for k,v in self._read().items() if k in {'enabled','volume','test_generation'}}
            result.update({k: data[k] for k in ('enabled', 'volume') if k in data})
            if data.get('test'):
                result['test_generation'] += 1
            atomic_json(self.path, result)
            return result

    def status(self):
        result = self.read()
        state = self.path.with_name('audio-feedback-state.json')
        try:
            report = json.loads(state.read_text())
            if not isinstance(report, dict):
                raise ValueError('Invalid audio worker status')
            result['player'] = {k: report.get(k) for k in ('pid', 'updated_at', 'last_event', 'playing', 'last_result', 'error', 'voice_available')}
            stamp = report.get('updated_at', 0)
            if not isinstance(stamp, (int, float)) or not 0 <= time.time() - stamp <= 3.:
                result['player']['error'] = 'audio_worker_offline'
        except (OSError, ValueError, TypeError):
            result['player'] = None
        return result


def save_resource(node, data):
    """Old G12 gateways already carry configuration Save requests.

    The typed audio payload saves ONLY audio preferences. It never calls map
    saving or a navigation action; mixed audio/map requests are rejected first.
    New gateways may use the dedicated /api/audio endpoint instead.
    """
    if 'reception' in data:
        if set(data)!={'reception'}:raise ValueError('导览配置不能与其他资源混合提交')
        return node.reception_save(data['reception'])
    if 'audio_feedback' in data:
        if set(data) != {'audio_feedback'}:
            raise ValueError('声音设置不能与保存地图混合提交')
        return node.audio.update(data['audio_feedback'])
    return node.save_map(data['name'])


@dataclass(frozen=True)
class Cue:
    kind: str
    generation: int
    epoch: int
    created: float

    @property
    def priority(self):
        return PRIORITY[self.kind]


class CuePolicy:
    """Announce meaningful transitions, not periodic global replans or notices."""
    def __init__(self, obstacle_delay=1.5, obstacle_cooldown=15.):
        self.obstacle_delay = obstacle_delay
        self.obstacle_cooldown = obstacle_cooldown
        self.epoch = 0
        self.reset()

    def reset(self):
        self.previous = None
        self.stream = None
        self.generation = 0
        self.blocked_since = None
        self.obstacle_announced = False
        self.last_obstacle = -math.inf
        self.last_now = None
        self.reception_run=None;self.reception_announced=False;self.reception_wait=None
        self.epoch += 1

    def disconnected(self):
        # Reconnection must not replay old Start or queue stale obstacle speech.
        self.reset()

    def observe(self, data, now):
        if not math.isfinite(now):
            self.reset()
            return None
        session = data.get('session') or {}
        phase, generation = session.get('phase'), session.get('generation')
        if not isinstance(phase, str) or type(generation) is not int or generation < 0:
            self.reset()
            return None
        stream = data.get('request_token')  # RAM-only server identity, never logged.
        changed = self.previous is None or stream != self.stream or generation < self.generation
        if self.last_now is not None and now < self.last_now:
            changed = True
        if changed:
            self.reset()
            self.previous, self.generation, self.stream, self.last_now = phase, generation, stream, now
            return None
        previous = self.previous
        new_goal = generation != self.generation
        if new_goal:
            self.epoch += 1
            self.blocked_since = None
            self.obstacle_announced = False
        self.previous, self.generation, self.last_now = phase, generation, now
        kind = None
        if phase == 'replanning':
            if previous != 'replanning' or new_goal:
                self.blocked_since, self.obstacle_announced = now, False
            if (not self.obstacle_announced and self.blocked_since is not None and
                    now - self.blocked_since >= self.obstacle_delay and
                    now - self.last_obstacle >= self.obstacle_cooldown):
                self.obstacle_announced, self.last_obstacle = True, now
                kind = 'obstacle'
        else:
            if phase != previous or new_goal:
                if phase == 'executing':
                    if previous == 'replanning':
                        if self.obstacle_announced and now - self.last_obstacle >= 3.:
                            kind = 'continued'
                    else:
                        kind = 'started'
                elif phase == 'arrived' and previous in ACTIVE:
                    kind = 'arrived'
                elif phase == 'blocked' and previous in ACTIVE | {'planning', 'ready'}:
                    kind = 'no_path' if previous in {'planning', 'ready'} else 'failed'
                elif phase in {'manual', 'paused', 'stopped', 'cancelled'} and previous in ACTIVE:
                    kind = {'manual': 'manual', 'paused': 'paused'}.get(phase, 'stopped')
            self.blocked_since = None
            self.obstacle_announced = False
        kind=self.reception_filter(kind,data,now)
        return Cue(kind, generation, self.epoch, now) if kind else None

    def reception_filter(self,kind,data,now):
        mission=(data.get('reception')or{}).get('mission')or{}
        phase=mission.get('phase');run=mission.get('run_id')
        active=phase in {'dispatching','navigating','rerouting','waiting_route','presenting','guest_wait','dwell','paused'}
        if not run or not active:return kind
        if run!=self.reception_run:
            self.reception_run=run;self.reception_announced=False;self.reception_wait=None
        # Internal leg cancellation is part of the same task, not a user stop.
        if phase in ('dispatching','rerouting','waiting_route')and kind in ('started','stopped'):kind=None
        if kind=='started':
            if self.reception_announced:kind=None
            else:self.reception_announced=True
        if phase=='waiting_route':
            if self.reception_wait is None:self.reception_wait=now
            if now-self.reception_wait>=self.obstacle_delay and now-self.last_obstacle>=self.obstacle_cooldown:
                self.last_obstacle=now;kind='obstacle'
        else:self.reception_wait=None
        return kind

    def valid(self, cue, data, now, max_age=3.):
        phase = (data.get('session') or {}).get('phase')
        reception_phase=((data.get('reception')or{}).get('mission')or{}).get('phase')
        generation = (data.get('session') or {}).get('generation')
        allowed = {
            'started': {'executing'}, 'continued': {'executing'},
            'obstacle': {'replanning'}, 'arrived': {'arrived'},
            'no_path': {'blocked'}, 'failed': {'blocked'},
            'manual': {'manual'}, 'paused': {'paused'},
            'stopped': {'stopped', 'cancelled'}, 'test': set(),
        }
        return (cue.epoch == self.epoch and cue.generation == generation and
                0 <= now - cue.created and (max_age is None or now - cue.created <= max_age) and
                (cue.kind == 'test' or phase in allowed[cue.kind] or cue.kind=='obstacle' and reception_phase=='waiting_route'))
