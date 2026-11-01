"""Semantic route decisions, not a collision checker or an obstacle eraser."""
from dataclasses import dataclass
import heapq
import math

@dataclass
class DoorEvidence:
    state: str = 'unknown'
    stamp: float = -math.inf
    source_sequence: int = -1
    clear_since: float | None = None
    def observe(self, state, stamp, sequence, now, clear_hold=1.5):
        if state not in ('open', 'closed', 'unknown') or type(sequence) is not int or sequence <= self.source_sequence:
            return False
        if type(stamp) not in (int,float) or type(now) not in (int,float) or not math.isfinite(stamp) or not math.isfinite(now) or not -.05 <= now-stamp <= 1.:
            return False
        previous=self.stamp
        self.source_sequence = sequence; self.stamp = stamp
        if state != 'open':
            self.state = state; self.clear_since = None
        else:
            if self.clear_since is None or stamp-previous>.75: self.clear_since = stamp
            self.state = 'open' if stamp-self.clear_since >= clear_hold else 'unknown'
        return True
    def current(self, now, expiry=3.):
        return self.state if -.05 <= now-self.stamp <= expiry else 'unknown'

def route(document, start, goal, door_states=None, closed_edges=()):
    points = {p['id']: p for p in document['points']}
    if start not in points or goal not in points: raise ValueError('路线端点不存在')
    graph = {k: [] for k in points}; states = door_states or {}; closed = set(closed_edges)
    for edge in document['edges']:
        # Unknown doors are not declared open. Existing free-space navigation can
        # independently find a verified path; graph routing never clears cells.
        if edge['id'] in closed or edge['door_id'] and states.get(edge['door_id']) != 'open': continue
        a, b = edge['a'], edge['b']; p, q = points[a], points[b]
        cost = math.hypot(p['x']-q['x'], p['y']-q['y']) + edge['penalty']
        graph[a].append((b, cost, edge['id']))
        if edge['bidirectional']: graph[b].append((a, cost, edge['id']))
    heap = [(0., start, [start], [])]; best = {start: 0.}
    while heap:
        cost, here, path, edges = heapq.heappop(heap)
        if cost > best[here]: continue
        if here == goal: return {'points': path, 'edges': edges, 'cost': cost}
        for there, distance, edge in graph[here]:
            candidate = cost + distance
            if candidate < best.get(there, math.inf):
                best[there] = candidate; heapq.heappush(heap, (candidate, there, path+[there], edges+[edge]))
    return None

class ObstructionPolicy:
    """Configurable service choices after a real stop; units are seconds.

    People count/classification is optional and must never be inferred from an
    unclassified LiDAR blob. Input `crowd` means an external verified label.
    """
    def __init__(self, policy): self.policy = policy; self.reset()
    def reset(self): self.since = None; self.clear_since = None; self.last_plan = -math.inf
    def decide(self, now, *, blocked, alternate=False, crowd=False, door_closed=False):
        if not math.isfinite(now): raise ValueError('计时无效')
        if not blocked:
            if self.since is None: return 'continue'
            if self.clear_since is None: self.clear_since = now
            if now-self.clear_since < self.policy['clear_hold']: return 'wait_clear'
            self.reset(); return 'resume'
        self.clear_since = None
        if self.since is None: self.since = now
        elapsed = now-self.since
        if elapsed >= self.policy['max_wait']: return 'stop_notify'
        if not door_closed and elapsed < self.policy['brief_wait']: return 'wait'
        if crowd and not alternate: return 'wait_crowd'
        if now-self.last_plan >= self.policy['replan_interval']:
            self.last_plan = now
            return 'detour' if alternate else 'replan'
        return 'wait'

def worthwhile_shortcut(old_cost, new_cost, gain=.2):
    return (all(math.isfinite(x) and x >= 0 for x in (old_cost, new_cost)) and
            old_cost > 0 and new_cost <= old_cost*(1-gain))
