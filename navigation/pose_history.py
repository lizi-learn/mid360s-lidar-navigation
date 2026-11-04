"""Original-stamped pose interpolation for a registered cloud; no extrapolation."""
import bisect
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    position: tuple
    quaternion: tuple
    bracket_ns: tuple


class PoseHistory:
    def __init__(self, duration_ns=1_000_000_000, max_span_ns=500_000_000, limit=512):
        self.duration_ns = duration_ns
        self.max_span_ns = max_span_ns
        self.limit = limit
        self.times = []
        self.poses = []

    def add(self, stamp_ns, position, quaternion):
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            raise ValueError('Invalid original pose stamp')
        p, q = tuple(position), tuple(quaternion)
        if len(p) != 3 or len(q) != 4 or not all(math.isfinite(x) for x in p+q):
            raise ValueError('Invalid measured pose')
        norm = math.sqrt(sum(x*x for x in q))
        if not .8 < norm < 1.2:
            raise ValueError('Invalid measured quaternion')
        q = tuple(x/norm for x in q)
        i = bisect.bisect_left(self.times, stamp_ns)
        if i < len(self.times) and self.times[i] == stamp_ns:
            self.poses[i] = (p, q)
        else:
            self.times.insert(i, stamp_ns)
            self.poses.insert(i, (p, q))
        minimum = self.times[-1]-self.duration_ns
        keep = max(0, bisect.bisect_left(self.times, minimum)-1, len(self.times)-self.limit)
        if keep:
            del self.times[:keep]
            del self.poses[:keep]

    def at(self, stamp_ns):
        i = bisect.bisect_left(self.times, stamp_ns)
        if i < len(self.times) and self.times[i] == stamp_ns:
            p, q = self.poses[i]
            return PoseSample(stamp_ns, p, q, (stamp_ns, stamp_ns))
        if i == 0 or i == len(self.times):
            return None
        left, right = self.times[i-1], self.times[i]
        if right-left > self.max_span_ns:
            return None
        f = (stamp_ns-left)/(right-left)
        a, b = self.poses[i-1], self.poses[i]
        p = tuple(x+f*(y-x) for x, y in zip(a[0], b[0]))
        qa, qb = a[1], b[1]
        dot = sum(x*y for x, y in zip(qa, qb))
        if dot < 0:
            qb = tuple(-x for x in qb)
            dot = -dot
        dot = min(1., max(-1., dot))
        if dot > .9995:
            q = tuple(x+f*(y-x) for x, y in zip(qa, qb))
        else:
            angle = math.acos(dot)
            q = tuple((math.sin((1-f)*angle)*x+math.sin(f*angle)*y)/math.sin(angle)
                      for x, y in zip(qa, qb))
        norm = math.sqrt(sum(x*x for x in q))
        return PoseSample(stamp_ns, p, tuple(x/norm for x in q), (left, right))
