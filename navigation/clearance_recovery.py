"""Require a continuous clear observation interval after a stop or health fault."""
import math
class ClearanceRecovery:
    def __init__(self,hold=.75):
        if not math.isfinite(hold) or not 0<hold<=5:raise ValueError('Invalid clear observation interval')
        self.hold=hold;self.clear_since=None;self.last=None
    def step(self,now,clear):
        if not math.isfinite(now):self.clear_since=None;self.last=None;return False
        if self.last is not None and now<self.last:self.clear_since=None
        self.last=now
        if not clear:self.clear_since=None;return False
        if self.clear_since is None:self.clear_since=now
        return now-self.clear_since>=self.hold
