"""Optional translation/turn handoff using observed mean body motion.

A zero-forward turn request first stops the prior translation. After a pivot,
forward motion waits for yaw rate to settle. The normal command gate, collision
monitor and rate limits remain authoritative. Thresholds require gait validation.
"""
import math
class GaitHandoff:
    def __init__(self,linear_settle=.06,angular_settle=.06,settle_time=.15):
        if not all(math.isfinite(v) and v>0 for v in (linear_settle,angular_settle,settle_time)):raise ValueError('Invalid handoff thresholds')
        self.linear_settle=linear_settle;self.angular_settle=angular_settle;self.settle_time=settle_time;self.reset()
    def reset(self):self.phase='stopped';self.stable_since=None;self.last_time=None
    def settled(self,now,condition):
        if not condition:self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=now
        return now-self.stable_since>=self.settle_time
    def step(self,command,intent,measured,now,intent_age,feedback_age):
        values=(*command,*intent,*measured,now,intent_age,feedback_age)
        if (not all(math.isfinite(v) for v in values) or not 0<=intent_age<=.3 or not -.05<=feedback_age<=.5
                or self.last_time is not None and not 0<now-self.last_time<=.25):
            self.reset();self.phase='feedback_unavailable';return (0.,0.,0.)
        self.last_time=now
        if abs(intent[0])+abs(intent[2])<1e-8:self.phase='stopped';self.stable_since=None;return (0.,0.,0.)
        turning=abs(intent[0])<.005 and abs(intent[2])>.005
        if turning:
            if self.phase not in ('braking_translation','pivot'):
                self.phase='braking_translation';self.stable_since=None
            if self.phase=='braking_translation':
                if not self.settled(now,math.hypot(measured[0],measured[1])<=self.linear_settle):return (0.,0.,0.)
                self.phase='pivot';self.stable_since=None
            return (0.,command[1],command[2])
        if self.phase in ('pivot','braking_translation','settling_yaw') or self.phase!='forward' and abs(measured[2])>self.angular_settle:
            if self.phase!='settling_yaw':self.phase='settling_yaw';self.stable_since=None
            if not self.settled(now,abs(measured[2])<=self.angular_settle):return (0.,0.,0.)
        self.phase='forward';self.stable_since=None;return command
