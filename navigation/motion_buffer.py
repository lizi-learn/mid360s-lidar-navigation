"""Candidate body-speed feedback/ramp. Place BEFORE collision monitoring.

No actuator I/O. Normal stop requires explicit intent; unclassified zero,
takeover, collision and health loss bypass the ramp. Parameters are candidates,
not measured P2 stopping performance or proof of dynamic balance.
"""
import math

class MotionBuffer:
    def __init__(self,accel=.10,decel=.15,angular_accel=.30,settle_time=.15):
        if not all(math.isfinite(v) and v>0 for v in (accel,decel,angular_accel,settle_time)):
            raise ValueError('positive_finite_motion_parameters_required')
        self.accel=accel;self.decel=decel;self.angular_accel=angular_accel;self.settle_time=settle_time
        self.reset()
    def reset(self):
        self.v=self.w=0.;self.last=None;self.phase='stopped';self.stable_since=None
    def _settled(self,now,condition):
        if not condition:self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=now
        return now-self.stable_since>=self.settle_time
    def step(self,target,measured,now,feedback_age,*,hard_stop=False,normal_stop=False,frame_verified=False):
        values=(*target,*measured,now,feedback_age)
        dt=.01 if self.last is None else now-self.last
        if (hard_stop or not frame_verified or not all(math.isfinite(x) for x in values)
                or not 0<dt<=.25 or not -.005<=feedback_age<=.15
                or not 0<=target[0]<=.15 or target[1]!=0 or abs(target[2])>.24):
            self.reset();self.phase='immediate_zero';return (0.,0.,0.)
        self.last=now
        v,_,w=target
        linear=math.hypot(measured[0],measured[1]);angular=abs(measured[2])
        # A zero from a collision monitor MUST NOT be treated as a soft stop.
        if v==0 and w==0:
            if not normal_stop:
                self.reset();self.phase='immediate_zero';return (0.,0.,0.)
            self.phase='normal_braking'
        elif v==0:
            if self.phase=='pivot' and linear>.06:
                # Recheck throughout a turn: a zero forward command does not
                # prove the body stayed still. Withdraw yaw immediately, then
                # require a fresh translation-settling interval before pivoting.
                self.w=0.;self.phase='braking_translation';self.stable_since=None
            if self.phase not in ('braking_translation','pivot'):
                self.phase='braking_translation';self.stable_since=None
            if self.phase=='braking_translation':
                if self._settled(now,linear<=.06 and abs(self.v)<1e-6):
                    self.phase='pivot';self.stable_since=None
                else:v=w=0.
        else:
            if self.phase in ('pivot','braking_translation','settling_yaw'):
                if self.phase!='settling_yaw':self.phase='settling_yaw';self.stable_since=None
                if not self._settled(now,angular<=.06 and abs(self.w)<1e-6):v=w=0.
                else:self.phase='forward';self.stable_since=None
            else:self.phase='forward';self.stable_since=None
        # Opposite turns pass through zero and wait for measured yaw to settle.
        if self.w*w<0 or w*measured[2]<0 and angular>.06:
            w=0.;self.phase='settling_yaw';self.stable_since=None
        def slew(current,target,rate):
            delta=target-current;return current+math.copysign(min(abs(delta),rate*dt),delta)
        self.v=slew(self.v,v,self.accel if v>self.v else self.decel)
        self.w=slew(self.w,w,self.angular_accel)
        if self.phase=='normal_braking' and self.v==self.w==0 and self._settled(now,linear<=.06 and angular<=.06):
            self.phase='stopped'
        return (self.v,0.,self.w)
