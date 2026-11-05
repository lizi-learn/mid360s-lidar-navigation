"""Candidate body-speed feedback/ramp. Place BEFORE collision monitoring.

No actuator I/O. Normal stop requires explicit intent; unclassified zero,
takeover, collision and health loss bypass the ramp. Parameters are candidates,
not measured P2 stopping performance or proof of dynamic balance.
"""
import math
from collections import deque
from gate import Limits

class MotionBuffer:
    def __init__(self,accel=.10,decel=.15,angular_accel=.30,settle_time=.15,feedback_timeout=None,ramp_enabled=True):
        if not all(math.isfinite(v) and v>0 for v in (accel,decel,angular_accel,settle_time)):
            raise ValueError('positive_finite_motion_parameters_required')
        self.feedback_timeout=Limits().sensor_timeout if feedback_timeout is None else feedback_timeout
        if not math.isfinite(self.feedback_timeout) or not 0<self.feedback_timeout<=Limits().sensor_timeout:
            raise ValueError('feedback timeout must not exceed the command gate sensor expiry')
        self.accel=accel;self.decel=decel;self.angular_accel=angular_accel;self.settle_time=settle_time
        self.ramp_enabled=ramp_enabled
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
                or not 0<dt<=.25 or not -.005<=feedback_age<=self.feedback_timeout
                or not 0<=target[0]<=.8 or target[1]!=0 or abs(target[2])>1.):
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
        # An in-place reversal waits for yaw to settle. Ordinary forward path
        # corrections slew through zero without treating biped yaw ripple as
        # another request to stop translation and begin an in-place pivot.
        if target[0]==0. and (self.w*w<0 or w*measured[2]<0 and angular>.06):
            w=0.;self.phase='settling_yaw';self.stable_since=None
        def slew(current,target,rate):
            delta=target-current;return current+math.copysign(min(abs(delta),rate*dt),delta)
        if self.ramp_enabled:
            self.v=slew(self.v,v,self.accel if v>self.v else self.decel)
            self.w=slew(self.w,w,self.angular_accel)
        else:
            # The physical gait handoff owns the sole input ramp. Keep only
            # the measured phase transitions and immediate-stop behavior here.
            self.v,self.w=v,w
        if self.phase=='normal_braking' and self.v==self.w==0 and self._settled(now,linear<=.06 and angular<=.06):
            self.phase='stopped'
        return (self.v,0.,self.w)


class MeasuredMotionMean:
    """Mean body velocity for phase transitions; retains the original source time.

    Step-frequency ripple must not look like sustained opposite yaw. Collision,
    operator and stale-source stops do not pass through this estimator.
    """
    def __init__(self, time_constant=.15, translation_window=0.):
        if not math.isfinite(time_constant) or time_constant<=0:
            raise ValueError('positive_finite_mean_time_required')
        if not math.isfinite(translation_window) or not 0<=translation_window<=.5:
            raise ValueError('invalid_translation_window')
        self.time_constant=time_constant;self.translation_window=translation_window
        self.source=None;self.value=None;self.translation=deque()
    def step(self, measured, source):
        measured=tuple(measured)
        if len(measured)!=3 or not all(math.isfinite(v) for v in (*measured,source)):
            self.source=None;self.value=None;self.translation.clear();return measured
        if self.source is not None and source<=self.source:
            return self.value
        if self.source is None or source-self.source>.5:
            self.value=measured;self.translation.clear()
        else:
            dt=source-self.source;alpha=dt/(self.time_constant+dt)
            self.value=tuple(a+alpha*(b-a) for a,b in zip(self.value,measured))
        if self.translation_window:
            self.translation.append((source,*measured[:2]))
            cutoff=source-self.translation_window
            while len(self.translation)>2 and self.translation[1][0]<=cutoff:self.translation.popleft()
            # Signed, source-time-weighted translation over a stride. Averaging
            # speed magnitudes turns zero-net biped sway into sustained drift;
            # a short EMA still passes much of that step-frequency component.
            # Retain the fast yaw estimator and every original freshness check.
            area=[0.,0.];span=0.
            for a,b in zip(self.translation,list(self.translation)[1:]):
                lo=max(a[0],cutoff);dt=b[0]-a[0]
                if dt<=0 or lo>=b[0]:continue
                fraction=(lo-a[0])/dt;width=b[0]-lo;span+=width
                for axis in range(2):
                    left=a[axis+1]+fraction*(b[axis+1]-a[axis+1])
                    area[axis]+=.5*(left+b[axis+1])*width
            if span>1e-9:self.value=(area[0]/span,area[1]/span,self.value[2])
        self.source=source;return self.value
