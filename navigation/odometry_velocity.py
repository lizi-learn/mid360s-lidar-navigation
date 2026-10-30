"""Causal planar velocity estimate for a stepping body, without smoothing its pose.

Only past source-stamped poses are used. This is control feedback, not a pose or
IMU fusion filter. Consumers must still enforce source freshness and clear state
when the reference changes. No pose is extrapolated during missing data.
"""
from collections import deque
import math
import numpy as np

class PlanarVelocity:
    def __init__(self, window=.4, minimum_span=.15, maximum_gap=.3):
        if not all(math.isfinite(x) for x in (window,minimum_span,maximum_gap)) or not 0<minimum_span<=window<=2 or not 0<maximum_gap<=1:
            raise ValueError('Invalid velocity estimator timing')
        self.window=window;self.minimum_span=minimum_span;self.maximum_gap=maximum_gap;self.reset()
    def reset(self):
        self.samples=deque(maxlen=2048);self.last=None
    def update(self, stamp, pose):
        values=np.asarray([stamp,*pose],dtype=float)
        if values.shape!=(4,) or not np.isfinite(values).all():
            self.reset();raise ValueError('Invalid source pose/time')
        t,x,y,yaw=map(float,values)
        if self.samples:
            prev=self.samples[-1];dt=t-prev[0]
            delta=math.atan2(math.sin(yaw-prev[3]),math.cos(yaw-prev[3]))
            if dt==0 and math.hypot(x-prev[1],y-prev[2])<1e-10 and abs(delta)<1e-10:return self.last
            if dt<=0:
                self.reset();raise ValueError('Pose time did not advance')
            if dt>self.maximum_gap:self.reset()
            elif math.hypot(x-prev[1],y-prev[2])/dt>2 or abs(delta)/dt>4:
                self.reset();raise ValueError('Pose discontinuity exceeds flat-ground profile')
            else:yaw=prev[3]+delta
        self.samples.append((t,x,y,yaw))
        while len(self.samples)>2 and self.samples[1][0]<=t-self.window:self.samples.popleft()
        if self.samples[-1][0]-self.samples[0][0]<self.minimum_span:self.last=None;return None
        a=np.asarray(self.samples);times=a[:,0]-t;centered=times-times.mean();denominator=float(centered@centered)
        rates=centered@a[:,1:]/denominator
        c,s=math.cos(yaw),math.sin(yaw)
        self.last=(float(c*rates[0]+s*rates[1]),float(-s*rates[0]+c*rates[1]),float(rates[2]))
        return self.last
