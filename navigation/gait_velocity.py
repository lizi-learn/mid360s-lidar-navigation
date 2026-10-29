"""Experimental mean-velocity response correction for an isolated gait model.

Not a motor driver or physical calibration. Output is an internal model request,
not a certified body-speed limit. Emergency zero always bypasses correction.
"""
import math

class GaitVelocityTracker:
    def __init__(self):
        self.filtered=None;self.integral=[0.,0.]

    def reset(self):
        self.filtered=None;self.integral=[0.,0.]

    def step(self, target, measured, dt, feedback_age):
        values=(*target,*measured,dt,feedback_age)
        if (not all(math.isfinite(x) for x in values) or not 0<dt<=.25 or not 0<=feedback_age<=.15
                or not 0<=target[0]<=.300001 or abs(target[1])>.350001
                or abs(measured[0])>2. or abs(measured[1])>4.):
            self.reset();return (0.,0.)
        if abs(target[0])+abs(target[1])<1e-9:
            self.reset();return (0.,0.)
        if self.filtered is None:self.filtered=list(measured)
        alpha=dt/(.25+dt)
        for i in range(2):self.filtered[i]+=alpha*(measured[i]-self.filtered[i])
        output=[]
        for i,(kp,ki,bound,maximum) in enumerate(((.5,.3,.05,.30),(.8,.4,.10,.35))):
            if abs(target[i])<1e-9:self.integral[i]=0.;output.append(0.);continue
            error=target[i]-self.filtered[i]
            self.integral[i]=max(-bound,min(bound,self.integral[i]+ki*error*dt))
            command=target[i]+kp*error+self.integral[i]
            # Never request reverse or yaw opposite the active requested turn.
            output.append(max(0.,min(maximum,command)) if i==0 else math.copysign(max(0.,min(maximum,command*math.copysign(1,target[i]))),target[i]))
        return tuple(output)
