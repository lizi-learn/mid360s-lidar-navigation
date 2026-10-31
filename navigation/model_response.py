"""Isolated-model speed tracking. No actuator I/O or physical calibration.

Navigation targets and learned-policy inputs are distinct quantities. Curves
must come from an explicit model experiment; no default hardware curve exists.
"""
import math


class ModelResponseTracker:
    def __init__(self, profile):
        if profile.get('scope') != 'isolated_model':
            raise ValueError('Only an explicit isolated-model profile is supported')
        self.curves = []
        for name in ('forward', 'yaw'):
            curve = [tuple(map(float, p)) for p in profile[name]]
            if (len(curve) < 2 or curve[0] != (0., 0.)
                    or not all(len(p) == 2 and all(math.isfinite(v) and v >= 0 for v in p) for p in curve)
                    or any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(curve, curve[1:]))):
                raise ValueError('Response curves must be finite and strictly monotone from zero')
            self.curves.append(curve)
        if self.curves[0][-1][0] < .15 or self.curves[1][-1][0] < .24:
            raise ValueError('Response curves do not cover candidate navigation targets')
        # An explicit measured model deadband may make the inverse response
        # discontinuous at zero. Do not infer that threshold for hardware.
        self.floor = tuple(profile.get('nonzero_policy_floor', (0., 0.)))
        if (len(self.floor) != 2 or not all(math.isfinite(v) and v >= 0 for v in self.floor)
                or any(v > self.curves[i][1][1] for i, v in enumerate(self.floor))):
            raise ValueError('Invalid explicit model nonzero policy floor')
        self.reset()

    def reset(self):
        self.filtered = None
        self.integral = [0., 0.]
        self.output = [0., 0.]

    def interpolate(self, index, value):
        curve = self.curves[index]
        for (a, x), (b, y) in zip(curve, curve[1:]):
            if value <= b:
                return x + (value-a)/(b-a)*(y-x)
        raise ValueError('Target outside model response curve')

    def step(self, target, measured, dt, feedback_age):
        if (len(target) != 2 or len(measured) != 2
                or not all(math.isfinite(x) for x in (*target, *measured, dt, feedback_age))
                or not 0 < dt <= .25 or not 0 <= feedback_age <= .15
                or not 0 <= target[0] <= .15 or abs(target[1]) > .24
                or abs(measured[0]) > 2 or abs(measured[1]) > 4):
            self.reset()
            return (0., 0.)
        if self.filtered is None:
            self.filtered = list(measured)
        alpha = dt/(.25+dt)
        for i in range(2):
            self.filtered[i] += alpha*(measured[i]-self.filtered[i])
        for i, (kp, ki, bound, maximum, rate) in enumerate(
                ((.10, .04, .015, .30, .15), (.12, .03, .03, .50, .30))):
            if target[i] == 0:
                # Zero is never lifted above the learned-policy deadband.
                self.integral[i] = self.output[i] = 0.
                continue
            sign = 1. if target[i] > 0 else -1.
            if self.output[i]*sign < 0:
                self.integral[i] = self.output[i] = 0.
                continue
            error = target[i]-self.filtered[i]
            candidate = max(-bound, min(bound, self.integral[i]+ki*error*dt))
            feedforward = sign*max(self.floor[i], self.interpolate(i, abs(target[i])))
            raw = feedforward + kp*error + candidate
            value = sign*max(0., min(maximum, raw*sign))
            applied = self.output[i] + max(-rate*dt, min(rate*dt, value-self.output[i]))
            # Do not accumulate the start-up error while the policy request is
            # still limited by its slew or magnitude. That error otherwise
            # becomes an avoidable overshoot after the gait starts stepping.
            if (raw-applied)*error > 0:
                candidate = self.integral[i]
                raw = feedforward + kp*error + candidate
                value = sign*max(0., min(maximum, raw*sign))
            self.integral[i] = candidate
            delta = max(-rate*dt, min(rate*dt, value-self.output[i]))
            self.output[i] += delta
        return tuple(self.output)
