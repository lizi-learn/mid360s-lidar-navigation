"""Final forward-only command contract, independent of ROS and gait implementation."""
from dataclasses import dataclass
import math

@dataclass
class Limits:
    max_v: float = .30
    max_w: float = .35
    accel: float = .15
    decel: float = .30
    angular_accel: float = .5
    command_timeout: float = .30
    sensor_timeout: float = .50

class ForwardGate:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.speed = .20
        self.armed = False
        self.v = self.w = 0.
        self.reason = 'disarmed'

    def set_speed(self, value):
        if not math.isfinite(value) or not .03 <= value <= self.limits.max_v:
            raise ValueError('speed must be finite and between 0.03 and 0.30 m/s')
        self.speed = value

    def step(self, command, dt, command_age, scan_age, odom_age,
             ground_ok=True, localized=True, paused=False):
        reason = ''
        values = (*command, dt, command_age, scan_age, odom_age)
        if not all(math.isfinite(x) for x in values) or dt <= 0 or dt > .25:
            reason = 'invalid_time_or_command'
        elif not self.armed or paused: reason = 'disarmed_or_paused'
        elif command_age < 0 or command_age > self.limits.command_timeout: reason = 'command_timeout'
        elif min(scan_age, odom_age) < -.05 or max(scan_age, odom_age) > self.limits.sensor_timeout: reason = 'sensor_timeout'
        elif not localized: reason = 'localization_unavailable'
        elif not ground_ok: reason = 'ground_unobserved'
        elif abs(command[1]) > 1e-6: reason = 'lateral_command_rejected'
        elif command[0] < -1e-6: reason = 'reverse_command_rejected'
        # A stop, collision-monitor zero, cancellation or health fault is immediate.
        # Normal speed changes are acceleration limited; emergency stops are not.
        if reason or abs(command[0]) + abs(command[2]) < 1e-9:
            self.v = self.w = 0.
            self.reason = reason or 'stop'
            return (0., 0., 0.)
        scale = 1.
        if command[0] > 0:scale = min(scale, self.speed / command[0], self.limits.max_v / command[0])
        if abs(command[2]) > 0:scale = min(scale, self.limits.max_w / abs(command[2]))
        target_v, target_w = command[0] * scale, command[2] * scale
        dv = self.limits.accel if target_v > self.v else self.limits.decel
        delta_v, delta_w = target_v - self.v, target_w - self.w
        fraction = min(1., dv * dt / max(abs(delta_v),1e-12),
                       self.limits.angular_accel * dt / max(abs(delta_w),1e-12))
        self.v += delta_v * fraction
        self.w += delta_w * fraction
        # Enforce a newly lowered speed cap immediately, even while decelerating.
        self.v = min(self.v, self.speed)
        self.reason = 'tracking'
        return (self.v, 0., self.w)
