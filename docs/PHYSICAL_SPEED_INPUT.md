# Physical velocity input candidate

`navigation/physical_speed_input.hpp` provides a pure C++ velocity-source arbiter.
It has no socket, joystick, motor, shared-memory writer, or robot-control process.
It is not connected to a real gait controller. Live web goals continue to be
path previews with `hardware_execution_enabled=false`.

The input is physical body velocity: forward `vx` in m/s, lateral `vy` in m/s,
and yaw rate `wz` in rad/s. Body height and individual joint targets remain the
responsibility of the existing walking controller. This candidate retains the
0.15 m/s forward and 0.24 rad/s yaw limits, rejects reverse and any lateral
command, and does not rescale normalized joystick axes.

## Controller integration contract

1. Produce `GaitFeedback` inside the actual controller loop. Explicitly identify
   the supported active policy, current controller instance, policy generation,
   walking readiness, real operator connection and activity. Never infer active
   policy from a process name or a generic walking-mode enum.
2. Supply verified calibration, ground, localization and health state. Keep
   feedback timestamps in the same machine's monotonic clock. Web state and
   navigation packets cannot assert controller readiness or renew feedback.
3. An explicit operator start calls `arm`. A zero-velocity packet must establish
   the new source handshake before moving packets can be accepted. Packets from
   before arming, restarted sources, replayed sequences and time regressions are
   rejected. Do not refresh source timestamps at a relay.
4. Call `poll` every controller tick, including ticks without new packets.
   Feedback older than 100 ms or commands older than 300 ms revoke navigation.
   Check original source ages, rather than packet arrival time alone.
5. Real calibrated joystick activity revokes navigation on the next tick and
   requests goal cancellation. Use the existing operator activity signal before
   command scaling; do not wait for another navigation packet. When navigation
   no longer owns velocity, use the existing manual control path. Revoking the
   velocity source does not command joint damping or disable standing support.
6. Cancel the old navigation action, acknowledge that cancellation, and publish
   takeover status to the web coordinator. Returning the stick to neutral must
   not re-arm navigation. A new explicit web start and new zero handshake are
   required. `arm` refuses while an exit notification is pending or a lease is
   already active. The caller remains responsible for cancellation completion
   and matching the cancellation to the correct goal.

The web coordinator, a real input transport, controller feedback exporter and
this integration are still required. The header alone cannot claim real manual
takeover, gait execution, obstacle stopping or full navigation acceptance.

## Validation boundary

The native C++ regression exercises active motion, manual takeover without a new
packet, neutral-stick non-resumption, explicit restart, zero handshake, speed
bounds, stale feedback, command silence, source/policy changes, invalid packets,
clock regressions, and queued packets predating arming. It runs with undefined
behavior sanitization through `tests/test_physical_speed_input.py`.

An isolated 13-joint model diagnostic on 2026-10-31 remained upright after an
explicitly supported initialization, but a 0.15 m/s input for 5 s advanced only
about 0.027 m. A separate one-meter Nav2 goal did not complete. A synthetic
obstacle insertion was acknowledged, but the required collision-monitor STOP
receipt was absent; zero output alone did not count as a passed obstacle test.
These failures remain open. The model uses ideal body odometry and does not
establish real payload, calibration, actuator response or LiDAR timing accuracy.
