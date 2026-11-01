# Direction-aware obstacle stopping

The optional `P2_NAV_DIRECTIONAL_STOP=true` runtime profile retains the configured
body and costmap padding. It replaces the fixed front stopping rectangle with
two native Nav2 checks: STOP for points already in that body, and APPROACH
prediction of the same body along the requested forward/turn trajectory for
one second, sampled every20ms. A predicted collision becomes immediate STOP,
rather than an extra policy-input slowdown. No lateral input is introduced.

The implementation uses the locked Humble1.1.20 [native polygon collision-time
projection](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/polygon.cpp).
The future-motion polygon subscribes to the actual published costmap footprint.
An empty, nonfinite or zero-area footprint produces STOP. Existing finite scan
observations, original scan/TF expiry, point threshold, source-health reporting,
operator takeover and cancellation remain. Other profiles keep their previous
configuration. This profile changes the stopping geometry, not the user-defined
body dimensions, and does not certify physical stopping performance.

The trigger was a supervised field request that advanced about25m net before
an obstacle stop. A left-front detour planned successfully with the unchanged
55cm body, but the fixed rectangle still prevented it from starting. Both
unsuccessful requests are retained. This is not an office traversal pass.

The synthetic native component checks a clear left-forward arc next to a
right-front wall, a blocked straight/right arc, future forward collision,
front/rear corner sweeps, a collision on the left arc, present body intrusion,
empty footprint, stale scan and failed TF, then clear recovery. It does not use
an office map, a physical plant or an actuator connection. Physical application
and navigation outcome must be reported separately after installation.
