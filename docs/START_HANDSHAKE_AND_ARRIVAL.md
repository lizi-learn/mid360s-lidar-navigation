# Start handshake and pivot-capable arrival

A real start request reached Nav2, obtained the native gait lease, then was
revoked about 40 ms later with `navigation_source_clock_rollback`. The zero
handshake used RPC send time, while subsequent velocity packets correctly used
the original 20 Hz command-gate evaluation time. A cached gate sample could
therefore be older than the handshake. Using a pre-arm gate sample instead would
also fail the native arm fence.

The transport now records the actual arm acknowledgement, waits for a genuine
gate evaluation after it, and uses that original evaluation time for the zero
handshake. Subsequent packets keep their original sample times. No clock,
expiry, rollback, takeover, or cancellation checks are disabled. A fake-clock
unit regression and the actual Python-to-C++ socket contract were exercised
with 20 Hz health samples; stale-command and manual-takeover cases still pass.
These fixtures do not operate motors or establish real walking performance.

The pivot-capable planner previously tried a direct connection only within
30 cm. More distant goals went through car-style lattice primitives, which
could produce a large arrival loop for an opposing final yaw. It now tries a
pivot, forward translation, and terminal pivot for any connection within the
map's diagonal. Every sample uses the existing footprint collision checker;
blocked connections fall back to lattice planning. The original short-path
helper limit remains the default for other callers. The requested footprint,
obstacle inputs, and speed limits are unchanged.

A real deployed planner produced a straight 5.32 m connection for the latest
selected point, with no positional overshoot to impose final yaw. Explicit final
heading remains a rotation at the endpoint. The APP defaults to a forward
arrival heading unless the operator chooses a fixed heading. Reload an already
open map page after updating its served UI. A path preview or zero handshake is
not recorded as successful physical walking.

A subsequent real start confirmed that the clock rollback was gone, but a
separate health-forwarding issue still revoked the lease. Native readiness
intermittently lost its health-derived bits while incoming sensor, localization,
and ground reports remained valid. The native 100 ms health decision lifetime
is unchanged. Console now forwards each gate callback immediately, and the gate
performs actual decisions at 50 Hz rather than 20 Hz. It still checks original
sensor ages on each evaluation; it does not renew sensor or odometry timestamps.
An 18 s read-only deployed check obtained 449 native status samples, no lost
health-derived bits, and a maximum sampled health age of 33.7 ms. This check is
not a claim of completed physical navigation.

A longer follow-up found a second, independent reference-report coupling:
reference validity was only published after processing a new cloud, even when
pose/velocity validation was still valid. The adapter now publishes an
independent reference decision after checking the original cloud/odometry ages,
pose, optional body transform and measured-velocity estimator. Repeated decisions
retain `reference_source_stamp_s` and expose its current age; they do not publish
another cached pose/TF. Gate and motion buffer consume this separate decision.
Source expiry and receipt deadlines are unchanged. A regression covers the
240 ms cached-source case and verifies that 510 ms expired data cannot renew it.

Two bounded real starts then maintained native ownership and delivered nonzero
commands, but did not produce meaningful body motion. A passive copy of the
existing control context confirmed AMP_WALK, no velocity interlock, and nonzero
controller velocity consumed by the policy. This is evidence of a remaining
policy-response problem, not a successful physical navigation result. The
second test's recorded UI scale was 1.0, contrary to its intended half-speed
setup; this discrepancy is retained and scale 0.5 was restored afterward.
