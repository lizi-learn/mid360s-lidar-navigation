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
