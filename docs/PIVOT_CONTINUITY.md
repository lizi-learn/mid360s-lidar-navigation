# Measured-response continuity for large pivots

Field telemetry showed repeated one-direction raw yaw requests followed by
zero-input intervals during large turns. The pivot helper unconditionally ended
each far pulse after three seconds. Each restart also restarted the native
0.5-input/s yaw slew, making a slow turning response visibly intermittent.

An initial candidate removed that deadline for every large turn. Controlled
long-delay/high-gain cases then overshot and failed; that candidate was rejected.
The selected change retains the original probe for unknown or fast responses.
Continuity is enabled only for a slow observed response with sustained actual
heading displacement, or slow peak-rate evidence from a completed and settled
far pulse. If the current peak rate rises, the original bounded probing returns.
These measurements choose the response mode; they do not authorize map clearance.

Large turns with that evidence keep their existing full raw request while the
remaining angle is large. Actual-rate braking, sign crossing, callback-gap
handling, complete-footprint collision projection, goal tolerance and progress
checks are unchanged. Small final corrections retain bounded pulses. The
selected input multiplier and 1.0 raw turning maximum are unchanged; 1.0 is a
policy input, not a measured rad/s limit. No gait or motor driver is changed.

In paired tests of the actual Nav2 plugin, eight synthetic slow-response large
turns had median duration 28.425s before and 22.35s after. Raw start counts fell
from 6–9 to 4–5. All 23 cases passed for both versions, including delayed fast
responses, small turns, actual synthetic obstacles and deactivation. Loaded
library paths/SHA, parameters, source, traces and seed are recorded. These are
pure-yaw delay/deadzone plants with the native input arithmetic and original
55cm square, not a biped simulation or a new physical walking result.

A wider preliminary 768-case arithmetic sweep retained its original 45-second
bound: the selected candidate had six failures versus 34 for the old helper,
with no formerly passing case failing. The remaining very slow cases and the
rejected unlimited candidate are retained privately; this is not an all-model
pass claim. `experiments/arrival/check_pivot_continuity.py` adds the reproducible
native scenarios to CI; `pivot_observation` also covers the continuity/probe
decision and a subsequent increase in measured response.
