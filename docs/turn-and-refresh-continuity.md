# Turn and path-refresh continuity

The native gait buffer now averages signed translation over a 0.5-second
source-time window when the physical gait owns the input ramp. A short EMA can
retain enough periodic lateral sway to repeatedly trip the existing translation
settling check during an otherwise in-place turn. Signed stride averaging
preserves sustained drift while reducing that periodic component. Yaw keeps its
original fast EMA. The existing thresholds, explicit zero behavior, source
expiry, collision decisions, footprint and native S-curve remain unchanged.
Generic launch configurations keep their existing mean. This is a feedback
estimator, not a slower actuator ramp or a replacement for obstacle checking.

Motion-buffer suppression logs include the actual mean, phase, command/source
ages and authority/collision state. Historical target and stage recordings show
where a request was withdrawn; they do not reconstruct every missing internal
buffer reason. Further physical recordings are needed to attribute remaining
stops and assess perceived smoothness.

P2 navigation trees also retry one failed path refresh inside the existing
pipeline before propagating failure to the normal route wait. The nonblocking
100 ms BT `Delay` does not send a robot Wait action or retain an old command.
The controller continues to evaluate its path against the current local map;
real obstructions and persistent planning failures still stop execution.
The P2 action acknowledgement budget is 200 ms, separately from all sensor,
operator and velocity source expiry settings. Generic configurations retain
their original 20 ms acknowledgement budget.

`refresh_retry` executes the actual Nav2 pipeline/recovery plugins: a transient
planner failure preserves controller execution, and a persistent failure halts
it. `check_stride_turn.py` compares the actual pivot/input arithmetic with both
feedback estimators on a named synthetic planar plant. Neither test constitutes
physical biped acceptance or an office travel-time prediction.
