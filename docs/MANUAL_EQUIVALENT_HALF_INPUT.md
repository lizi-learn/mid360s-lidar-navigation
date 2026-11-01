# D-policy manual-equivalent forward input

On 2026-11-01 the operator explicitly requested half the existing manual forward
maximum: `0.8 * 0.5 = 0.4` as the policy input. This supersedes the earlier
assistant-selected 0.15 forward candidate and the experimental low-speed PI
adapter. It does not assert that a policy input of 0.4 produces exactly 0.4 m/s.

For `p2-flat-model`, RPP, the input buffer, smoother, final gate, Python socket
encoder and native lease now share the nominal manual forward range 0..0.8.
The default scale remains 0.5, applied once by the final gate. The native adapter
slews toward the resulting input directly. It does not search for a learned
deadband, use an inverse model curve, or modify input according to measured
body-speed error. Requested input, applied policy input and measured velocity
remain separate telemetry fields. Curves, approach slowdown and obstacles can
request less than the selected maximum; 0.4 is not a forced minimum.

Only the forward range changes. Navigation remains forward-only, with zero
lateral input. Existing turn bounds, E-left permission and single Start, E-middle
pause, E-right/manual-stick takeover, cancellation and immediate obstacle zero
remain. Original source timestamps and expiry checks remain; no sensor data is
fabricated or restamped. The operator's 55 cm total square is unchanged.

The standalone checks include the complete buffer/gate/encoding half-input
calculation and a real Python-to-C++ Unix socket fixture that verifies a 0.4
native policy input. These checks are not evidence of physical walking. Earlier
no-motion tests, source-clock and health failures remain in the existing report.
Actual joint targets, measured joint response and driver overrides must be
reported from the subsequent real test before claiming that it moved.

Runtime geometry follow-up: the first real request still produced zero. The
configured costmaps used the requested55cm body, but collision StopZone retained
the old1.14m width and oscillated STOP/clear. Runtime geometry now updates both
StopZone and SlowZone from the same body, retaining the existing clearances,
point threshold and stop/slowdown actions. No returns are removed or reclassified.
The original failed request remains in private evidence; this fix needs a new
real request before it can establish movement.

The next real request stayed navigation_active but input peaked at0.09816.
Actual staged values established the extra reductions: raw0.39046, collision
slowdown0.15618, then the operator half multiplier. For the P2 manual-equivalent
profile, cost-based RPP speed scaling and the separate SlowZone multiplier are
therefore removed. Curvature and endpoint approach scaling, real collision STOP,
source expiry, operator takeover and input ramps remain. Generic profiles keep
their prior configuration. This change is not a claim of physical movement.

The operator subsequently requested a0.75 input, explicitly replacing half-mode
for the real short test. The first request estimated0.153m forward but repeated
zeros limited the applied input to0.566. Raw RPP yaw reached0.548, exceeding the
0.24 buffer contract and resetting the complete forward vector. Finite raw yaw
is now bounded at the buffer entry, retaining invalid/nonfinite rejection.
Forward steering reversals now slew through zero without entering an in-place
pivot settling phase; true in-place transitions retain measured settling.
A localization seed no longer inherits the navigation-goal free-cell condition;
finite/in-map checks and all actual goal/footprint checks remain. This permits a
real pose estimate in a map cell that still needs localization, without changing
map occupancy or pretending the location is collision-free.
