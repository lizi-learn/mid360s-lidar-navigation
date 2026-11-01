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
