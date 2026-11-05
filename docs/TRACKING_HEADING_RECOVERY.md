# Turn recovery after a locally aborted navigation action

A successful global preview does not prove the controller has a clear forward
arc at every later pose. In a numbered field tour, local tracking stopped at a
front corner although the global route remained valid. An independently
requested left observation turn changed measured yaw by about 27 degrees before
the real Spin collision result stopped it; replanning the original visit then
allowed the remaining visits to complete. That intervention is retained as an
interruption, not an uninterrupted autonomous tour pass.

The mission adapter now connects this failure path to the existing native Spin
recovery. It requires the real terminal action receipt for the current goal:
`STATUS_ABORTED` (6), navigation rather than rotation, the same mission epoch and
goal generation, and continuing operator authority. A notice string alone is not
a receipt. Cancelled actions, source/operator stops, replaced goals and an active
dispatch worker do not grant recovery.

For each visit leg, one recovery sequence can try the preferred 30-degree turn
and the opposite direction. Both use the existing Spin action, live costmap,
complete footprint, actual acceptance and terminal receipts. Fresh scan returns
choose a preference only; they never authorize clearance. A blocked Spin with
less than 0.03 rad measured yaw change proceeds to the opposite checked
direction before reusing a global plan. This progress check avoids repeating a
valid-but-untrackable plan after a stationary rejection. It is not a final goal
accuracy requirement.

After a genuine partial or completed turn, the adapter replans the same target,
arrival heading, remaining transit points and presentation actions. A completed
turn is never an arrived visit. A second tracking abort on the same leg ends the
task instead of spinning indefinitely. Explicit pause/resume, E-right, joystick
takeover, cancellation, sensor freshness and the native gait protections retain
their existing behavior. No direct motor command, obstacle clearing, footprint
reduction or timestamp renewal is introduced.

Focused tests exercise real Console callback status forwarding, mission
dispatch order, partial turns, two stationary rejections, once-per-leg
recovery, passed-waypoint receipts and cancellation races. These are software
tests. The new automatic link still needs a fresh physical tour; completion of
the interrupted earlier tour does not validate this newly added link.
