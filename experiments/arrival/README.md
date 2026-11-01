# Position targets and stalled final pivots

A map click now sends `heading_mode: position`; checking the arrival-heading
control sends `pose`. Legacy API goals retain pose semantics. Position mode
uses its own planner, behavior tree and XY-only checker with the existing
0.15 m tolerance. The original pose goal and its 0.15 rad yaw tolerance remain.
The E switch, manual takeover, explicit Start, cancel and collision stop are
unchanged.

The position planner first tries checked forward visibility connections through
places where the actual body can pivot. A lattice fallback repairs reference
centres whose interpolated body intersects the actual map, using a private
search copy and checking every result against an unmodified snapshot. It does
not remove obstacles, shrink the body or refresh sensor timestamps. Search
exclusions are not published as physical obstacles. Position mode retains the
lattice's feasible arrival heading, without appending an unnecessary terminal
pivot. It uses the native cached obstacle heuristic and unsmoothed lattice;
the existing RPP preview and command ramps remain on clear routes. After a pursuit collision, the controller tries shorter previews and a bounded forward/pivot velocity window. Every alternative is rechecked over the original time/goal-distance horizon, with no lateral command.

Explicit final turns can assist an observed low-input stall with a bounded
0.5 input pulse (0.55 s), followed by 1.5 s for feedback before another pulse.
This applies only to stalled pure turns; moving turns retain their response.
The assisted output is collision checked, then goes through existing ramps.
Generic profiles default to no assistance.

`check.py` loads the actual plugins on synthetic maps: a wall-adjacent target,
a reversed start into a corner, a sealed route and an occupied body interior.
Accepted paths are checked with the full 55 cm square at 0.01 m/rad intervals.
`check_tracking.py` exercises the actual controller and position goal checker in
8 synthetic delayed-response cases, including confirmed obstacle stop/resume.
It deliberately supplies an inconsistent final yaw to verify position semantics.
`check_pivot.py` uses the actual pure response and native ramp headers: all 24
small-angle model cases must reach and settle within the unchanged tolerance.
It also retains 12 wider-angle comparisons: 6 long-delay cases still overshoot,
with no regression from baseline. These are not full-angle or biped acceptance.

Private frozen-map replay is optional, with explicitly supplied paths; office
maps and traces are never included in this repository or public CI. A snapshot
may contain a person used in a field obstacle test. Snapshot feasibility is not
a verdict on a later live scene, and successful human-obstacle detours must not
be mislabeled as heading oscillation.

Humble's Smac planner binaries bundle common static caches. Link only the
lattice library for this extension; linking both the generic and lattice
binaries caused a reproducible double destruction during teardown, diagnosed
with AddressSanitizer. Normal process teardown is exercised by the fixture.

The optional closed-loop map replay uses the actual planner, controller and goal checker with a planar lag/gain response. Replanning pauses simulated time; it does not establish real-time or biped stability. A frozen reverse-start return still fails after replan and is retained as a failure. A trial preferring retained paths was rejected after a replay body intrusion; it is not part of this controller.
