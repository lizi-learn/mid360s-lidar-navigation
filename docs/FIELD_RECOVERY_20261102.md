# Field recovery candidate, 2026-11-02

The supervised office run exposed repeated planning stops, a large unintended right turn during an intended straight segment, and a return that needed manually dispatched short goals. The requested two round trips did not complete. This candidate is software and isolated planar validation, not a new physical robot acceptance.

## Changes

- Honor an explicit leading in-place pivot in the reference before translating. Pure pursuit otherwise looks through repeated-position rotation poses and can start driving while a delayed turn is still developing. Ordinary curved references retain forward/yaw tracking and zero lateral input. New goals and lifecycle transitions reset this phase.
- Once measured yaw response has actually occurred, use measured quiet to finish the pivot coast. Preserve the original wait for a response that has not yet appeared.
- If the ordinary complete position planner fails, try bounded local references internally and concatenate two complete native plans. Recheck the whole swept footprint on the current map. No intermediate navigation action or intermediate arrival stop is dispatched. Occupied destinations and disconnected known-space components remain rejected. Local-reference recovery uses one shared two-second visibility budget rather than launching another lattice repair for each anchor; candidate-edge checks honor the existing visibility deadline. A point-connected synthetic 30 cm opening was rejected for the unchanged 55 cm body in 4.63 s; the validated complete field-return path remains byte-identical.
- Mission dispatch waits within the existing initialization window for actual native readiness instead of failing on a single transient sample. Cancellation, authority loss and missing feedback retain their real outcomes; one Start produces at most one confirmation.
- Record complete source paths, orientations, producer stamps and asynchronous raw costmap bytes privately through a bounded background queue. Rotation, drops and errors are observable. Controller diagnostics identify pursuit versus velocity-window selection. No timestamp is renewed and these diagnostics never decide whether motion is allowed.

## Validation

- 337 local tests; real gzip round-trip, byte identity, bounded queue, rotation and I/O failure coverage for the recorder.
- On a private frozen field costmap, the original long return failed while short exit and later return plans separately succeeded. The candidate produces one complete approximately 41 m path. Independent filled-square SAT sampling at 1 cm / 0.01 rad found zero body intersections using the unchanged 0.55 m square. Inflation values were reconstructed from OccupancyGrid and are quantized; lethal and unknown cells are preserved.
- Actual RPP/planner/progress/goal-checker planar replay with unchanged native input ramp: gain 0.7, delay 0.2 s completed in 93.15 s versus 134.7 s before leading-pivot handling; cross-track RMS 0.164 versus 0.289 m. Delay 0.5 s, previously stuck and timed out, completed in 137.3 s. Both completed with zero body intrusions. These plant gains/delays are experiment parameters, not calibrated biped dynamics.
- Delayed corner and gentle-S component cases completed with zero body intrusions.
- Actual isolated ROS/Nav2 workflow passed continuous transit, observed door reopening, dynamic obstacle detour, directed arrival, middle-switch pause/resume and manual cancellation without automatic restart. The obstacle produced 23 finite ray returns and the model reported zero body collisions.

## Retained limits

The field right turn is proven to have been commanded by navigation while forward input remained nonzero. Full paths for its initiating revisions were not retained, so the exact reference/obstacle trigger is unproven. Later costmaps cannot identify that earlier obstacle or prove it was a moving support frame. The new private recorder addresses this evidence gap.

Earlier failed planner alternatives, mixed-kernel ABI failures, the 120 s incomplete replay and the long-delay timeout are preserved privately. The successful candidate has not yet been deployed to the physical robot. Static planar replay does not establish full-height clearance, dynamic obstacle handling on the field map, or physical gait stability. MID-360S unsynchronized device time still means ROS sensor headers represent host reception time.

E-left permission, E-middle pause with explicit resume, E-right/joystick takeover, cancellation and obstacle stopping remain unchanged. No footprint reduction or new operator confirmation/acceptance-file workflow is introduced. Private maps, office trajectories, recordings, vendor code and credentials are excluded from this repository.
