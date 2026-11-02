# Field recovery candidate, 2026-11-02

The supervised office run exposed repeated planning stops, a large unintended right turn during an intended straight segment, and a return that needed manually dispatched short goals. The requested two round trips did not complete. This candidate is software and isolated planar validation, not a new physical robot acceptance.

## Sensor projection correction

The adapter previously converted a registered cloud using the latest odometry pose, while retaining the older cloud stamp. Downstream TF therefore transformed polar returns using a different pose. The fix interpolates only measured, original-stamped poses at the cloud time, waits for a real bracket without extrapolation, and publishes a cloud-stamped `laser_projection` frame. Latest odometry output remains independent. The existing source expiry and unknown-ray rules remain unchanged.

Angular projection now uses the nearest circular bin at 720 bins rather than always rounding downward at 360 bins. This removes a systematic clockwise bias and bounds angular quantization to a quarter degree (about 4.8 cm at 11 m).

A real two-domain ROS adapter/TF regression with a fixed synthetic reflector, 100 ms cloud delay, 0.4 m/s translation and 0.8 rad/s yaw reproduced a mean displacement of 45.6 cm in the old adapter versus 1.47 cm corrected (maximum 2.35 cm). All output scans retained original cloud stamps. This proves the software defect; the original field cloud/pose pair was not preserved and its precise contribution to that field turn remains unproven.

## Path reuse consistency

The reuse branch previously required an extra4cm margin even though fresh planning could accept the unchanged original body at zero extra margin. It now tries the preferred margin and then revalidates the entire original filled/swept footprint. Occupied and unknown cells remain rejected; body size and obstacle stopping do not change. This is a route preference fallback, not a smaller footprint or a new operator workflow. A dedicated native60cm-corridor test actually exercised the original-body reuse branch, completed the whole reference with zero body intersections, then inserted48 real costmap obstacle cells and verified the route was rejected. The separate passing41m return models did not exercise this branch and are not cited as proof of the reuse fix.

## Changes

- Honor an explicit leading in-place pivot in the reference before translating. Pure pursuit otherwise looks through repeated-position rotation poses and can start driving while a delayed turn is still developing. Ordinary curved references retain forward/yaw tracking and zero lateral input. New goals and lifecycle transitions reset this phase.
- Once measured yaw response has actually occurred, use measured quiet to finish the pivot coast. Preserve the original wait for a response that has not yet appeared.
- If the ordinary complete position planner fails, try bounded local references internally and concatenate two complete native plans. Recheck the whole swept footprint on the current map. No intermediate navigation action or intermediate arrival stop is dispatched. Occupied destinations and disconnected known-space components remain rejected. Local-reference recovery uses one shared two-second visibility budget rather than launching another lattice repair for each anchor; candidate-edge checks honor the existing visibility deadline. A point-connected synthetic 30 cm opening was rejected for the unchanged 55 cm body in 4.63 s; the validated complete field-return path remains byte-identical.
- Mission dispatch waits within the existing initialization window for actual native readiness instead of failing on a single transient sample. Cancellation, authority loss and missing feedback retain their real outcomes; one Start produces at most one confirmation.
- Record complete source paths, orientations, producer stamps and asynchronous raw costmap bytes privately through a bounded background queue. Rotation, drops and errors are observable. Controller diagnostics identify pursuit versus velocity-window selection. No timestamp is renewed and these diagnostics never decide whether motion is allowed.

## Validation

- 344 local tests; real gzip round-trip, byte identity, bounded queue, rotation and I/O failure coverage for the recorder.
- On a private frozen field costmap, the original long return failed while short exit and later return plans separately succeeded. The candidate produces one complete approximately 41 m path. Independent filled-square SAT sampling at 1 cm / 0.01 rad found zero body intersections using the unchanged 0.55 m square. Inflation values were reconstructed from OccupancyGrid and are quantized; lethal and unknown cells are preserved.
- Actual RPP/planner/progress/goal-checker planar replay with unchanged native input ramp: gain 0.7, delay 0.2 s completed in 93.15 s versus 134.7 s before leading-pivot handling; cross-track RMS 0.164 versus 0.289 m. Delay 0.5 s, previously stuck and timed out, completed in 137.3 s. Both completed with zero body intrusions. These plant gains/delays are experiment parameters, not calibrated biped dynamics.
- Delayed corner and gentle-S component cases completed with zero body intrusions.
- Actual isolated ROS/Nav2 workflow passed continuous transit, observed door reopening, dynamic obstacle detour, directed arrival, middle-switch pause/resume and manual cancellation without automatic restart. The obstacle produced 23 finite ray returns and the model reported zero body collisions.

## Retained limits

Additional frozen post-stop-map control replay still stalls near the endpoint. Forward-reference, moving low-input and larger-radius prototypes failed their full-goal checks and were not adopted. These failures remain separate from the passing return/synthetic cohorts.

The field right turn is proven to have been commanded by navigation while forward input remained nonzero. Full paths for its initiating revisions were not retained, so the exact reference/obstacle trigger is unproven. Later costmaps cannot identify that earlier obstacle or prove it was a moving support frame. The new private recorder addresses this evidence gap.

Earlier failed planner alternatives, mixed-kernel ABI failures, the 120 s incomplete replay and the long-delay timeout are preserved privately. The successful candidate has not yet been deployed to the physical robot. Static planar replay does not establish full-height clearance, dynamic obstacle handling on the field map, or physical gait stability. MID-360S unsynchronized device time still means ROS sensor headers represent host reception time.

E-left permission, E-middle pause with explicit resume, E-right/joystick takeover, cancellation and obstacle stopping remain unchanged. No footprint reduction or new operator confirmation/acceptance-file workflow is introduced. Private maps, office trajectories, recordings, vendor code and credentials are excluded from this repository.
