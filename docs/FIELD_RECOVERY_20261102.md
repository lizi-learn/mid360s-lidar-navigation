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

## Follow-up: health forwarding during path/map callbacks

A subsequent physical Start was accepted, then revoked before effective stepping: the unchanged health flags were120 but their original evaluation was109ms old at the native reply, beyond the existing100ms contract. A separate actual ARM ROS observer measured301 producer samples, maximum26.4ms source gap and6.2ms delivery during a planning-only request. These observations identify a forwarding timing failure; they do not establish which individual Console callback caused the historical delay.

The Console previously serialized all ROS callbacks on one thread. Health now has a dedicated mutually exclusive callback group, a two-thread executor and a depth-one subscription to the latest original evaluation. This does not renew timestamps, loosen expiry or bypass missing health. An actual Console isolation test on ARM blocks unrelated callbacks for250ms: forwarding gaps were253.1ms with the old executor versus33.8ms corrected. Both preserve source stamps. The receipt fixture has no gait socket or actuator access; this is not a physical walking result.

A current-map complete far-goal search still fails. A continuous reference route can be planned when intermediate poses carry travel headings rather than all-zero headings; the subsequent Start exposed the forwarding fault above. Do not classify that route preview or Start acceptance as a completed physical trip.

The shared two-worker candidate was NOT deployed: exact CI41471400317 exposed a260.6ms forwarding gap in the corrected test. The revised implementation assigns the health-only subscriber its own Node and SingleThreadedExecutor/thread; the ordinary Console keeps its original serial executor. Both use the same original source stamps and unchanged100ms contract. The failed CI and ARM-vs-CI discrepancy remain recorded; no physical walking result is inferred.
# Native transport process isolation (November 3)

An actual left-turn recovery advanced about 139 degrees before a separate UI
stop. A focused regression reproduces a time-snapshot bug: a status RPC waits
25 ms while an observer supplies a new original switch sample; using the time
from before that wait misclassifies the sample as being from the future and
publishes pause. The Console now samples time again after blocking work and
uses one consistent current intent for its armed/paused pair.

UI switch observation also reads the exact native packet locally after learning
the source identity from the real controller. It reports an original receiver
observation explicitly, with no fabricated controller-reply timestamp. Controller
readiness remains separately checked by the native contract. Actual E modes,
stick activity, original 100 ms expiry and source changes are retained. A test
delays the read-only RPC path by 250 ms while the actual source remains current;
local observation must stay within the original deadline. A contended snapshot
can retain only its original sample and still expires at 100 ms.

The reply-only refresh candidate passed the initial x86 case but failed the ARM
65 ms control-delay test and its CI job; it was not installed. The source path
now optionally mirrors the exact 64-byte native operator packet after a
successful send, in the private socket directory. The transport reads that
source packet directly for the authority echo, checks the current native
instance and the unchanged 100 ms original timestamp, and otherwise falls back
to the actual gait reply. It never timestamps a snapshot read as a new sample.
The bridge remains a subscriber to the existing receiver; it neither opens the
serial port nor publishes joystick/motor commands. Only its own read-only user
service needs updating; the running D/driver binaries are unchanged.

The delay fixture requires the mirror to be consumed and checks every echoed
timestamp against actual producer packet timestamps. Source identity, expiry,
file permissions and symlink rejection have separate regression coverage.

The first far endpoint was later reached in the field (about 6 cm lidar/map
position error), but return startup revoked the lease. The native trace showed
fresh actual operator input and a roughly 1.3 ms health evaluation, while the
forwarded operator echo was over 116 ms old. Cached authority from before a slow
RPC had been reused. The transport now uses each latest actual reply and
refreshes authority after control and measurement replies. Original operator,
health and measurement sample times and all native deadlines remain unchanged.

The actual C++ lease fixture reproduces a failure with one 65 ms delayed control
reply; the corrected transport sustains a 0.6 policy input and still revokes on
missing health without automatic rearm. A separate 65 ms measurement-delay case
also passes. A development case combining 65 ms measurement and 35 ms control
delay still fails sustained policy input: its measurement transport interval
exceeds the original 100 ms frame budget and resets the response. That failed
case is retained and is not a claimed passing result or a reason to relax the
deadline. Office round-trip completion must be recorded separately.

The corrected control chain subsequently sustained real nonzero strategy input
while lidar odometry advanced about 4.7 m. The office leg then stopped during
replanning; it did not reach its destination. All twelve reference goals were
still present after the first reference had passed behind the robot. The old
0.30 m intermediate-reference radius could be missed between approximately
1.2 s planner updates during continuous walking. Both continuous route trees
now use a 0.75 m reference radius. Final position/heading checking, collision
geometry and stops are unchanged. The actual Nav2 RemovePassedGoals plugin
fixture compares the missed-reference case and verifies that the last goal is
retained. An already passed prefix must also be removed before resuming a
cancelled test; restarting the old complete list would request a return to it.

A real full-route preview after deployment exposed an additional Console
ordering fault: two readers could deliver an older native telemetry snapshot
after a newer snapshot, causing a false raw-source replay cancellation. The
Console now discards duplicate or older snapshots from the same native source
under the session lock. It never renews their original time. Raw operator packet
sequence validation, identity changes, expiry and newer manual takeover remain
unchanged. A regression first reproduces the old cancellation, then verifies
that the corrected reader retains the prepared goal and still accepts a newer
stick takeover. No physical office leg had completed when this was identified.

The independent health callback was insufficient: a subsequent field Start
was accepted and then revoked, with no effective stepping observed. A final
sent health sample was only about 10 ms old, but that asynchronous receipt does
not prove that the native loop used the same sample. Neither this acceptance
nor the previous zero-speed acknowledgements establish physical navigation.

The Console now owns a separate native-control process. That process receives
the original ROS health evaluations and measured velocity directly, and owns
the existing native socket. Map rendering, path handling and other UI Python
work cannot hold its GIL. A private authenticated Unix IPC carries explicit
Start/Stop and status. The IPC key is inherited through a pipe; a second pipe
closes the native client when the parent exits. The D binary, original source
timestamps, lease deadlines, speed settings and E/stick behavior are unchanged.

`experiments/gait_velocity_input/check_process.py` compares the old in-process
transport with the new process using an actual compiled C++ contract fixture,
a separate ROS/operator producer and six 250 ms UI GIL stalls. The old version
loses ownership; the new version retains ownership and a 0.6 policy input. The
fixture also checks original health and operator expiry, middle pause, right
manual mode, stick takeover, no automatic rearm and parent exit. It addresses
no robot hardware. Actual office-route completion remains a separate field
result and must be recorded as such.


An additional session polling race was reproduced with a current native observer sample arriving after the UI read its evaluation clock but before it acquired the session lock. The old caller stopped a valid executing session; the corrected caller takes evaluation time under the session lock. Source timestamps and deadlines are unchanged. The comparison regression fails on c9a5cfc and passes with 354 unit checks. Physical return remains incomplete at this checkpoint.


The first atomic-clock candidate did not complete the physical return. A joint Console/transport test then reproduced stale UI observations after a 150-ms UI request: original native data stayed available, but the session emitted a paused velocity intent from its earlier observation. Each session decision now consumes the latest original native packet immediately before evaluation. The old e5ff81d comparison fails. An intermediate 72fccff candidate failed a health-forwarding fixture and was not deployed; the corrected version retains the nonblocking health-forwarding path and passes 355 checks. No source sample timestamp or native expiry is changed; the independent controller remains authoritative for actual source loss and takeover. This is not a completed office return claim.
