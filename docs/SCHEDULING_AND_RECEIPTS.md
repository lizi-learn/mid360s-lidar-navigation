# Planner load and command receipts

With a physical gait connection, the workbench now launches the ROS workflow at
its inherited normal priority. AMCL, the lattice planner and mapping SLAM lower
only their own priority with `nice -n 10`. The command buffer, final gate,
controller, smoother, observation adapter and collision monitor no longer
inherit the planner's lower priority. This needs no realtime policy, negative
nice value, root process scan or change to the motor driver. A parent already
started at a lower priority still requires its service configuration to be
corrected; a child cannot undo that without privileges.

The launch argument is `physical_gait:=true`, selected automatically by the
workbench when its physical gait transport is configured. Generic launches keep
their existing normal scheduling. Operator actions and source expiry remain
unchanged; there is no extra confirmation or automatic rearm.

The private bounded velocity recorder now includes original health and measured
velocity source times, health flags, controller instance, policy generation,
readiness, armed/owned state and the last authority exchange. Phase, reason and
readiness changes are recorded even between the usual 10 Hz periodic samples.
Recording wall and monotonic times are explicitly separate from source times.
The bounded queue and background file writer remain outside socket exchanges;
recording neither refreshes data nor changes command output.

Local component validation used a synthetic 46 m corridor, a 40 m planning query,
four shared CPU cores and eight bounded background compute workers. The actual
ROS planner returned 791 poses. The repaired launch recorded 1,369 original gate
evaluations over 28 seconds, with maximum evaluation interval 24.88 ms and no
interval above 100 ms. Actual process priorities matched the requested split;
no navigation action, ARM or nonzero velocity was sent. The old global-nice
baseline also had no 100 ms gap in this test, so this experiment verifies launch
behavior and timing in this fixture, not reproduction or elimination of every
physical startup timeout. It does not test biped balance, stopping distance,
whole-route navigation or physical sensor failure recovery.
