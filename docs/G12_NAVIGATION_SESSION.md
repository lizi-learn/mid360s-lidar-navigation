# G12 E authority and two-step navigation session

The physical E switch permits a **new explicit start** on the left. Middle pauses
and zeros navigation intent. Right yields manual ownership. Raw stick activity
also cancels: returning to neutral/left never resumes the old goal. The current
middle behavior is a conservative default, not a separately confirmed preference.

The touch console supports pan, pinch/wheel/button zoom, map/pose focus, tap goal,
preview, explicit confirmation, cancel, speed cap and blocked-route notification.
A paired Android client can embed the same-origin workbench inside its APP and
switch to that tab on a newly verified left position. This repository does not
publish the private full controller APP, factory program or office data.

`operator_bridge.py` observes an existing receiver ROS stream. It never opens a
serial port, writes joystick memory, or publishes motor commands. Commissioning
must match machine/installation identity, receiver executable SHA and actual
physical switch observation. Host receiver timestamps are preserved, not device
sampling timestamps. A >20ms wall/monotonic offset change latches invalid;
100ms sample expiry, replay, malformed flags and unknown raw positions reject.
Humble one-argument callbacks use sole-publisher discovery for UI provenance;
this observer is **not** a substitute for arbitration in the gait control loop.

`NavigationSession` carries goal generations through planning, confirmation,
action acceptance and terminal results. Cancellation survives late acceptance,
source change and stale input. Sending a cancel RPC is not a terminal receipt.
`PhysicalSpeedInput` is an unconnected C++ arbitration prototype with persistent
cancel-generation acknowledgement, not a deployed factory controller hook.

## Collision receipt on locked Humble

Nav2 1.1.20 computes STOP in its collision monitor but does not publish the
`state_topic` consumed by this application. Verified against [upstream
source](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/collision_monitor_node.cpp).
`observed_collision_monitor` subclasses it and publishes its actual internal
action after the unchanged `process()` returns. It neither reconstructs STOP
from velocity nor changes the collision algorithm, footprint or stop zone. The
polygon name is empty because Humble does not retain that accessible attribution.
The lifecycle follows the original component. A missing monitor receipt is not a
successful obstacle test. Recovery behavior waits/replans; it never invokes
ClearEntireCostmap to manufacture a route.

A STOP freezes navigation intent while Nav2 continues planning. Resume requires a
new fresh path and continuous observed clearance for .75s, fresh left authority
and healthy sensors/localization/ground. Three blocked new plans or 8s without a
route cancels with a zero navigation intent and an operator notice.

## Reproduction and limits

Build with `bash scripts/build-nav-controller.sh` (Humble 1.1.20 and
`ros-humble-nav2-collision-monitor` required). In an isolated ROS domain with full
Nav2 runtime installed, source ROS and the built overlay and run:

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197
export P2_NAV_CONTROLLER_SETUP="$PWD/nav_controller_ws/install/setup.bash"
source /opt/ros/humble/setup.bash
source "$P2_NAV_CONTROLLER_SETUP"
python3 experiments/g12_session/check.py --directory /tmp/g12-session-evaluation
```

Use a fresh directory; do not run this alongside another group on the same
8956 HTTP port/domain. Bound the command to 170s externally. The harness stops
its own launch children in finally and stores actual parameters, requests,
Nav2 lifecycle states, STOP action receipts, scene controls/ack and source SHA.

2026-10-31 isolated planar run passed: preview/cancel, right takeover with no
resume, middle pause, raw-stick intervention, confirmed goal arrival, acknowledged
obstacle insertion, actual monitor STOP, bounded blocked notice with zero output,
and zero full-envelope overlap attempts. Seed=0, deterministic synthetic raycast
scene. This is **not P2 balance, RL tracking or real walking acceptance**. Narrow
door/turn/tracking failures and original D13 model failures remain unresolved.

Earlier attempts are retained privately: missing planner overlay; planning before
navigation lifecycle Active; missing truth instrumentation; genuine STOP observed
in log but no machine-readable Humble state. Only the final run with all joint
conditions passed. Physical 1006 E commissioning and native ARM build are separate
from this synthetic acceptance.

Live web execution remains hard-disabled. Physical D13 transport, policy/mode
feedback, body/neck extrinsics, near-ground coverage and actual stopping response
must be commissioned before enabling hardware walking. Current P2 candidate
forward cap .15m/s, lateral zero, full footprint/stop/ground/localization/stale
checks are retained. APP installation and map preview must not be called complete
navigation acceptance.
