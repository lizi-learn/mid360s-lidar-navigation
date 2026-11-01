# Keep long paths outside the command heartbeat process

The physical workbench now receives its live path view from a small separate ROS
process. Nav2 continues to use the original `nav_msgs/Path`; its planner,
controller and collision geometry are unchanged. The relay sends **every** XY
point and the original ROS path timestamp as a compact JSON message. It does
not shorten the route, smooth obstacles, restamp data or certify clearance.
The workbench keeps the same source-age check and obstacle reroute behavior.
Simulation and workbenches without a physical gait connection retain the direct
Path subscription.

The command RPC loop also releases its state lock before waiting for a socket
reply. A slow measured-velocity reply can no longer prevent fresh gate callbacks
or cancellation from acquiring that lock. After a slow reply, a new authority
packet forwards the latest actually received gate decision with its original
source time. The native 100 ms health limit, measured source expiry, operator
priority and no-auto-rearm behavior are unchanged. The workbench uses a single
ROS executor thread for its existing mutually exclusive callback group.

A controlled blocked-socket test reproduced the old callback/Cancel lock stall;
the candidate permits those callbacks to complete while the reply remains
blocked. The real ROS component test sent synthetic 812-point paths between
processes and checked every coordinate and original source timestamp. These are
component checks, not physical navigation acceptance.

Field testing before the relay moved an estimated 6.1 m over about 30 seconds,
then cancelled on a navigation heartbeat gap. After the relay, a subsequent
134-second request retained native navigation ownership until an explicit Stop;
it stalled at an in-place turn with a 0.24 policy yaw input and negligible body
rotation. This is evidence that further gait-input adaptation is needed, not a
completed office route or a claim that every possible timing gap is eliminated.
