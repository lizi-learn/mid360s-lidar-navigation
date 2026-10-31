# Collision observation validity

This bounded ROS component test publishes only `/collision_source_guard/*`
topics in a localhost-only domain of 200 or higher. It loads no robot, gait
controller, office map or physics model, and uses stationary synthetic TF.

The installed Nav2 Humble 1.1.20 source API returns no success flag when
projection fails. The observed monitor checks each enabled source before
publishing velocity and keeps the native polygon decision helpers. Missing,
expired or untransformable observations produce zero velocity, a STOP decision
and false source validity. At least one enabled source must supply projected
endpoints. This stack's height-sliced scan contract treats an all-NaN or
all-infinite scan as unknown, rather than evidence of clear space. The existing
source timeout, TF tolerance, footprint and stop polygons are unchanged.

The downstream gate also requires recent valid source health. A fault disarms
it and revokes the current goal's `arm_generation`; repeated arm messages for
that generation cannot resume it. The console cancels the goal and a subsequent
goal confirmation must carry a newer generation. The Python gate and console
regressions exercise this latch separately from the monitor component test.

Run after building the extension:

```bash
source /opt/ros/humble/setup.bash
source nav_controller_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=215
python3 experiments/collision_source_guard/check.py \
  --binary "$PWD/nav_controller_ws/install/p2_nav_controller/lib/p2_nav_controller/observed_collision_monitor" \
  --output /tmp/collision-source-guard.json
```

The seven cases are startup without observations, clear finite returns, a real
synthetic obstacle inside the unchanged stop polygon, an all-NaN scan, expired
source stamps, missing sensor TF, and recovery to finite clear returns. The
report saves actual output sample counts, velocity maxima, decisions and source
validity; adjacent files preserve exact parameters and node logs.
`--expect-old-defect` is only for reproducing the old binary's unsafe component
behavior, not a passing criterion for the corrected binary.

These checks validate failure handling at this component boundary. They do not
validate biped balance, physical calibration, stopping distance, current sensor
mounting angle, full navigation, or repair the underlying cause of TF lag.
