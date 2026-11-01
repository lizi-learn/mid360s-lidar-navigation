# P2 heading preview comparison

The P2 profile changes only its velocity-scaled preview bounds from0.4–0.8m to
1.2–1.6m. The existing2s lookahead time, forward input selection, yaw input limit1.0,
forward/yaw blending, zero lateral input, collision checks, body dimensions,
operator takeover and source expiry remain. The generic profile is unchanged.

A supervised field out-and-back run showed repeated heading corrections before
a separately confirmed human detour near its return endpoint. The recorded
controller request and measured heading rate had about1s of correlation lag;
this is descriptive closed-loop evidence, not an identified physical plant.
The native RPP [velocity-scaled preview calculation](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)
uses measured forward speed, so raising only the maximum leaves a short preview
at startup and after a stop. That first candidate,0.6–1.6m, still failed several
synthetic cases and was rejected. Its results are retained locally.

The component instantiates the actual own RPP plugin and native SimpleGoalChecker,
reads parameters and footprint back from the configured nodes, and integrates a
synthetic planar body. It includes the actual pure-arithmetic `SpeedResponse`
input ramp plus an explicit gain, delay and first-order response. There is no
robot model, gait policy, localization filter, perception or navigation planner.
The two hypothetical gain/delay pairs are1.2/0.3s and0.8/0.5s, with fixed seed0;
they are sensitivity cases, not calibrated physical dynamics. The robot is never
contacted. Tests require localhost ROS in a domain>=200.

Each profile follows a20m straight, a gentle S, a90deg L corridor and a straight
with a confirmed obstacle injected into the actual costmap. The obstacle stays
for2s; the required behavior is collision stop and resume after its removal,
not replanning around a person. The full55cm square is independently checked
against full occupied cells by SAT at10ms interpolated intervals. Passing means
that this sampled model reached the original0.15m/0.15rad goal, had zero body
intrusions, and actually triggered the obstacle stop where required. It does
not certify continuous physical clearance or stopping performance.

All8 candidate cases passed locally. Straight steady-state RMS reference error
was0.0021/0.0140m in the two synthetic models, with no reverse heading corrections
above the fixture threshold after5s; the largest candidate RMS among all cases
was0.0618m. These values are **not real-robot accuracy**. The short-preview
baseline failed several goals and touched a corridor wall in the same models;
those failed rows remain in the output. Counts from different goal durations
must not be presented as a same-duration percentage reduction.

Build after `scripts/build-nav-controller.sh` and sourcing its install prefix:

```sh
cmake -S experiments/heading_preview -B /tmp/heading-preview-build
cmake --build /tmp/heading-preview-build -j2
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=231 python3 experiments/heading_preview/check.py \
  --binary /tmp/heading-preview-build/heading_preview \
  --output /tmp/heading-preview/comparison.json
```

CI keeps the full requested/actual parameters, every case, traces and failures.
This parameter change does not fix the previously observed low-input terminal
pivot stall. Physical heading smoothness still needs a new supervised run.
