# Optional gait-space planning experiment

This is an opt-in **planning** extension for Nav2 Humble 1.1.20. It does not
replace the default planner/controller or establish complete biped navigation
or physical acceptance. The current narrow-door tracking problem remains open.
It contains no vendor models, policies, robot recordings or office maps.

The planner retains the continuous full-footprint checks from
[`continuous_forward`](../continuous_forward/README.md), ranks valid states by
the center inflation potential, and rejects paths whose accumulated absolute
rotation exceeds 90 degrees without a continuous 0.50 m forward leg. These are
experimental model constraints, not a universal biped turning specification.
The 0.30 m terminal suffix keeps the existing XY-first terminal contract; it is
still physically collision checked and is not a terminal gait acceptance claim.

Search repairs add bounded, query-local, thread-local angular-transition
exclusions at the **actual violating edge**. Exclusions never modify the physical
costmap. This conservative search is not complete and does not prove optimality.
It may reject a feasible route. All safety footprints, padding, parking zones,
source/command expiration gates and the 0.15 m/s candidate limit are retained.

Build and run the component contracts in an isolated workspace:

```bash
bash experiments/gait_space/build.sh
bash experiments/gait_space/check.sh
```

The build fetches ten Apache-2.0 Nav2 source files pinned by commit and SHA-256,
then applies the published patch. See `p2_smac_gait/upstream-provenance.json` and
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md). It starts no ROS graph
or hardware interface. Use one experimental SMAC overlay per shell/process.
Both overlays intentionally provide the same lattice SONAME; mixing them is
unsupported. `check.sh` verifies the actually loaded library identity.

To generate a planner-only profile:

```bash
python3 experiments/gait_space/configure.py --frozen-replay --output /tmp/gait-plan.yaml
```

For a read-only reproduction, source `gait_nav_ws/install/setup.bash`, start the
normal navigation simulation with that profile and
`experiments/gait_space/fixtures/frozen.yaml`, then run:

```bash
python3 experiments/gait_space/replay_queries.py --output /tmp/new-gait-replay
```

Use a fresh output directory. The client checks the exact synthetic obstacle
mask, complete stopping footprint, actual immutable gait bounds and a nonempty
all-zero safe-command stream. It only requests `ComputePathToPose`; it does not
arm navigation or request `NavigateToPose`.

The frozen regression includes the retained failure: attaching an exclusion to
the earlier rotation-budget origin missed an over-budget edge after several
short forward legs. Both query outcomes changed from `[4, 6]` to `[4, 4]` after
anchoring the actual edge. Eight nonempty independent filled-footprint and Nav2
C++ boundary checks found zero invalid samples at both map resolutions, before
and after. See `validation-2026-10-30.json`. These are planning results only.

Executing the planned turns still requires validated gait tracking, source-
synchronous leg/body feedback and per-machine calibration. No controller
experiment or ideal model body feedback is included here.
