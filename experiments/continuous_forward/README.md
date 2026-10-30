# Continuous forward lattice experiment

This isolated planning experiment fixes reproducible collision-search defects and
adds finer forward-only headings. It does not establish complete narrow-door
navigation or physical biped acceptance. The normal launch profile is unchanged.

The search checks pure-rotation intermediate poses, continuous subcell positions
and exact footprint headings. A second check samples the entire returned path,
the actual start connection and the terminal connection against the original
map, including the filled rectangular footprint. Bounded repair searches can
exclude invalid reference centers; they never erase or change physical obstacles.

The nominal P2 padded stopping rectangle remains `[-0.63,-0.77]` to `[1.05,0.77]`
metres. Its circumradius is about 1.3021m. The generated experimental global
inflation radius covers that radius plus one map cell (1.4m for this profile).
The 32 heading directions are exact integer-grid forward rays with in-place
pivots. They include no commanded lateral or reverse motion. The nonuniform
heading table uses a one-cell lookup setting to avoid an invalid rotated angular
lookup; the two-dimensional obstacle heuristic remains active.

The build fetches ten SHA-256 checked Apache-2.0 source files from Nav2 commit
`a097086719c88f781aa59788eca29ac6ca5e56db`, applies the included patch without
fuzz and keeps their copyright notices. It requires installed Humble 1.1.20
headers. Sources are fetched into the build directory, never into this checkout.
The overlay selects one lattice shared library; loading multiple SMAC libraries
with duplicate static caches is unsupported.

From the repository root, on an isolated ROS Humble installation:

```bash
bash scripts/build-nav-controller.sh
bash experiments/continuous_forward/build.sh
bash experiments/continuous_forward/check_collision.sh
python3 experiments/continuous_forward/configure.py --output /tmp/p2-continuous.yaml
```

`P2_CONTINUOUS_WS` can select a separate build directory. The collision regression
verifies clear endpoint footprints and a lethal intermediate footprint. The
installed stock library accepts that transition; the patched library rejects it.
Both runs check the actual loaded library identity and fail on wrong results,
including in Release builds.

To evaluate a navigation graph, explicitly source `continuous_nav_ws/install/setup.bash`
and supply the generated YAML to an isolated simulation. The baseline controller
is retained. This build does not start a graph or send any robot commands.
Keep the 0.15m/s forward cap, physical and stopping footprints, freshness,
localization and ground gates. An unavailable route is a failed route.

Frozen synthetic-map planning recovered two previously failed off-center
corridor queries; independent filled-footprint and Nav2 checks found no invalid
samples in those returned paths. Full gait tracking trials still failed at
narrow passages or fine turning. Later stop/dwell and phase feedback prototypes
are private experiments and are not enabled here. Model dynamics, policies,
robot descriptions, office maps, recordings and credentials are not included.

The authored synthetic frozen mask is included as `frozen-synthetic.yaml` and
`frozen-synthetic.pgm`; its source and mask digests are in
`validation-2026-10-30.json`. To reproduce the read-only 32-heading queries,
source the baseline and experimental install setups in both terminals, then use
an otherwise unused localhost domain:

```bash
export ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=1
unset P2_NAV_NATIVE_EXEC P2_NAV_NATIVE_CONFIG P2_NAV_NATIVE_MODEL
python3 experiments/continuous_forward/configure.py --frozen-replay --headings 32 --output /tmp/p2-frozen.yaml
ros2 launch navigation/stack.launch.py mode:=navigation simulation:=true params:=/tmp/p2-frozen.yaml map:="$PWD/experiments/continuous_forward/frozen-synthetic.yaml"
```

In the second terminal, using the same domain:

```bash
python3 experiments/continuous_forward/replay_queries.py --output /tmp/p2-frozen-replay-32
```

The replay client sends only `ComputePathToPose`, verifies the actual full
stopping footprint and unchanged lethal mask, saves before/after costmaps and
paths, and requires observed safe commands to stay zero. Output directories
must be new, preserving failed runs. Stop this graph before repeating with
`--headings 8` and a new result directory; the two recorded 8-heading queries
are expected to fail. This is a planning regression, not an avoidance run.
