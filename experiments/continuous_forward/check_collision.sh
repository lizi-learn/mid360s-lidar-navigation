#!/usr/bin/env bash
set -eo pipefail
experiment_dir="$(cd "$(dirname "$0")" && pwd)"
nav_workspace="${P2_CONTINUOUS_WS:-$(cd "$experiment_dir/../.." && pwd)/continuous_nav_ws}"
source /opt/ros/humble/setup.bash
source "$nav_workspace/install/setup.bash"
fixture="$experiment_dir/biped-straight-pivot8.json"
env LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}" ros2 run p2_nav_swept_trial lattice_sweep_regression stock "$fixture"
env LD_LIBRARY_PATH="$nav_workspace/install/p2_smac_continuous/lib:${LD_LIBRARY_PATH:-}" ros2 run p2_nav_swept_trial lattice_sweep_regression patched "$fixture"
