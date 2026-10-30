#!/usr/bin/env bash
set -eo pipefail
experiment_dir="$(cd "$(dirname "$0")" && pwd)"
nav_workspace="${P2_GAIT_WS:-$(cd "$experiment_dir/../.." && pwd)/gait_nav_ws}"
source /opt/ros/humble/setup.bash
source "$nav_workspace/install/setup.bash"
export LD_LIBRARY_PATH="$nav_workspace/install/p2_smac_gait/lib:${LD_LIBRARY_PATH:-}"
"$nav_workspace/install/p2_nav_gait_trial/lib/p2_nav_gait_trial/gait_contract_regression"
"$nav_workspace/install/p2_nav_gait_trial/lib/p2_nav_gait_trial/gait_hook_regression" "$experiment_dir/biped-straight-pivot32.json" "$nav_workspace/install/p2_smac_gait/lib/libnav2_smac_planner_lattice.so"
