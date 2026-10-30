#!/usr/bin/env bash
# Builds isolated experimental planner packages. Starts no ROS graph or hardware.
set -eo pipefail
experiment_dir="$(cd "$(dirname "$0")" && pwd)"
nav_workspace="${P2_GAIT_WS:-$(cd "$experiment_dir/../.." && pwd)/gait_nav_ws}"
source /opt/ros/humble/setup.bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
p=Path(get_package_share_directory('nav2_smac_planner'))/'package.xml'
if ET.parse(p).getroot().findtext('version')!='1.1.20':
    raise SystemExit('This experiment requires the reviewed Nav2 Humble 1.1.20 headers and ABI.')
PY
mkdir -p "$nav_workspace/src"
for package in p2_smac_gait p2_nav_gait_trial; do
  nav_target="$nav_workspace/src/$package"
  if [[ ! -e "$nav_target" && ! -L "$nav_target" ]]; then
    ln -s "$experiment_dir/$package" "$nav_target"
  fi
  test "$(readlink -f "$nav_target")" = "$experiment_dir/$package" || { echo 'Unexpected existing experiment source.' >&2; exit 2; }
done
# MAKEFLAGS is necessary as colcon's Make backend can add its own parallel flags.
export MAKEFLAGS='-j1 -l1'
export CMAKE_BUILD_PARALLEL_LEVEL=1
cd "$nav_workspace"
colcon build --executor sequential --parallel-workers 1 --packages-select p2_smac_gait p2_nav_gait_trial --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
