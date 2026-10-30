#!/usr/bin/env bash
# Build our optional Humble extension in a separate workspace. No hardware starts.
set -eo pipefail
nav_root="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
for name in ['nav2_regulated_pure_pursuit_controller','nav2_smac_planner']:
 p=Path(get_package_share_directory(name))/'package.xml'
 v=ET.parse(p).getroot().findtext('version')
 if v!='1.1.20':raise SystemExit('This extension was validated with Nav2 Humble 1.1.20; '+name+' is '+str(v)+'. Review compatibility before building.')
PY
nav_workspace="$nav_root/nav_controller_ws"
mkdir -p "$nav_workspace/src"
nav_target="$nav_workspace/src/p2_nav_controller"
if [[ ! -e "$nav_target" && ! -L "$nav_target" ]]; then ln -s "$nav_root/navigation/p2_nav_controller" "$nav_target"; fi
test "$(readlink -f "$nav_target")" = "$nav_root/navigation/p2_nav_controller" || { echo 'Unexpected existing controller source.' >&2; exit 2; }
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
cd "$nav_workspace"
colcon build --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
