#!/usr/bin/env bash
set -euo pipefail
nav_root="$(cd "$(dirname "$0")" && pwd)"
cd "$nav_root"
if [[ -f .env ]]; then set -a; source .env; set +a; fi
export OPENBLAS_NUM_THREADS="${P2_NAV_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${P2_NAV_OMP_THREADS:-1}"
nav_profile="${P2_NAV_ROBOT_PROFILE:-$nav_root/config/robot.local.json}"
if [[ -f "$nav_profile" ]]; then
  nav_runtime="$(python3 "$nav_root/robot_profile.py" --profile "$nav_profile" --runtime "${P2_NAV_RUNTIME:-}" --runtime-base "$nav_root/runtime/robots" --sensor-config "$nav_root/config/MID360S.json")"
  export P2_NAV_ROBOT_PROFILE="$nav_profile"
else
  if [[ -n "${P2_NAV_ROBOT_PROFILE:-}" ]]; then echo 'Selected robot profile is missing.' >&2; exit 2; fi
  nav_runtime="${P2_NAV_RUNTIME:-$nav_root/runtime}"
fi
export P2_NAV_RUNTIME="$nav_runtime"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-188}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export VIEW_PORT="${VIEW_PORT:-8920}"
export LD_LIBRARY_PATH="$nav_root/sdk/lib:${LD_LIBRARY_PATH:-}"
test -f config/MID360S.json || { echo 'Run scripts/configure.py first.' >&2; exit 2; }
test -f ros_ws/install/setup.bash || { echo 'Build the pinned ROS dependencies first.' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "$nav_root/ros_ws/install/setup.bash"
set -u
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
mkdir -p "$nav_runtime"
nav_children=()
cleanup(){ for nav_pid in "${nav_children[@]}"; do kill -TERM "$nav_pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT
trap 'exit 0' INT TERM
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:="$nav_root/config/MID360S.json" >"$nav_runtime/livox.log" 2>&1 &
nav_children+=("$!")
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$nav_root/config/mid360s_lio.yaml" >"$nav_runtime/fastlio.log" 2>&1 &
nav_children+=("$!")
/usr/bin/python3 -S "$nav_root/nav_server.py" --bind "${VIEW_BIND:-127.0.0.1}" --port "$VIEW_PORT" >"$nav_runtime/view.log" 2>&1 &
nav_children+=("$!")
/usr/bin/python3 -S "$nav_root/sensor_watchdog.py" >"$nav_runtime/watchdog.log" 2>&1 &
nav_children+=("$!")
set +e
wait -n
exit 1
