#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-194}"
export PYTHONPATH="/usr/lib/python3/dist-packages:$root:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
mkdir -p "$root/../runtime/navigation"
exec ros2 launch "$root/stack.launch.py" "$@"
