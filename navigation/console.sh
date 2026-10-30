#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
nav_controller_setup="${P2_NAV_CONTROLLER_SETUP:-$root/../nav_controller_ws/install/setup.bash}"
if [[ -f "$nav_controller_setup" ]]; then source "$nav_controller_setup"; fi
set -u
export OPENBLAS_NUM_THREADS="${P2_NAV_BLAS_THREADS:-1}"
export OMP_NUM_THREADS="${P2_NAV_OMP_THREADS:-1}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-194}"
export PYTHONPATH="/usr/lib/python3/dist-packages:$root:${PYTHONPATH:-}"
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 -S "$root/console.py" "$@"
