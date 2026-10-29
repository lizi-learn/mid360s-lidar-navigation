#!/usr/bin/env bash
# Explicit, user-prefix build. Does not install system packages or start hardware.
set -eo pipefail
nav_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$nav_root"
source /opt/ros/humble/setup.bash
mkdir -p .deps ros_ws/src sdk
checkout_pinned(){
 local dep_url="$1" dep_commit="$2" dep_dir="$3"
 if [[ ! -e "$dep_dir" ]]; then
  git clone --filter=blob:none --no-checkout "$dep_url" "$dep_dir"
  git -C "$dep_dir" fetch --depth 1 origin "$dep_commit"
  git -C "$dep_dir" checkout --detach "$dep_commit"
 fi
 test "$(git -C "$dep_dir" rev-parse HEAD)" = "$dep_commit" || { echo "Unexpected dependency revision: $dep_dir" >&2; exit 2; }
}
checkout_pinned https://github.com/Livox-SDK/Livox-SDK2.git c0796f04c143143899c87a773d9f6b7136453c0b "$nav_root/.deps/Livox-SDK2"
checkout_pinned https://github.com/Livox-SDK/livox_ros_driver2.git 21445540f0d100dc86a7e6df312dd70bbdb4afdf "$nav_root/.deps/livox_ros_driver2"
checkout_pinned https://github.com/hku-mars/FAST_LIO.git cc3c9e8b1469b33b8072764d6db2348ed6dc597f "$nav_root/.deps/FAST_LIO"
git -C .deps/FAST_LIO submodule update --init --recursive
if ! git -C .deps/FAST_LIO apply --reverse --check "$nav_root/patches/fast_lio_imu_queue.patch" 2>/dev/null; then
 git -C .deps/FAST_LIO apply --check "$nav_root/patches/fast_lio_imu_queue.patch"
 git -C .deps/FAST_LIO apply "$nav_root/patches/fast_lio_imu_queue.patch"
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
cmake -S .deps/Livox-SDK2 -B .deps/sdk-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$nav_root/sdk"
cmake --build .deps/sdk-build --parallel "$CMAKE_BUILD_PARALLEL_LEVEL"
cmake --install .deps/sdk-build
# Avoid upstream build.sh, which removes the workspace's build/install folders.
cp .deps/livox_ros_driver2/package_ROS2.xml .deps/livox_ros_driver2/package.xml
for dep in livox_ros_driver2 FAST_LIO; do
 link_name="$dep"; [[ "$dep" != FAST_LIO ]] || link_name=fast_lio
 target="$nav_root/ros_ws/src/$link_name"
 if [[ ! -e "$target" && ! -L "$target" ]]; then ln -s "$nav_root/.deps/$dep" "$target"; fi
 test "$(readlink -f "$target")" = "$nav_root/.deps/$dep" || { echo "Unexpected existing workspace source: $target" >&2; exit 2; }
done
export LD_LIBRARY_PATH="$nav_root/sdk/lib:${LD_LIBRARY_PATH:-}"
cd ros_ws
colcon build --packages-select livox_ros_driver2 --parallel-workers 1 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DLIVOX_LIDAR_SDK_LIBRARY="$nav_root/sdk/lib/liblivox_lidar_sdk_shared.so" -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$nav_root/sdk/include"
source install/setup.bash
colcon build --packages-select fast_lio --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
