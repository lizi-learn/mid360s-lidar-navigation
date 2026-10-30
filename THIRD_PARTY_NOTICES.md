# Third-party components

The original Python application, UI and documentation in this repository are
MIT-licensed. Upstream dependencies are fetched separately and are not relicensed
by this repository. No robot vendor controller, private IMU driver or binary is
included.

| Component | Source | Notice |
| --- | --- | --- |
| Livox SDK2 | https://github.com/Livox-SDK/Livox-SDK2 | Retain the upstream LICENSE.txt and notices for the pinned revision. |
| Livox ROS Driver2 | https://github.com/Livox-SDK/livox_ros_driver2 | MIT; copyright Livox. Retain its LICENSE.txt. |
| FAST-LIO ROS2 | https://github.com/hku-mars/FAST_LIO/tree/ROS2 | Upstream root LICENSE is GNU GPL v2. Its package.xml has a different legacy label; this project follows the root LICENSE for its patch. |
| ikd-Tree | https://github.com/hku-mars/ikd-Tree | FAST-LIO submodule; retain its upstream license and copyright. |
| NumPy / SciPy | https://numpy.org/ / https://scipy.org/ | Separately installed BSD-licensed dependencies; preserve their distribution notices. |
| Nav2 | https://github.com/ros-navigation/navigation2 | Separately installed; retain package-specific licenses. Much of Nav2 is Apache-2.0; nav2_amcl includes LGPL-2.1-or-later code. |
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox | Separately installed; LGPL-2.1-only and upstream component notices apply. |
| ROS 2 / PCL / Eigen | Their upstream distributions | Installed separately; their licenses remain applicable. |

`patches/fast_lio_imu_queue.patch` is a small modification of GPL-2.0 FAST-LIO
code, distributed under that license rather than the application's MIT license.
The GPL v2 text is provided in `LICENSES/GPL-2.0.txt`. Apply the patch to the exact
FAST-LIO revision in `dependencies.lock.json`. If distributing the patched
algorithm or a binary, comply with its upstream license and source obligations;
the MIT application license does not replace them.

The optional `navigation/p2_nav_controller` package contains this project's own
Nav2 Humble extensions. It links to separately installed RPP and Smac packages
from Nav2 1.1.20; their Apache-2.0 notices and upstream licenses remain applicable.
No robot vendor source, model mesh or policy weight is included.

可选通信实现使用 ROS Humble 的 [rmw_cyclonedds](https://github.com/ros2/rmw_cyclonedds) 与 [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) 系统包；不随本源码包分发，使用时保留各上游许可。
