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
| Nav2 | https://github.com/ros-navigation/navigation2 | Separately installed; Apache-2.0, retain upstream notices. |
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox | Separately installed; LGPL-2.1-only and upstream component notices apply. |
| ROS 2 / PCL / Eigen | Their upstream distributions | Installed separately; their licenses remain applicable. |

`patches/fast_lio_imu_queue.patch` is a small modification of GPL-2.0 FAST-LIO
code, distributed under that license rather than the application's MIT license.
The GPL v2 text is provided in `LICENSES/GPL-2.0.txt`. Apply the patch to the exact
FAST-LIO revision in `dependencies.lock.json`. If distributing the patched
algorithm or a binary, comply with its upstream license and source obligations;
the MIT application license does not replace them.
