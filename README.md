# MID-360S 雷达可视化导航

倒装 MID-360S 的 ROS 2 雷达惯性建图、浏览器可视化、已知地面路径预览与人工推架录包。适用于没有深度相机、先验证地图与定位的实验平台。

**当前是实验版本。新增 SLAM Toolbox / Nav2 前向导航工作台，支持仿真中的完整建图、地图保存载入、定位、目标导航和速度调节。** 实机提供建图与路径预演；真实行走仍需机身外参、整机零位及步态接入验收。公开程序不发布电机、颈部或全身控制命令。

- [交互演示与说明](https://lizi-learn.github.io/mid360s-lidar-navigation/)：全部使用合成几何，不连接实机。
- [前向导航工作台与完整流程](docs/NAVIGATION.md)
- [换机与实机操作](docs/MIGRATION.md)
- [验证结果与已知问题](docs/VALIDATION.md)
- [第三方依赖与许可](THIRD_PARTY_NOTICES.md)

## 能做什么

| 功能 | 范围 |
| --- | --- |
| 实时点云 | SDK2 → Livox ROS Driver2 → FAST-LIO；倒装与初始重力对齐 |
| 局部地图 | 跟随雷达移动的近期地面/障碍栅格；未知区域保持阻塞 |
| 路径预览 | A*、障碍膨胀、禁止斜穿障碍角；起点可能吸附到附近已观察地面，不代表盲区可通行 |
| 人工推架记录 | 原始逐点雷达时间、雷达 IMU、里程计、可选机身 IMU、轨迹与累计点云 |
| 数据健康 | 按消息源时间检查过期；明显跳变锁定路径预览并保留诊断数据 |
| 双 IMU | 从 `/p2_nav/body_imu` 接收独立对照数据；不直接融合 |

原传感器页面的累计图用于检查建图质量，切回“局部”后可预览路径；它仍使用固定地图高度门限。新增 `navigation/` 工作流采用局部地面拟合、SLAM Toolbox 地图与 Nav2 规划，见上面的完整流程。

## 不接硬件，先试合成演示

Python 3.10+：

```bash
git clone https://github.com/lizi-learn/mid360s-lidar-navigation.git
cd mid360s-lidar-navigation
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/python demo_server.py --port 8921
```

打开 <http://127.0.0.1:8921/>，点击地面预览路径。合成演示不启动 ROS、驱动、电机或真实录包。GitHub Pages 版本提供相同合成网格的浏览器交互；本地 Python 版本使用实际 `FloorMap.plan`。

```bash
.venv/bin/python -m unittest discover -s tests -v
```

## 实机：Ubuntu 22.04 / ROS 2 Humble

代码来自 RK3588 / Ubuntu 22.04 实验部署。其他平台的构建、传感器型号与安装方式需要实测；公开可移植版本不等于新机器已验证。

1. 安装 ROS 2 Humble，并准备构建与系统 Python 依赖：

   ```bash
   sudo apt-get install git cmake build-essential libeigen3-dev libpcl-dev \
     libjsoncpp-dev python3-colcon-common-extensions python3-numpy python3-scipy \
     ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rviz2 ros-humble-rosbag2
   ```

2. 按传感器实际配置好专用网卡。本项目不自动改网卡或访问电机总线。示例地址仅作示意：

   ```bash
   python3 scripts/configure.py --host-ip 192.168.1.100 --lidar-ip 192.168.1.196
   ```

3. 构建锁定版本的上游依赖：

   ```bash
   bash scripts/build-deps.sh
   ```

   SDK 安装在本项目 `sdk/`，ROS 工作空间在 `ros_ws/`。不覆盖系统 SDK，也不调用上游会删除工作空间构建目录的 `build.sh`。构建脚本已做语法及补丁适用性检查，通用脚本尚未在另一台空白机器完成端到端构建验收。

4. 保持安装牢固、雷达与机身静止；查看并核对 `config/mid360s_lio.yaml` 的雷达内部 IMU 外参和数据类型，然后运行：

   ```bash
   bash run-stack.sh
   ```

   页面默认 <http://127.0.0.1:8920/>；ROS 域默认 188、仅本机通信。可复制 `.env.example` 为 `.env` 调整域和端口。远程查看建议使用自己配置的 SSH 本地转发；本页面没有认证，不应直接发布到公网。

5. 在地面标记起点和朝向。静止 5 秒后点击“开始推架记录”，等待“原始录包就绪”；先慢推小圈，回到**同一物理标记、同一朝向**后静止 2 秒，再确认返回并保存。

   当前姿态补偿要求倒装。仅知道“线束朝后”不能替代外参验证；不要复制另一台机器人脖子的编码器零位或绝对位置。

可选 RViz：在同一 ROS 域和已 source 的工作空间下执行 `rviz2 -d config/navigation.rviz`。

## 记录与数据完整性

每段记录位于 `runtime/surveys/<session>/`：

- `raw_bag/`：不压缩 SQLite，约 256 MiB 分片；保留 ROS 消息原值与逐点时间。
- `trajectory.csv`、`lidar.csv`、`body.csv`：派生对照数据；CSV 中 Livox 加速度从 g 转为 m/s²，原始录包不改写。
- `map.pcd`、`map.pgm`、`map.yaml`：累计观测；未知地面不自动补成可通行。
- `session.json`、配置快照、可选 `returns.json`：本段状态与操作者确认。

单段 20 分钟自动保存；磁盘余量低于 3 GiB 自动停止，开始时要求至少 5 GiB。曾遇到“实时 zstd 压缩正常退出、分片却不可读”，因此采集期间取消压缩。**不能直接读取正在写入的 SQLite 文件**；先结束并保存，再检查分片完整性与回放。重启传感器服务会改变地图参考，不能把重启前后轨迹直接拼成可信地图。

## 可选机身 IMU

本项目只要求 `/p2_nav/body_imu` 为 `sensor_msgs/msg/Imu`，加速度 m/s²、角速度 rad/s，时间戳真实且递增。可选 `/p2_nav/body_imu_meta` 可用于保存设备时序元数据；没有该话题不影响雷达主链路。

机身 IMU 驱动由使用者提供。未包含原机器人私有驱动、共享内存 ABI、串口程序或电机控制器。原实验的机身设备时间无效、采用主机接收时间，且存在采样间断和停车后航向差；没有把它当作位置真值或进行双 IMU 融合。

## 许可

独立 Python/UI/文档采用 [MIT](LICENSE)。FAST-LIO 队列补丁沿用上游 GPL-2.0，见 [第三方说明](THIRD_PARTY_NOTICES.md) 与 `LICENSES/GPL-2.0.txt`。上游 SDK、驱动和算法按各自许可使用；本仓库不附带其二进制或完整源码。
