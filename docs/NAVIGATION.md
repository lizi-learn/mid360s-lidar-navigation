# 前向导航工作流（ROS 2 Humble）

本目录把传感器预览扩展到可运行的 SLAM / Nav2 工作流。默认控制器为 RPP，采用前向移动和原地转向，侧向与倒退指令会被最终速度门拒绝。公共版本可在平面模拟器中完整执行导航；实机模式接收真实雷达，提供建图、定位与路径预演，输出仅到 `/p2_nav/cmd_preview`。

## 算法选择

```mermaid
flowchart LR
    L[MID-360S + 内置 IMU] --> F[FAST-LIO]
    F --> P[局部地面拟合与高度切片]
    P --> S[SLAM Toolbox 建图与回环]
    S --> M[地图与位姿图保存]
    M --> A[AMCL 地图定位]
    A --> G[Smac State Lattice 前向路径]
    G --> R[RPP 跟随与朝向调整]
    R --> V[平滑与碰撞监视]
    V --> Q[速度与数据健康门]
    Q --> X[仿真执行 / 实机预演]
```

- **FAST-LIO**：保留雷达内置 IMU 的紧耦合里程计。机身 IMU 暂作独立诊断；没有时钟和机身外参标定时不进行重复融合。
- **SLAM Toolbox**：在虚拟二维扫描上建立可保存、可重载的占据地图和位姿图，启用回环。高度切片不能把三维环境中的台阶、悬空物和坡道全部压缩为可靠二维地图。
- **Smac State Lattice**：使用 Nav2 自带的差速运动基元，禁用反向扩展；不采用全向横移基元。规划的可转向性需要与双足实际步态重新验收。
- **RPP**：默认方案，接近障碍和转弯时降低速度，必要时先调整朝向再前进。选择依据是本次实际运行的三目标与故障测试，不是声称算法在所有平台上最优。
- **MPPI DiffDrive**：保留对比配置，明确限制 `vy=0`、`vx>=0`。本轮平面仿真有一个目标超时，尚未通过，不能当作默认可用配置。

参考：[RPP 官方文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)、[RPP 论文](https://arxiv.org/abs/2305.20026)、[Nav2 调参指南](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/)、[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)。项目按实际安装的 Humble API 配置；不要直接用 Rolling/Jazzy 参数替换。

## 安装与启动

Ubuntu 22.04 / ROS 2 Humble，系统 Python 3.10：

```bash
sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox python3-numpy python3-scipy python3-yaml
bash navigation/console.sh --simulation
```

浏览器打开 <http://127.0.0.1:8930/>。服务仅绑定本机；远程使用 SSH 本地转发。ROS 域默认 194，传感器域默认 188，均仅本机通信。公共代码不提供电机总线、机器人遥控或全身控制的接口。

### 仿真完整流程

1. 填一个新的地图名称，点击“开始建图”。
2. 点击“运行仿真建图路线并保存”。模拟机器人沿前向路线运动并转向，结束后保存 `mapped.pgm`、`mapped.yaml`、`mapped.posegraph` 和 `mapped.data`。
3. 选择已保存地图，点击“载入地图，进入导航”。默认仿真起点为已知的 `(2, 2.5, 0)`；实际部署不能假设机器人每次都在这个位置。
4. 点击已观察的空白地面选目标，调整到达朝向，点击“导航到选点”。页面显示路径、位置、速度指令和定位状态。
5. 速度滑块设定的是**上限**，默认 0.20 m/s、通用范围 0.03–0.30 m/s；实际执行速度还受转弯、障碍和模型限制。点击“停止并取消目标”立即撤销授权并取消目标。

地图与日志默认保存在 `runtime/workflow/`。切换模式会停止本页面启动的进程；停止服务也会终止其仿真和建图路线。没有安装导航依赖时，原有的纯 Python 合成演示仍可使用。

### 实机建图与预演

先运行根目录的传感器链路，并等待重力对齐完成：

```bash
bash run-stack.sh
# 第二个终端
export P2_LIDAR_ALIGNMENT="$PWD/runtime/map_alignment.json"
bash navigation/console.sh
```

实机适配器从域 188 读取 `/cloud_registered` 与 `/Odometry`，自行按当前会话的重力对齐矩阵转换点云，在域 194 发布隔离的扫描、里程计和 TF。它采用雷达地面投影作为临时 `base_link`，**不是已经标定好的机器人机身坐标**。地面拟合随局部里程计高度变化；未观测扫描角度为 NaN，不能伪装成无遮挡射线。前方地面支持不足、源数据过期、里程计跳变或重力参考变动会阻止速度预演。

人工推架建图时，点击保存，然后载入地图；在图上设置当前初始位置和朝向，等待定位就绪，再点击目标点生成路径。实机目标按钮调用 `ComputePathToPose`，不调用执行导航动作。重启传感器、改变脖子角度、移动安装支架或更换机器人后，应新建会话并重新标定，不拼接不同参考系的地图。

## 速度接口与机器人接入

链路：`cmd_raw → velocity_smoother → cmd_smooth → collision_monitor → cmd_collision → ForwardGate → cmd_safe/preview`。

- 最大前进 0.30 m/s、角速度 0.35 rad/s；加速度 0.15 m/s²，减速度 0.30 m/s²，角加速度 0.5 rad/s²。
- 命令超时 0.30 s；扫描或里程计超过 0.50 s、定位无效、地面缺失、暂停或取消目标均立即输出零。
- 缩放线速度与角速度时保持曲率；最终门拒绝侧向、倒退、无穷数及无效时间。
- `velocity_contract.hpp` 定义独立 C++ 消费协议：48 字节、单调时间、实例号、序列号和 300 ms 租约。检查健康、标定、步态状态、导航授权以及急停/人工控制优先级。它不打开串口或共享内存，也不等于已接入厂商整机程序。

本次 P2 模型测试发现：原始步态指令 0.10 m/s 几乎没有平移，0.20 m/s 才有明显前进。通用的低速接近参数不能直接复制到该步态。动力学实验使用了明确标注的**仅仿真**响应补偿；这不是实机速度标定，不能据此启用真实行走。

公共版本默认足迹为 0.70 × 0.80 m 的矩形并另加 0.05 m 膨胀边距，属于配置假设。真机应测量支架、机身、手臂与踏步外包络，再确定足迹、最小通道和停止距离。仅用单个静态平面扫描无法完成双足平衡、落脚、楼梯、跌落边缘或人体避让的全部判断。

## 可重复验证

```bash
python3 -m unittest discover -s tests -v
g++ -std=c++17 -O2 tests/velocity_contract_test.cpp -o /tmp/velocity-contract-test
/tmp/velocity-contract-test
# 使用工作台完成建图并保存后，也可在同一 ROS 域运行动作测试：
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="/usr/lib/python3/dist-packages:$PWD/navigation:${PYTHONPATH:-}"
/usr/bin/python3 -S navigation/benchmark.py navigation --directory runtime/benchmark
```

`benchmark.py` 使用实际 Nav2 动作服务器。包含三个目标、速度上限、无地面、扫描/里程计中断、动态障碍与侧向错误注入。场景碰撞计数、模拟真值误差与 ROS 状态全部写入报告。它的默认世界是平面运动学，不能据此宣称双足动力学已通过。可选的 `native_backend.py` 使用独立进程、超时租约和有界时长连接外部动力学后端；厂商模型、权重与专有源码不随本仓库分发。

对真实录包的 `bag_to_scan.py` 和 `replay_scan.py` 只读取已经关闭并确认完整的 SQLite 录包；使用原始点时间和录制的 LIO 位姿作运动补偿，再回放到隔离 ROS 域。它不重新运行 FAST-LIO，也没有独立位置真值，因此回放完成、启用回环或地图看起来闭合都不能代表精确闭合误差已经测定。
