# 换机器检查

迁移的是传感器可视化软件，不是上一台机器的电机配置或地图定位状态。

1. **固定安装。** 核对倒装姿态、前向、线束余量和离地高度。第一次测试固定安装角度。软件不会自动调脖子；上一台约 5°低头仅是那个安装的实验起点，不是通用最优角度。
2. **网络。** 确认 LiDAR IP、主机专用网卡地址、实际 UDP 端口和 SDK 型号配置。`scripts/configure.py` 只写本地配置，不会配置网卡。不要向电机专用网卡附加雷达地址。
3. **内部与外部坐标。** YAML 的 `extrinsic_T/R` 是雷达到其内部 IMU 的外参起点，需核对硬件版本。雷达到机身 `base_link`、脖子运动外参、真实足迹和碰撞模型未提供。页面箭头参考雷达光心投影。
4. **构建。** 使用 `dependencies.lock.json` 中的上游版本；检查补丁成功、没有同时运行第二个雷达驱动、没有混入另一个 ROS 域的数据。
5. **静止初始化。** 启动时保持静止，让 IMU 初始化和初始地面高度估计完成。确认源时间年龄合理、没有时间回退、位置跳变或反复重启。
6. **小圈记录。** 在地面做固定标记，先推 5–10 米的小圈。返回同一点、同一朝向后静止，再记录闭合检查。没有准确返回时，起终点差不等于漂移。
7. **保存后回读。** 先停止录包并确认退出正常、metadata 存在，再逐个检查 SQLite 和顺序读取/回放。不要在采集中查询数据库。回放应放在隔离 ROS 域，例如 189，不能与实机域混用。
8. **再扩大测试。** 对照墙面、走廊、实际测量距离、重复路线和静止段；验证局部地面、回环/重定位与地图重载。完成这些检查后，才另行接入机器人执行层。

## 已有 ROS 环境

可以仅运行 `nav_server.py` 对接现有 `/livox/imu`、`/Odometry` 和 `/cloud_registered`；接口与坐标必须符合当前 FAST-LIO 配置。它会发布 `/map`、`/p2_nav/planned_path` 及预览参考 TF，不发布行走命令。

原始录包需要 `/livox/lidar` 的 Livox CustomMsg。记录器列出的可选机身话题未出现时，仍会保存已出现的主链路数据。若系统已有 `map` 坐标或同名 TF 发布者，需要隔离本项目的 ROS 域，不能直接产生冲突的 TF。

## 可选用户服务

默认模板假设克隆到 `~/mid360s-lidar-navigation`；其他路径先修改模板中的工作目录与启动路径。构建和手动检查通过后：

```bash
mkdir -p ~/.config/systemd/user
cp deploy/mid360s-navigation.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user start mid360s-navigation.service
```

模板没有自动安装、启用开机启动或启动其他机器人服务。停止：`systemctl --user stop mid360s-navigation.service`。传感器数据持续过期时，watchdog 会退出当前栈；可选服务会以新的地图参考重启，并保存退出时的录包。启动次数有限制，硬件断开时应先停止服务。

## P2-1003 / P2-1005 的独立安装配置

换机时取新的源码和依赖锁定文件，为新机器重新生成网络与身份配置。不要复制旧机器的 `runtime/`、`config/robot.local.json`、脖子零位或已经标记为验收通过的外参。下面的 IP 仅为格式示例，填写新机器雷达专用网卡的实际地址；命令不会改网卡：

```bash
python3 scripts/configure.py \
  --host-ip 192.168.1.100 --lidar-ip 192.168.1.196 \
  --robot-name P2-1003 \
  --robot-serial "$(tr -d '\000\n' </proc/device-tree/serial-number)"
```

1005 使用自己的名字和板卡序列号。缺少板卡序列号时先查明身份，不用另一台的值代填。已有配置需人工核对后使用 `--force`；每次重新生成都会得到新的安装 ID，旧数据继续保留。

启动传感器时会核对板卡身份、雷达网络配置，然后把新数据放到 `runtime/robots/<机器名>/<安装ID>/`。不同安装不会共享初始重力对齐、录包或定位状态；手动指定的运行目录若已绑定其他机器，或存在没有身份标记的旧数据，会拒绝使用。默认不删除或接管旧目录。

```bash
bash run-stack.sh
# 另一个终端，传感器初始化完成后：
bash scripts/build-nav-controller.sh
bash navigation/console.sh --gait-profile p2-flat-model
```

工作台自动识别本地安装配置，使用该安装的对齐文件和独立地图目录。自定义配置可设置 `P2_NAV_ROBOT_PROFILE`；自定义数据目录可设置 `P2_NAV_RUNTIME`。两个 ROS 域仍分别用于传感器与导航，改域时要让适配器源域一起匹配。

如果指定 `P2_NAV_BODY_CALIBRATION`，除 URDF 哈希、关节链和测量安装矩阵外，外参 JSON 还必须包含与安装配置一致的 `robot_serial`、`installation_id`。适配器也会核对传感器对齐文件的安装身份。复制旧外参再改 `hardware_verified` 不能完成新机器标定。

仿真不接受实机身份配置；默认运行目录为 `runtime/simulation/workflow/`，也拒绝写入绑定了实机身份的目录。源代码可跨机器，已编译的 SDK、ROS 包和模型后端需按目标架构重新构建。x86 仿真后端不能直接放到 RK3588 上运行。

这些检查已经覆盖错误机器拒绝启动、旧目录不被接管、安装目录分离与外参身份不匹配等回归；没有连接 P2-1003 / 1005 做本轮实物验证。第一次换机仍按上面的小圈建图、保存回读、定位与受控行走顺序验收。公开程序保持预演接口，不自动启用整机驱动。
