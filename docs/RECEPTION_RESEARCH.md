# 导购接待导航参考调研 · 2026-11-02

以下是公开项目的功能与设计参考，自有实现不复制对方私有代码、模型或素材。视频只用于了解交互/演示，不作为本机器人已达到同等性能的证据。

| 来源 | 可借鉴内容 | 本项目取舍 |
| --- | --- | --- |
| [Nav2 Route Server 官方文档](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/route_server/configuring_route_server/) | 预定义路线图、动态边代价、通道关闭、经过节点时的动作 | 增加语义路线层，继续使用现有实际规划/跟踪控制，不整套替换已经实测的控制器 |
| [Nav2 Waypoint Follower 官方源码说明](https://github.com/ros-navigation/navigation2/blob/main/nav2_waypoint_follower/README.md) | 多点顺序执行、到点任务插件、失败处理 | 独立的接待任务引擎，加入讲解、访客继续、取消和当前E档位所有权语义 |
| [Open-RMF 接口文档](https://github.com/open-rmf/rmf_docs/blob/main/docs/source/interfacing/index.rst) | 门控制器状态/请求、多机器人交通接口 | 借鉴门的明确状态来源；目前单机不引入全量多机调度依赖，也不假装已连接自动门硬件 |
| [RTAB-Map ROS 官方SLAM说明](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/doc/rtabmap.md) | 图SLAM、回环、数据库与三维地图；无相机时依靠接近性激光匹配 | 作为续扫、多区域和回环后端的后续对比候选；本轮不替换FAST-LIO运行链 |
| [NVIDIA Nvblox 官方文档](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) | GPU三维重建并产生Nav2二维代价图，支持深度相机及三维雷达输入 | 先实现轻量XYZ场景预览；是否引入GPU重建须结合ARM设备和实测负载，不能仅为漂亮画面挤占定位/控制 |
| [Steve Macenski 的 Go2 路线导航演示 / YouTube](https://www.youtube.com/watch?v=KDMI70BuDs0) | 原项目作者展示足式平台的路线导航组合 | 参考路线/监控组合，四足运动能力不等于P2双足已验证能力 |
| [爱折腾智能机器人：激光3D脚本多点导航 / Bilibili](https://www.bilibili.com/video/BV1WW2hYDEqN/) | 作者自己的多点流程演示 | 参考操作组织；页面介绍与演示标题不能证明其算法细节或本项目精度 |

源码检索快照：Nav2 main `c633de75b9663e54dbe48d8177b1a2cd11cfd986`；Open-RMF rmf_ros2 main `b892a9a4b80e3abf7d4616b21dd57cfe98a31749`；RTAB-Map ROS ros2 `f04a52d255ded6396dc11a54bfe01079ec0b0023`。实际部署仍锁定既有ROS Humble/厂商依赖，并未升级成这些main快照。

许可证核查：Nav2 route与waypoint两个package.xml声明Apache-2.0（仓库整体API返回NOASSERTION，不能用该值概括所有包）；rmf_ros2 Apache-2.0；rtabmap_ros BSD-3-Clause；[NVIDIA-ISAAC-ROS/isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) Apache-2.0。没有在本轮复制其源码。首次查询错误组织名 `nvidia-isaac` 返回404，随后从NVIDIA官方文档链接定位了正确仓库。

产品能力的后续优先级：多条接待路线、产品内容管理、精细点位与方向、动态通道、简单人工介入、可解释失败与恢复、场地版本与导入导出；多楼层、电梯、自动充电和多机调度属于后续现场集成，不因用户只提出导购目标就默认增加为导航启动条件。
