# 本地接管协议合成检查

不启动 ROS、步态 SDK、仿真动力学或真实执行器。测试中的健康、策略和地面输入是明确标记的合成夹具，不代表实机验收。

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Inavigation experiments/gait_velocity_input/check_loop.cpp -o /tmp/gait-loop
/tmp/gait-loop
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Inavigation experiments/gait_velocity_input/contract_server.cpp -o /tmp/gait-contract
PYTHONPATH=navigation python3 experiments/gait_velocity_input/check_transport.py /tmp/gait-contract
```

跨 UID 检查需要本地 root 权限，只在临时目录创建合成 socket，分别验证 UID 1000 可访问、其他 UID 被文件权限及 `SO_PEERCRED` 拒绝，结束后删除 socket：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Inavigation experiments/gait_velocity_input/check_peer_acl.cpp -o /tmp/check_peer_acl
sudo env PYTHONPATH="$PWD/navigation" python3 experiments/gait_velocity_input/check_peer_acl.py /tmp/check_peer_acl
```

协议、接入要求与未通过项见 [步态接口说明](../../docs/GAIT_HANDOFF.md)。
