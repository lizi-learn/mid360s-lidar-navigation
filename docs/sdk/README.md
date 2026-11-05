# P2 Navigation SDK · NavBridge 0.1.0

把业务程序与底盘分开：同一 Python / TypeScript 客户端接 P2 导航，或接 WATER 底盘适配器；已有 WATER 业务程序可接明确声明的本地兼容子集。导航功能基于已冻结的 V2，SDK 是 V2 之后的独立扩展。

**当前是 SDK 首版，不是云迹全部产品/固件认证。** 已完成本地合同测试；同事现用 SDK 的版本、请求分帧与业务调用尚未拿到。云迹云端订单 API 与底盘 WATER API 不等价。

- [完整项目手册](PROJECT_GUIDE.md)：架构、部署、建图、导览、地图管理、排障和换机。
- [兼容矩阵与来源](COMPATIBILITY.md)：确切命令、传输合同、未实现功能和真实互通边界。
- [OpenAPI 3.1](../../api/navigation.openapi.json)：机器可读 HTTP 合同。
- Python 包：`p2-navigation-sdk`；TypeScript 包：`@p2-navigation/sdk`。
- 代码入口：`navigation_sdk/`；TypeScript：`clients/typescript/`；示例：`examples/sdk/`。

## 五分钟本地运行

Python 3.10+，SDK 没有第三方运行依赖。先在合成后端验证业务调用：

```bash
python3 -m venv .venv-sdk
. .venv-sdk/bin/activate
python -m pip install .
navbridge --backend mock --water-bridge-port 31001
```

```python
from navigation_sdk import NavigationClient
nav = NavigationClient("http://127.0.0.1:18940")
print(nav.capabilities())
task = nav.visit("产品展示", idempotency_key="demo-visit-001")
print(nav.task(task["id"]))
nav.cancel(task["id"])
```

mock 不接 ROS/机器人，也不会按定时器伪造到达。合同测试可显式 `POST /mock/advance {"state":"succeeded"}` 推进合成状态；该端点在 P2/WATER 后端不存在。

TypeScript 安装 Release 的 TGZ，Node 18+ 可直接运行：

```typescript
import {NavigationClient} from '@p2-navigation/sdk';
const nav = new NavigationClient('http://127.0.0.1:18940');
const task = await nav.move({marker:'接待起点'}, {idempotencyKey:'order-001'});
console.log(await nav.task(task.id));
```

## 连接 P2

在装有本次 **Console SDK 扩展**的导航源码旁单独运行：

```bash
navbridge --backend p2 --p2-url http://127.0.0.1:8930 \
  --state-dir ./private-sdk-state --water-bridge-port 31001
```

实际端口以部署为准：8930 是导航 Console 默认端口；18930 是本项目开发电脑曾使用的转发端口。SDK 不启动、重启 D/驱动/雷达，不直接发布速度。旧 V2 Console 可读取状态；其缺少任务引用比较接口时，能力表明确禁用 SDK 派发，安装本次附加接口后接通。它是软件接口升级，不增加人工确认或验收文件步骤。

`move` 只导航；`visit` 执行“导航→朝向→介绍→停留/等待”；`route` 执行已保存的路线。避免业务程序自己讲解、SDK 又重复讲解。

## WATER 兼容入口

本项目兼容服务的请求必须以 LF 或 CRLF 结束，命名合同 `navbridge-water-lf/1`。原生 WATER 手册未定义请求结束符，本实现不靠 TCP 分包/静默时间猜测运动请求是否完整。

```python
from navigation_sdk.water import WaterClient
c = WaterClient("127.0.0.1", 31001, terminator="\n")
print(c.markers())
print(c.move(marker="接待起点"))  # OK 只表示受理
print(c.status())
c.cancel()
c.close()
```

连接真实 WATER 底盘时客户端默认发送裸 URI，结束符可显式选择；先按同事的 SDK/固件核对。多点 `markers/count`、容差覆盖、轮速控制、自由急停、回充、乘梯等未在兼容服务实现，明确返回不支持，不默默忽略参数。

## 测试

```bash
python3 -m unittest discover -s tests_sdk -v
node --test clients/typescript/test.mjs
python3 -m unittest discover -s tests -v
```

这些测试使用本地 HTTP/TCP 合成夹具和项目实际会话代码，不连接真实机器人或云迹云端。V2 的实机结果、原始失败和未完成项保留在 `docs/BASELINE_V2.md`。
