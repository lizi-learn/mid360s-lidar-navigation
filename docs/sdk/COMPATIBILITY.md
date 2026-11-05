# 云迹兼容矩阵与证据

资料核对日期：2026-11-05。目标是让业务层逐步共用导航接口；不能仅凭产品网页宣称全系列无缝替换。

## 三种接入方式

| 方式 | 用途 | 当前结果 |
| --- | --- | --- |
| 统一 HTTP SDK | 新程序使用相同 Python / TypeScript 方法接 P2 或 WATER provider | 已实现并通过本地合同测试；各 provider 能力可查询 |
| WATER → P2 兼容服务 | 老程序已有 `/api/move` 等底盘调用 | 已实现下面子集；请求 LF/CRLF profile 与原 SDK 的分帧仍需逐字节核对 |
| 云迹云端 Open API | 同事的酒店/配送云平台、订单、回调、门店业务 | 仅完成官方历史签名向量工具与合同研究；没有冒充云迹云服务 |

## WATER → P2 命令表

| 命令/参数 | 实现 | 说明 |
| --- | --- | --- |
| `/api/move?marker=名称` | 已实现 | 只导航，不自动播放 P2 产品介绍 |
| `/api/move?location=x,y,theta` | 已实现 | 米/弧度；location 与 marker 同时出现时 location 优先 |
| `/api/move/cancel` | 已实现 | 取消当前导航，不是双足电机失能 |
| `/api/robot_status` | 已实现子集 | 五种 move_status（结果未知时为null）；缺失电量、楼层、充电、急停保持 null |
| `/api/request_data?topic=robot_status&frequency=2` | 已实现 | 当前 profile 0–2 Hz，0 关闭；callback 与 response 分流 |
| `/api/markers/query_list` | 已实现 | results 按名称索引；marker_name/floor/key/pose；空列表为 null |
| `/api/markers/insert?name=名称` | 已实现 | 实际当前位姿同名 upsert；不伪造当前位姿 |
| `/api/markers/insert_by_pose` | 已实现 | name,x,y,theta 必填；同名更新，不清除介绍和路线 |
| `/api/markers/delete?name=名称` | 已实现 | P2 被路线/边引用时明确报 MARKER_IN_USE，避免悄悄破坏路线 |
| `/api/markers/count` | 已实现 | 返回 results.count |
| `/api/software/get_version`、`/api/robot_info` | 已实现自有身份 | 返回 navbridge 版本/产品号，不冒充云迹固件 |
| notification 01001–01004 | 已实现子集 | 订阅连接上按本桥任务状态变化发送，含扩展 task_id；不是完整厂商事件集 |
| `/api/markers/query_brief`、floor/type/num 配置 | 未实现 | P2 尚无 WATER 楼层/特殊点位映射；不能猜楼层 |
| `markers/count` 巡游、容差、重试、angle_offset 等覆盖 | 未实现 | 不降级成单点，不忽略参数；统一 SDK 可使用 P2 已保存路线 |
| `/api/joy_control`、`/api/estop` | 不等价，未实现 | 轮式 SI 速度/可推动自由停止不能直接映射双足 D 输入与电机状态 |
| 回充、乘梯、闸机、厂商升级/关机 | 未集成 | 没有相应硬件和业务接口时不返回成功 |

返回码采用 WATER 已公开的 `OK / INVALID_REQUEST / REQUEST_DENIED / UNKNOWN_ERROR` 合同子集；具体原因另在 `extensions.error_code`。未知参数报错，不把未生效设置回执成成功。

## 线包与任务的关键差别

厂商 WATER v1.8.7 说明 TCP 31001、类 URL 请求、JSON 响应及 uuid，但没有定义请求的分隔符/长度前缀。社区实现默认发送裸请求，同时允许配置终止符。**本桥使用确定的 LF/CRLF 分帧，是本项目 profile，不是厂商已确认的原生线包格式。** TCP 半包、粘包及 UTF-8 多字节切分均有自动测试；没有完整结束符的请求不会执行。

`uuid` 是请求关联标识；`task_id` 是导航任务标识。受理回执 `OK` 不等于到达，`running_status=idle` 或速度为零也不等于到达。P2 完成采用原动作/任务真实终态；介绍任务完成还要经过介绍、停留或访客继续。

厂商公开的 robot_status 示例不保证带 task_id。统一 WATER provider 只有在状态确实携带相同 task_id 时才关联为已完成，否则任务状态明确 unknown，并保留原厂 move_status。原业务程序可继续用低层 WaterClient 原始状态，按它已有的唯一控制者约定处理；本 SDK 不把未证明的关联当事实。

P2 本来没有电量/充电/软硬急停来源，兼容状态保留 null；老程序若要求这些字段必须是数字/布尔，需要由其业务层支持未知值，或另接真实数据。楼层号不是地图 ID，也不是 Z 高度。P2 点位未设朝向时，WATER 四元数采用 0 朝向占位，并明确 `extensions.heading_unspecified=true`，不宣称已测朝向。

## 云端签名与业务边界

`navigation_sdk.cloud_signing` 按官方历史示例实现排序原始标量参数、HMAC-SHA1、密钥后附 `&`、Base64。官方 `testid` 向量输出 `VQAdXELyv7rl/6E4bj1VdV4X6vI=`，已离线核对。请求方法/路径没有擅自加入签名；GET URL 编码与签名前原值区分。复杂对象/布尔序列化缺少官方向量，工具要求调用方按其既有 SDK 明确预序列化。

云端 orderSN/orderTaskId、配送/取货/排队状态与本地 move 状态不是一回事。没有将 `/v1/order/cancel` 偷换成 `/api/move/cancel`，也没有请求真实云迹服务或复制它的帐号体系。后续接同事云平台，沿用其鉴权、设备身份与业务回调，在设备驱动层选择 P2/WATER provider。

## 如何完成同事程序互通

需要的具体材料是：同事实际 SDK 语言/版本、用到的方法清单、请求终止字节、去敏请求/回执/订阅样例，以及其任务完成判断方式。把这些加入 `tests_sdk` 作为固定合同夹具，再补缺失映射。点位名称、map/floor 映射、单位和未知字段策略都应写成显式 profile；不改 V2 标签，不引入第二次开始确认。

目前没有同事实际程序或真实云迹底盘的互通记录，故本版只声明“已实现并离线验证的兼容子集”，不能声明“所有云迹产品完整兼容”。

## 一手资料与固定版本

- [云迹产品目录](https://www.yunjichina.com.cn/product)：产品定位参考，不是 SDK 合同。
- [厂商署名 WATER v1.8.7 手册](https://bbs.realman-robotics.cn/uploads/20240510/bb301c61dc206155dd8e0ae6fa79e2bb.pdf)：2021-11-02；TCP/uuid 第9页，移动10–14页，状态14–19页，点位20–28页，订阅32–36页，通知36–48页。链接为第三方公开托管；本站不再分发原 PDF。
- [官方 yunji-ai/open-api 文档](https://github.com/yunji-ai/open-api/blob/d357b1a941bd6db73a547cb68b4e14a8929620ad/docs/README.md)：历史 commit d357b1a（2020-04-02），不代表已验证现行云服务。
- [官方 JavaScript 签名示例](https://github.com/yunji-ai/open-api/blob/d357b1a941bd6db73a547cb68b4e14a8929620ad/examples/JavaScriptSign.js)：大小写使用 testid；README 中 testId 不一致已保留说明。
- [社区 WATER2 客户端源码](https://github.com/ros-claw/water2-chassis-mcp/blob/52ca9525e63ae5b5ed5fa556afc38e26e47b3f1d/src/water2_chassis_mcp/client.py)：作为该实现自身行为的证据，非厂商认证；其声称的 v1.8.8 原文未公开，未据此扩大全系列兼容范围。

协议适配由本项目独立编写，没有复制无明确许可的社区代码。公开包仅含自有代码、合成样例与资料链接，不含厂商手册原件、私有机器人资产、办公室地图/录包或凭据。
