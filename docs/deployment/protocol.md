# 当前 V2 设备协议入口

当前固件由 [CLD1994/device-nerve-esp32](https://github.com/CLD1994/device-nerve-esp32) 维护。完整命令、参数范围与固件状态转换以该仓库的 [线协议](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/protocol.md)、[标定保存](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/calibration.md) 和 [STEP_LOSS 诊断](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/step-loss-diagnostic.md) 为准。本页只说明 cartpole 当前消费者使用的接口，不维护另一份固件参数注册表。

## 部署路径

| 路径 | 适用对象 |
|---|---|
| [`edge/`](../../deploy/v2/edge/README.md) | 正式 CV610 端侧单动作 OM 推理与控制；以 `run --package <edge-package.txt>` 加载从正式模型清单派生的模型包。目标板验证范围见该目录说明。 |
| [`device/run_policy.py`](../../deploy/v2/device/README.md) | Windows Python 单动作 ONNX 运行器及调试参照；频率与速度尺度读取 `model.json`，支持 400/500 Hz、0.9/1.25 m/s。 |
| [`device/edge/`](../../deploy/v2/device/edge/README.md) | C 会话及接收适配接口；尚未集成目标板 NPU 控制循环。 |
| `pc_runner.py` | 旧 V2 参考入口，不用于正式模型部署。 |
| [`../protocol.md`](../../deploy/protocol.md) | V1 旧装置的 `S`/`P` ASCII 协议，与 V2 不同。 |

V2 使用 **460800 8N1、ASCII 行文本**，行首、命令名与枚举值区分大小写。完整行以换行结束；接收器允许可选回车。不是二进制帧协议。

## 状态与有效性

```text
STATE <flags> <x_um> <theta_urad> <vx_mm_s> <vth_mrad_s>
MODE <mode> <reason>
```

`STATE` 后必须恰有五个十进制整数。`flags` 范围为 0～3：位 0 表示小车位置及速度有效，位 1 表示摆杆角度及角速度有效。其余四个数值依次为小车位置（µm）、摆角（µrad）、小车速度（mm/s）和角速度（mrad/s）。标定坐标中点为 x=0；摆角按 ±π 环绕。两路传感器均为 MT6835，但固件小车编码器故障的线协议名称仍为 `AS5600`，不能由名称推断当前器件。

固件状态报告目标为 500 Hz，与上位机 400/500 Hz 决策周期分开。运行器按绝对栅格消费新观测，不是收到每行就推理。状态行不带设备采样时间戳或序号，主机接收年龄不能证明端到端新鲜度；两路传感器也不保证同步。

固件持续发送 STATE，有效性与数值在同一行关联，不使用独立的 `VEL_VALID` 行。无效通道的位置和速度字段必须均为 0；这些 0 是占位符，不能当作真实测量送入推理。

| flags | 小车通道 | 摆杆通道 |
|---|---|---|
| 0 | 无效 | 无效 |
| 1 | 有效 | 无效 |
| 2 | 无效 | 有效 |
| 3 | 有效 | 有效 |

四个测量值必须在有符号 32 位整数范围内；有效摆角范围为 −3141592～3141592 µrad。Python 与正式 edge 消费者均拒绝格式异常、越界整数和非零无效占位。主机会话依据有效位和接收超时判定是否可使用数据，不能仅依据 MODE 或一个看似正常的位置值判定有效。暂时无效时的暂停与恢复行为见对应运行器文档；固件进入 FAULT 后，测量恢复不自动恢复控制会话。

## 标定、配置与启动

- `QUERY` 查询模式；`CALIB?` 查询行程、保存状态和本次参考状态。
- 完整标定使用 `MARK LEFT`、`MARK RIGHT`。有可用保存行程时，重启后的左参考确认使用 `CONFIRM LEFT`，不会自动使能。`CAL_STATE=3` 不等于本次定位完成；当前会话预检要求 `CAL_STATE=2`、`CAL_SAVED=1`、`CAL_STORAGE=0`。
- `CONFIG?` 查询全表；`CONFIG? <NAME>` 只读单键，返回 `OK <NAME> <value>`。持续控制运行器轮转单键查询，避免全表回复突发。
- `CONFIG <NAME> <value>` 修改配置。持久化与允许模式见固件文档；不要假设全部运行参数都保存在 NVS。
- `ENABLE` 仍要求有效标定及供电验证；成功后为 `MODE READY NONE`。运行器不替用户调用 ENABLE，不修改固件参数。

频率、动作倍率和训练加速度来自正式模型 `model.json`，端侧准备工具由它生成模型包；不要根据目录名猜测参数。运行器核对当前固件配置，不自动覆盖台位。运行参数是配置值，不是实测机械能力。现场行程、方向与摆角零点必须读回核对。

## 动作与停止

`VEL <v_mm_s>` 是目标速度，不是力或 PWM。固件按自身加速度限幅跟踪目标；主机每次动作前检查模式、有效性、配置与时序。当前运行器只接受单动作模型，不使用双动作 `VELA` 实验协议。

正常停止先发 `VEL 0`，继续接收新状态，再请求 `DISABLE`。READY 下引擎仍非零速时，固件返回 `ERR BUSY`；只有确认 `MODE DISABLED NONE` 才记录禁用成功。FAULT 中的 DISABLE 不清除原故障；RESET 是独立的故障清除操作，不是自动恢复运动。

固件看门狗、位置保护与运行器时序检查是不同机制。Windows ONNX 运行器将位置保护交给固件；正式 edge 根据回读的轨道长度和软件余量抑制越界方向动作。两者的默认摆角控制区域为 ±20°，暂停后回到 ±18° 内才恢复发送速度。具体暂停、失效与退出条件以各自 README 为准，各机制不保证硬实时或物理停车距离。

## 可选 STEP_LOSS 事件

固件主线合入 PR #17 后支持可选接口：

```text
RUN?
OK RUN <run_id>
EVENT STEP_LOSS <event_id> <run_id> <device_us>
```

`CONFIG_CARTFSM_STEP_LOSS_EVENTS` 提供运行身份和首次事件通知；诊断轨迹由独立的 `CONFIG_CARTFSM_STEP_LOSS_DIAGNOSTICS` 开关启用。`CONFIG_CARTFSM_STEP_LOSS_OBSERVE_ONLY` 才决定是否仅记录 STEP_LOSS、保持 READY 并交给上位机处置。三项默认关闭，事件通知或诊断开关本身不等于关闭丢步保护。

主机 `--step-loss-monitor` 校验同一运行身份并保留事件窗口，不切换低速、不改模型动作。正常固件保护进入 FAULT 时仍停止；observe-only 保持 READY 且其他条件满足才继续。`--timing-profile debug` 是独立显式选项，事件记录不会自动放宽时序。事件无 ACK 或重放保证，设备时间与主机接收时间不能直接相减。

控制结束并停稳后，用固件仓库的配套工具读取 `STEPLOSS?` / `STEPTRACE?`。当前轨迹为 SLT4 软件 STEP 位置与缓存编码器的非同步证据，不是旧实验 SLT3 的 PCNT/EN/SPI 记录，也不证明物理脉冲或实际失步。

## 验证边界

协议与单元测试通过不代表 Windows 串口、实际 ONNX 模型、ESP32 目标构建或真机闭环验收。目标版本与构建开关必须一并记录；旧实验分支现场结果不能替代主线版本验收。
