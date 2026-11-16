# 当前 V2 设备协议入口

当前固件由 [CLD1994/device-nerve-esp32](https://github.com/CLD1994/device-nerve-esp32) 维护。完整命令、参数范围与固件状态转换以该仓库的 [线协议](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/protocol.md)、[标定保存](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/calibration.md) 和 [STEP_LOSS 诊断](https://github.com/CLD1994/device-nerve-esp32/blob/main/docs/step-loss-diagnostic.md) 为准。本页只说明 cartpole 当前消费者使用的接口，不维护另一份固件参数注册表。

## 部署路径

| 路径 | 适用对象 |
|---|---|
| [`device/run_policy.py`](../../deploy/v2/device/README.md) | 当前 Python 单动作 ONNX 运行器；固定 400/500 Hz，速度尺度 1.25 m/s。 |
| [`device/edge/`](../../deploy/v2/device/edge/README.md) | C 会话及接收适配接口；尚未集成目标板 NPU 控制循环。 |
| `pc_runner.py`、[`edge/`](../../deploy/v2/edge/README.md) | 旧 V2 200 Hz 参考控制器。其配置推送、标定及回中流程未适配当前装置，不能直接用于新模型。 |
| [`../protocol.md`](../../deploy/protocol.md) | V1 旧装置的 `S`/`P` ASCII 协议，与 V2 不同。 |

V2 使用 **460800 8N1、ASCII 行文本**，行首、命令名与枚举值区分大小写。完整行以换行结束；接收器允许可选回车。不是二进制帧协议。

## 状态与有效性

```text
STATE <x_um> <theta_urad> <vx_mm_s> <vth_mrad_s>
VEL_VALID <cart> <pole>
MODE <mode> <reason>
```

四个 STATE 数值依次为小车位置（µm）、摆角（µrad）、小车速度（mm/s）和角速度（mrad/s）。标定坐标中点为 x=0；摆角按 ±π 环绕。两路传感器均为 MT6835，但固件小车编码器故障的线协议名称仍为 `AS5600`，不能由名称推断当前器件。

固件状态报告目标为 500 Hz，与上位机 400/500 Hz 决策周期分开。运行器按绝对栅格消费新观测，不是收到每行就推理。状态行不带设备采样时间戳或序号，主机接收年龄不能证明端到端新鲜度；两路传感器也不保证同步。

`VEL_VALID` 两项只能为 0/1，固件在变化时及周期心跳报告；任一路无效时不发送新的 STATE。主机会话要求明确有效性并维护超时，不能仅依据 MODE 或一个看似正常的位置值判定测量有效。故障后的数据恢复不自动恢复同一次控制会话。

## 标定、配置与启动

- `QUERY` 查询模式；`CALIB?` 查询行程、保存状态和本次参考状态。
- 完整标定使用 `MARK LEFT`、`MARK RIGHT`。有可用保存行程时，重启后的左参考确认使用 `CONFIRM LEFT`，不会自动使能。`CAL_STATE=3` 不等于本次定位完成；当前会话预检要求 `CAL_STATE=2`、`CAL_SAVED=1`、`CAL_STORAGE=0`。
- `CONFIG?` 查询全表；`CONFIG? <NAME>` 只读单键，返回 `OK <NAME> <value>`。持续控制运行器轮转单键查询，避免全表回复突发。
- `CONFIG <NAME> <value>` 修改配置。持久化与允许模式见固件文档；不要假设全部运行参数都保存在 NVS。
- `ENABLE` 仍要求有效标定及供电验证；成功后为 `MODE READY NONE`。运行器不替用户调用 ENABLE，不修改固件参数。

默认设备档案保留 500 Hz / 10000 mm/s² 基线；400 Hz Acc9 模型的清单记录目标加速度为 9000 mm/s²，运行器据此核对设备。两者速度尺度均为 1250 mm/s。运行参数是配置值，不是实测机械能力。现场行程、方向与摆角零点必须读回核对，不能用仓库档案覆盖现场事实。

## 动作与停止

`VEL <v_mm_s>` 是目标速度，不是力或 PWM。固件按自身加速度限幅跟踪目标；主机每次动作前检查模式、有效性、配置与时序。当前运行器只接受单动作模型，不使用双动作 `VELA` 实验协议。

正常停止先发 `VEL 0`，继续接收新状态，再请求 `DISABLE`。READY 下引擎仍非零速时，固件返回 `ERR BUSY`；只有确认 `MODE DISABLED NONE` 才记录禁用成功。FAULT 中的 DISABLE 不清除原故障；RESET 是独立的故障清除操作，不是自动恢复运动。

固件看门狗、位置保护与上位机时序检查是不同机制。ONNX 运行器不额外设置位置门或摆角门；通用 Python/C 会话默认仍有位置门，详见各自 README。各机制不保证硬实时或物理停车距离。

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
