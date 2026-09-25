# v2 回零与运行期偏差判定规格

本规格定义回零完成信号与 READY 运动故障判定的行为契约。线协议权威仍在
`deploy/v2/protocol.md`；本规格对其转换表、reason 枚举与参数注册表的修订
随实施同步落地。固件实现于 `tmp/PD-STEP`（分支 `cartpole`）。

回零完成信号采用**位置偏差判据**：小车道电机堵转后，指令位置继续前进而
AS5600 实测位置停转，两者偏差越限即判定到位。驱动芯片自带的无传感器堵转
检测特性不参与任何判定，相关配置项与引脚不占用（见 §5）。

## 1. 参与者与信号定义

| 名称 | 定义 | 来源 |
|---|---|---|
| 监督任务 | 1 kHz 周期（CPU1），回零与运行判定的唯一判定点 | cartfsm |
| `C` | 指令位置，step_engine 位置计数换算为 µm | step_engine |
| `E` | 实测位置，AS5600 多圈累计换算为 µm | as5600 |
| `v_meas` | 实测速度，监督任务内对 `E` 作毫秒差分（µm/ms 即 mm/s） | cartfsm |
| `D` | 位置偏差，`|E − C|`，µm | cartfsm |
| `T` | 偏差判定阈值，即参数 `STEP_LOSS_UM` | 参数注册表 |
| `latch` | 达速锁存，布尔，见 §2 | cartfsm |

`T` 由参数 `STEP_LOSS_UM` 单一权威承担双角色：READY 丢步故障阈值与
HOMING 到位判定阈值共用，不新增 HOME 专用阈值参数。

## 2. HOMING 行为契约

进入条件不变：DISABLED 收到 `HOME`。监督任务按拍续喂回零速度
`−HOME_VEL`（方向 −x），总时长由 `HOME_TIMEOUT` 把关，`RESET` 中止回零
——以上均为既有行为，不变。

**HOME 入口重锚**：进入 HOMING 时固件把指令位置 `C` 整定到当前实测
位置 `E`（引擎停止时整定），`D` 自入口从 0 起算。DISABLED 期间电机
失电、小车可被手移，`C` 不再反映实况；不重锚则故障后的 `D` 残留使
「挪车后重试」（§6 负路径恢复）与 protocol.md 操作流程第 7 步的
FAULT→RESET→HOME 恢复环路不可达。

**达速锁存 `latch`**：进入 HOMING 时清零；连续 2 拍满足
`v_meas ≤ −0.8 × HOME_VEL`（含符号，回零方向 −x）即置位，置位后保持到
离开 HOMING，不复位。2 拍约束用于抑制 AS5600 单拍计数噪声（±2 count
≈ ±20 mm/s）。`latch` 的物理含义：编码器实测小车确实朝挡块方向跑起来过。
它以 C3b 域对齐（`DIR` 有效时 AS5600 计数增）为前提——C3b 通过后 `E` 与
`C` 同号域，回零命令必使 `E` 递减，符号判据在任意正确配置的台位上成立。

监督任务每拍按优先序判定，同拍多条件命中时取序号最小者：

| 序 | 条件 | 结果 |
|---|---|---|
| 1 | AS5600 读取失败 | FAULT / `AS5600`（`E` 无效时不得用于任何判定，故最先判） |
| 2 | MT6835 读取失败 | FAULT / `MT6835` |
| 3 | `latch = 0` 且 `D > T` | FAULT / `STEP_LOSS`（起点压挡块、皮带卡死或域未对齐；日志提示挪车后重试） |
| 4 | `latch = 1` 且 `D > T` | READY：急停 + E/C 同帧重定基到 −265000 µm，推送 `MODE READY NONE` |
| 5 | 超过 `HOME_TIMEOUT` | FAULT / `HOME_TIMEOUT` |

序 3 与序 4 是同一偏差事件按 `latch` 分类：达速后越限是到位，未达速越限
是故障。序 4 优先于序 5：超时拍上同时满足到位条件时判到位。

完成动作的次序与原子性沿用既有实现：停机 → `as5600_set_origin(−265.0 mm)`
→ `step_engine_set_position(−265.0 mm)`（E、C 必须同帧换到新坐标系）→
持锁切换模式 → 推送 MODE 行。

**判定的物理语义与时序**：小车抵挡块后电机堵转，`E` 停止，`C` 以
`HOME_VEL` 速率继续前进，`D` 随之单调增长。确认延迟 = `T / HOME_VEL`
（默认 3000 µm @ 50 mm/s = 60 ms）。延迟期间小车物理位置不变——挡块
约束下电机无法继续前进，`C` 的账面超前在重定基时丢弃。空载巡航时 `D`
仅为负载角滞后与皮带弹性，量级远低于 `T`（C3b 以同阈值口径验证），故
`D` 的增长只可能来自电机停转，单拍越限即判定，无去抖。

**诊断输出**：FAULT/`HOME_TIMEOUT` 时日志打印 `E`、`C`、`v_meas` 与
`latch` 状态，区分「从未达速」（电机未随动或速度过低）与「达速后未撞挡」
（传动脱离）。FAULT/`STEP_LOSS`（HOMING 中）时日志提示：起点压挡块、
卡死或域未对齐，挪车后重试。

## 3. READY 行为契约（修订）

| 条件 | 结果 | 说明 |
|---|---|---|
| `D > T` | FAULT / `STEP_LOSS` | 唯一运动故障判据；运行中堵转由它捕获，延迟 = `T / |v|`（200 mm/s 时 15 ms） |
| AS5600 / MT6835 读取失败 | FAULT / 对应 reason | 不变 |
| `|x| > X_LIMIT` | 钳制向外速度分量 | 不变 |

回零完成后小车抵着挡块由保持电流顶住属正常状态：静止时 `C` 不前进，
`D` 不增长，无误报。堵转专用 reason（`STALL`）不存在，见 §4。

## 4. 线协议修订（落地到 protocol.md）

- 转换表 HOMING 行事件「堵转触发」→「到位判定（达速后 `|E−C|` 越限）」，
  次态与触发点语义不变（仍为 READY、`x = −265000 µm`）。
- 转换表 READY 行事件「丢步 / 堵转（运动中）/ 编码器失败」→「丢步 /
  编码器失败」。
- reason 枚举移除 `STALL`，终集为 `NONE / STEP_LOSS / AS5600 / MT6835 /
  HOME_TIMEOUT`。
- 参数注册表移除 `SGTHRS`；`STEP_LOSS_UM` 条目补注双角色与延迟公式。
- 端侧 `deploy/v2/edge/`：`proto` 枚举移除 `FW_REASON_STALL`；参数
  推送表（`rig.c` 的 `CFG_FW_PARAMS`）本不含 `SGTHRS`，无需改动。
- `deploy/v2/onboard-checklist.md` 同步：C1 改写为到位判定验证（含负路径），
  C2（SGTHRS 扫描）删除，阶段 D 标定集相应缩减，未决②③关闭或改写。

## 5. TMC2209 驱动配置（目标状态）

驱动芯片承担基础驱动职责：运行电流与 chopper 配置（保持 SpreadCycle，
不回 StealthChop——不为噪音引入行为变量）。下述能力不使用、配置不写入、
引脚不占用：

- StallGuard 阈值（`SGTHRS`）与冷却阈值（`TCOOLTHRS`）写入；
- DIAG 输出：GPIO 读取、内部上拉、极性 Kconfig 选项（含三份存量
  sdkconfig 中符号的清理）；
- `SG_RESULT` / `TSTEP` 诊断读数与回零途中的周期日志；
- `cartfsm_report_stall` 注入接口。

## 6. 测试与验收

**流程自测（flowtest，注入式）**。注入点设在判定入口，真实代码路径完整：

- `loss_bias`（既有）：对 `E` 注入偏置，驱动 `D`；
- `follow_cmd`（新增，仅 flowtest 编译）：判定用 `E` 以指令位置替代
  ——理想编码器替身，使超时/失联用例不依赖台位 12V；
- `speed_latch` 覆写（新增，仅 flowtest 编译）：-1 自然判据（默认）、
  0 强制清零、1 强制置位，替代真实运动置位 `latch`。

| 用例 | 注入序列 | 期望 |
|---|---|---|
| 到位 | HOME → 锁存置位 → `loss_bias` > `T` | READY，静止 `x_um = −265000` |
| 未达速偏差 | HOME → `loss_bias` > `T`（锁存保持 0） | FAULT / `STEP_LOSS` |
| READY 偏差 | READY → `loss_bias` > `T` | FAULT / `STEP_LOSS`（既有） |
| 超时 / 失联 / RESET 中止 / 重回零 | 既有用例 | 不变（超时用例的持续运动断言保留） |
| 挪车重回零 | HOMING 未达速偏差 FAULT → RESET → 清偏置重 HOME → 注入到位 | READY，静止 `x_um = −265000`（入口重锚回归锚） |

`cm_step` 宿主向量同步：HOMING 完成事件的语义为「到位」，READY 不再
消费该事件。

**上板验收**（清单改写后）：

| 项 | 操作 | 期望 |
|---|---|---|
| 正常回零 | `py home`（HOME_VEL 50） | 小车走到挡块停住，`MODE READY NONE`，`x_um ≈ −265000` |
| 负路径 | 起点压挡块时 `py home` | FAULT / `STEP_LOSS`，日志含挪车提示；挪车 ≥5 mm 后重 HOME 正常完成 |
| 静置无误报 | READY 抵挡块静置 1 min | 无 FAULT |
| 回零散布 | `py home` ×5（间隔 RESET） | 散布 ≤ 2000 µm（C3，不变） |
| 域对齐 | `py vel 50 --hold 3` | x_um 增大、无 STEP_LOSS 误报（C3b，前提地位不变） |
| 运行保护 | READY 手推小车 ≥3 mm | FAULT / `STEP_LOSS`（既有行为复核） |

## 7. 已知局限

- 皮带跳齿（电机转、小车不动）不可见：AS5600 装于电机侧，`E` 跟随电机。
- AS5600「I2C 应答正常但数值冻结」若发生在 `latch` 置位后，会在错误位置
  判到位；此情形无检测手段，接受为已知风险。
- 确认延迟 `T / |v|` 随速度降低而增大，回零语义下无影响。

## 8. 实现范围（建议，非硬性）

`cartfsm`（判定重写、注入口、params 去 `sgthrs`）、`cartfsm_core`（事件
语义）、`tmc2209`（瘦身）、`bsp`（DIAG 移除）、`cartproto`（reason 字符串）、
flowtest/selftest 用例、三份存量 sdkconfig 符号清理、`protocol.md` 与
`onboard-checklist.md` 修订、端侧 `edge/proto.*` 与 `config.h`、host 工具
文档。模块划分与文件拆分允许实施者按实际代码结构调整。

## 9. 未决事项

- `STEP_LOSS_UM` 双角色下的默认值实测微调（C3/C3b 口径回填）。
- 回零散布实测值（预期量级：挡块接触弹性决定，与检测方式无关）。
