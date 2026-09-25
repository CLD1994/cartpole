# v2 手动两点标定与运行保护规格

本规格定义手动两点标定（坐标建立）与 READY 运行保护的行为契约。线协议权威仍在
`deploy/v2/protocol.md`；本规格对其命令集、命令×模式矩阵、模式机与参数注册表的
修订随实施同步落地。固件实现于 `tmp/PD-STEP`（分支 `cartpole`）。

坐标由操作者手动推动小车建立：左端、右端各标记一次，中点为 x=0。驱动芯片的
无传感器堵转检测特性不参与任何判定（已退役，见 git 历史）。

## 1. 参与者与信号定义

| 名称 | 定义 | 来源 |
|---|---|---|
| `E` | 实测位置，AS5600 多圈累计换算 µm，1 kHz 采样 | as5600 |
| `C` | 指令位置，step_engine 位置计数换算 µm | step_engine |
| `D` | 位置偏差 `|E − C|` | cartfsm |
| `T` | 丢步判定阈值，参数 `STEP_LOSS_UM` | 参数注册表 |
| `L` | 标定行程，`L = E_right − E_left`（µm） | cartfsm |
| `m` | 软限位两端余量，参数 `X_MARGIN_UM` | 参数注册表 |
| 标定状态 | `CAL_NONE`（未标）/ `CAL_LEFT`（仅左标）/ `CAL_DONE`（完成），RAM，断电即回 `CAL_NONE` | cartfsm |

## 2. 标定行为契约

标定命令仅 DISABLED 态接受（EN 关，操作者可自由手推小车）：

```
MARK LEFT\n     记当前 E 为左端，回 OK MARK LEFT <E µm>
MARK RIGHT\n    记右端并完成标定，回 OK MARK RIGHT <L µm>
ENABLE\n        标定完成后使能进 READY，推 MODE READY NONE
```

回执原则：有 `OK` 回执的命令（`CONFIG` / `MARK` / `ENABLE`）在拒绝时回
`ERR <NAME>`；无回执命令（`VEL` / `RESET`）在不适用态静默忽略。

**标定决策表**（当前态 DISABLED）：

| 标定状态 | 命令 | 结果 |
|---|---|---|
| 任意 | `MARK LEFT` | 记 `E_left = E`；状态 → `CAL_LEFT`；回 `OK MARK LEFT <E>`（重复标记即覆盖重开） |
| `CAL_NONE` | `MARK RIGHT` | `ERR MARK RIGHT`（无左标） |
| `CAL_LEFT` | `MARK RIGHT`，`L` 合法 | 记 `E_right = E`；状态 → `CAL_DONE`；**E/C 同帧重定基**：中点为 x=0（`E、C` 同帧换算，重定基后右端读 `+L/2`）；回 `OK MARK RIGHT <L>` |
| `CAL_LEFT` | `MARK RIGHT`，`L` 非法 | `ERR MARK RIGHT`（保持 `CAL_LEFT`，左标值保留） |
| `CAL_DONE` | `MARK RIGHT` | `ERR MARK RIGHT`（先 `MARK LEFT` 重开标定） |
| `CAL_DONE` | `ENABLE` | → READY（见 §3） |
| `CAL_NONE` / `CAL_LEFT` | `ENABLE` | `ERR ENABLE`（标定未完成） |

**L 合法性**（纯函数，宿主可测）：`100 000 µm ≤ L ≤ 1 000 000 µm` 且
`L > 0`。该判据兼任域对齐关卡：AS5600 计数方向与物理相反时操作者按
「左→右」顺序标记得到的 `L` 为负即被拒，无需独立的方向校验步骤。

标定值仅存 RAM：断电丢失，`RESET` 不清除。重新标定的路径是断电重上电，
或经 FAULT→RESET 回 DISABLED 后再 `MARK LEFT`。

## 3. ENABLE 行为契约

DISABLED 且 `CAL_DONE` 时 `ENABLE` 是进入 READY 的唯一路径，动作次序：

1. **C 重锚到 E**（`step_engine_set_position(um_to_cmd_steps(E))`，引擎停止
   时整定）：标定完成到 ENABLE 之间手挪过小车不产生 STEP_LOSS 误报；
   重锚与其他复位动作先于模式发布完成（监督任务见 READY 即开始判定，
   不得携旧值执行首拍）；
2. EN 开、以零速进入 READY；
3. 持锁切换模式，推 `MODE READY NONE`。

若 ENABLE 时小车已越出软限位窗口（如被推过端点），重锚照常执行；软限位
立即钳制向外分量，向内运动不受限。

## 4. READY 行为契约

| 条件 | 结果 | 说明 |
|---|---|---|
| `D > T` | FAULT / `STEP_LOSS` | 唯一运动故障判据，运行中堵转/丢步由它捕获，延迟 = `T / |v|` |
| AS5600 / MT6835 读取失败 | FAULT / 对应 reason | 不变 |
| `x > L/2 − m` 且 `v > 0`，或 `x < −L/2 + m` 且 `v < 0` | 向外分量钳 0 | 软限位窗口 `[−L/2+m, L/2−m]`，向内不受限；运行中收紧 `X_MARGIN_UM` 立即生效 |

坐标语义：x=0 为两标中点，`x ∈ [−L/2, +L/2]`，与仿真训练的中点为零
分布一致。STATE 流的 `x_um` 自 `CAL_DONE` 起有效（此前为上电累计值）。

命令看门狗、`VEL` 斜坡语义、FAULT 停机次序均不变。

## 5. 线协议修订（落地到 protocol.md）

- 模式机三态：`DISABLED / READY / FAULT`；`HOME` 命令与 `HOMING` 态、
  `HOME_TIMEOUT` 概念移除。转换表：

| 当前态 | 事件 | 次态 |
|---|---|---|
| DISABLED | `MARK LEFT` / `MARK RIGHT`（见 §2 决策表） | DISABLED |
| DISABLED | `ENABLE`（`CAL_DONE`） | READY |
| READY | 丢步 / AS5600 / MT6835 读取失败 | FAULT |
| FAULT | `RESET` | DISABLED |

- 命令×模式矩阵（回执原则见 §2）：

| 命令 | DISABLED | READY | FAULT |
|---|---|---|---|
| `VEL` | 忽略 | 执行 | 忽略 |
| `MARK LEFT` / `MARK RIGHT` | 执行或 `ERR`（§2 表） | `ERR` | `ERR` |
| `ENABLE` | 执行或 `ERR`（§2 表） | 忽略 | 忽略 |
| `RESET` | 忽略 | 忽略 | → DISABLED |
| `QUERY` | 回 `MODE` | 回 `MODE` | 回 `MODE` |
| `CONFIG` | 接受 | 接受 | `ERR` |
| `CONFIG?` | 接受 | 接受 | 接受 |

- reason 终集：`NONE / STEP_LOSS / AS5600 / MT6835`（`HOME_TIMEOUT` 移除）。
- 参数注册表（九项）：删 `HOME_VEL / HOME_TIMEOUT / X_LIMIT`，增
  `X_MARGIN_UM`（默认 10000 µm，合法 1000–50000 µm，含义「软限位两端
  余量」）；`STEP_LOSS_UM` 含义去掉回零角色。
- 操作流程（演示当天）：上电（DISABLED）→ 推最左 `MARK LEFT` → 推最右
  `MARK RIGHT`（回显 L，肉眼核对 ≈530 mm）→ `ENABLE` → 端侧 `VEL` 小速度
  回中 → 扶摆杆、策略接管；FAULT 后 `RESET` 回 DISABLED，标定仍有效，
  直接 `ENABLE` 恢复。
- 端侧 `deploy/v2/edge/`：`rig.c` 的 `CFG_FW_PARAMS` 去掉
  `HOME_VEL/HOME_TIMEOUT/X_LIMIT`（改为九项新集）；console 的回零流程
  改为标定引导（`MARK LEFT → MARK RIGHT → ENABLE → 回中`）。
- 上板清单 C 段改写为标定规程（见 §7）。

## 6. 固件实现范围（建议，非硬性）

- 移除：`cm_homing_event`、`cm_speed_reached`、达速锁存全套、监督任务
  HOMING 分支（续喂/判定）、`HOME_ORIGIN_MM`、flowtest 回零用例、
  `cm_step` 的 `CM_HOMING` 态。
- 新增：`cm_calib_valid(long l_um)` 纯函数（L 合法性）；标定状态机与
  `E_left` 静态量；`CART_CMD_MARK_LEFT / MARK_RIGHT / ENABLE` 命令执行。
- 保留：STEP_LOSS 运行保护、E/C 同帧重定基与重锚、软限位钳制框架
  （`cm_soft_limit_clamp` 改窗口化签名）、StallGuard/DIAG/SGTHRS 退役状态。
- 文档：删 `homing-spec.md` / `homing-plan.md`；`protocol.md` 与
  `onboard-checklist.md` 按 §5/§7 重写相关节。

## 7. 测试与验收

**宿主纯核心向量**：三态转换表逐行；标定决策表逐行（含 `L` 边界
99999/100000/1000000/1000001 µm）；`cm_soft_limit_clamp` 窗口化边界；
命令×模式矩阵。

**flowtest（注入式，仅 `CONFIG_CARTFSM_FLOW_TEST` 编译）**：`MARK` 的
`E` 读取经一次性 `loss_bias` 注入提供两标位置（「手在别处」的替身，
判定拍/标记读数消费即清）：

| 用例 | 序列 | 期望 |
|---|---|---|
| 标定正路径 | 注入 E₁ → `MARK LEFT` → 注入 E₂（E₂−E₁ 合法）→ `MARK RIGHT` → `ENABLE` | 两 `OK` 回显；READY；E/C 重定基后无 STEP_LOSS 误报 |
| 无左标 | `MARK RIGHT` | `ERR`，状态不变 |
| L 非法 | 两点重合（E₂=E₁） | `ERR MARK RIGHT`，保持 `CAL_LEFT` |
| 未标定 ENABLE | 上电即 `ENABLE` | `ERR ENABLE` |
| 重开标定 | 完成后 `MARK LEFT` → 新 `MARK RIGHT` → `ENABLE` | 新 L 生效 |
| READY 偏差 | 标定+ENABLE 后注入偏差 > T | FAULT / `STEP_LOSS` |
| 失联优先 / RESET 恢复 | 既有口径 | 不变 |

**上板验收**（清单 C 段新规程）：

| 项 | 操作 | 预期 |
|---|---|---|
| 标定正路径 | 推左标左、推右标右、`ENABLE` | 回显 L 合理（≈530 mm）；`MODE READY NONE`；回中后 x_um≈0，两端 ≈±L/2 |
| 负路径 | 先右后左（L<0）、两点重合、未标定 `ENABLE` | 均回 `ERR`，状态不变 |
| 软限位 | 右端 `VEL +200` / `VEL −200`；左端对称 | 向外钳制不动、向内可动 |
| 标定重复性 | 同一物理端点重复标记 3 次读 `OK` 回显 | 散布 ≤ 100 µm（传感器噪声级） |
| 既有保护 | STEP_LOSS（手推 ≥3 mm）、拔线、看门狗、长稳 | 与原口径一致 |

## 8. 已知局限

- 软限位窗口的绝对位置取决于操作者是否推到真实机械端点；L 回显供肉眼
  核对（≈530 mm），显著偏小即提示未推到位。
- 标定期间 `E` 的固有跳变（台位已知现象，+10.4 mm 量级）若发生在标记
  读数那一拍，会等量偏移该端点；对软限位与中点的影响为常值偏移，不随
  运行放大。
- `ENABLE` 后的 `VEL` 跟踪依赖电机与驱动硬件正常（台位电机抖动问题
  独立待查，不在本规格范围）。

## 9. 未决事项

- `X_MARGIN_UM` 默认 10000 µm 与合法范围上机微调。
- `L` 合法窗 100–1000 mm 是否按台位实测行程（≈530 mm）收紧。
