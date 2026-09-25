# cartpole 上板执行清单（固件改造收尾，由台位操作者亲自执行）

固件源码：`tmp/PD-STEP`（分支 `cartpole`）。线协议权威：
`deploy/v2/protocol.md`。串口联调工具：固件仓库
`tools/cartpole_host.py`（`pip install pyserial`）。

下述 PowerShell 命令均需先激活环境：

```powershell
. C:\esp\v6.1\esp-idf\export.ps1
cd tmp\PD-STEP
```

按序执行；每步的「记录」列回填后提交（验收记录文件在固件仓库
`docs/reviews/`）。任何一步不符，停下记录现象与日志，反馈后修。

## 阶段 A：selftest 构建（电机不使能，无 12V 也可）

```powershell
idf.py -B build-selftest -DSDKCONFIG=build-selftest/sdkconfig `
      -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/selftest.defaults" build
idf.py -B build-selftest flash monitor
```

上电自运行各组件自测，UART0 控制台核对：

| # | 检查项 | 预期 | 记录 |
|---|---|---|---|
| A1 | 启动横幅 | 无 panic；`bsp: BSP initialization complete`（含 I2C 扫描见 0x36、SPI 初始化）；EN 保持高（电机不使能） | |
| A2 | settings | `SELF-TEST DONE: 21/21 PASS`（默认微步 8 等向量） | |
| A3 | step_engine | `SELF-TEST DONE: 7/7 PASS`（含首拍间隔、看门狗恢复、换向长拍、急停、X_INV 电平翻转）；ISR max/avg cycles 记录备查（第四轮实测 max ≈ 19.5k cycles ≈ 81 µs，avg ≈ 11k ≈ 46 µs，见未决⑤） | ISR max/avg: |
| A4 | mt6835 **首读关卡** | `MT6835 在线——首读 raw21=<非 0 且稳定>`；`SELF-TEST DONE: 3/3 PASS`（静止抖动 ≤ 2 count）。若 raw21 恒 0/乱码 → 核对 T4-brief.md 帧格式 | raw21: |
| A5 | telemetry | `SELF-TEST DONE: 2/2 PASS` | |
| A6 | cartproto | `SELF-TEST DONE: 3/3 PASS`；UART1 冒烟三行 `MODE DISABLED NONE` / `OK VEL_MAX 1000` / `ERR VEL_MAX`（460800，CH343P 口可见）；启动无 `GPIO 41 is not usable` 警告 | |
| A7 | cartfsm 核心 | `SELF-TEST DONE: 3/3 PASS` | |
| A8 | STATE 流 | 460800 抓 10 s：`state --secs 10` → 行率 199–201 Hz、UNKNOWN=0、静止 x_um 漂移 < 1000 µm | 行率: 漂移: | 200.7 Hz / UNKNOWN 0 / 漂移 20 µm（修复 host 工具窗口丢行后实测，eb48ec3） |
| A9 | 手推方向 | 在 as5600 自测末尾的 **MANUAL CHECK（= A9）**段执行：转电机轴（= 经皮带移动小车）3 圈，日志给 `delta` 符号——记下转动方向与符号的关系，留 C4 交叉核对 | x_um 随移动单调变化；符号记入下表 | 符号: |
| A10 | 手转摆杆一圈 | 在 mt6835 自测末尾的 **MANUAL CHECK（= A10）**段执行：提示后 15 s 内将摆杆转一整圈后停在出发点，自动判定 | `A10 判定：圈数守恒 OK（检测到 k 圈，残余 ±x°）`，k 与实际转的圈数一致 | k / 残余: |

## 阶段 B：flowtest 构建（开 EN；有 12V 时小车真实运动）

```powershell
idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig `
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build
idf.py -B build-flowtest flash monitor
```

| # | 检查项 | 预期 | 记录 |
|---|---|---|---|
| B1 | cartfsm 流程自测 | `SELF-TEST DONE: 7/7 PASS`（标定正路径→READY；无左标与 L 非法拒且状态保持；上电未标定 ENABLE 拒；重开标定；READY 偏差→FAULT/STEP_LOSS；AS5600 失联→FAULT（日志含耗时 ms，设计 ≤15）；RESET 后直接 ENABLE 恢复） | 失联耗时: |
| B2 | FAULT 次序 | 日志/MODE 行序：目标置 0 → EN 关 → `MODE FAULT <reason>` | |
| B3 | 行中 `\r` 注入 | 串口发送 `VE\rL 200\n` 与 130 字节乱码+`\n` | 均无任何命令执行、无输出（重同步生效） | `VE\rL 200\n`：无任何回应行 ✓；乱码组待补 |

## 阶段 C：正式固件 + 两点标定（机械在位、12V）

```powershell
idf.py -B build build
idf.py -B build flash
```

工具 `py` = `python tools/cartpole_host.py --port <COM> `。
标定规程（每次上电执行，RAM 标定断电丢失）：推最左 `py mark-left` →
推最右 `py mark-right` → `py enable`。

| # | 检查项 | 操作 | 预期 | 记录 |
|---|---|---|---|---|
| C1 | 标定正路径 | 推最左 `py mark-left` → 推最右 `py mark-right` → `py enable` | 两 `OK` 回显（L ≈ 530 mm）；`MODE READY NONE`；`py state` 静止 x_um ≈ +L/2，回中后 ≈ 0 | L: |
| C2 | 标定负路径 | 先右后左再标、两点重合、未标定即 `py enable` | 均回 `ERR`，状态不变 | |
| C3 | 域对齐 | `py vel 50 --hold 1` 记实际方向 = +x | x_um 增大（L 为正已保证 E 域；此处核指令域，反了改 X_INV） | |
| C4 | 符号约定 | 同向手掰摆顶看 theta | theta 同向增大；`VEL −50` 反向 | |
| C5 | 摆角零位 | 手扶摆杆机械直立读 STATE theta | 读数 θ₀；标定值 = −θ₀ | θ₀: |
| C6 | 标定落地 | `py config POLE_ZERO_URAD <−θ₀>`（及 POLE_INV/X_INV）后复核 | 直立 theta≈0；VEL 50 方向与 theta 符号符合约定 | |
| C7 | 软限位 | 右端 `py vel 200` / `py vel -200`；左端对称 | 向外钳制不动、向内可动 | |
| C8 | 标定重复性 | 同一端点重复标记 3 次读回显 | 散布 ≤ 100 µm | |
| C9 | 看门狗 | `py wdt-test` | ~100 ms 减速到 0；QUERY 回 READY NONE；恢复发送继续跟踪 | |
| C10 | 丢步/拔线/长稳 | READY 中手推小车 ≥3 mm（STEP_LOSS）；拔 AS5600（再拔 MT6835）；`py state --secs 600` | 手推 ≤ T/|v| 内 FAULT STEP_LOSS；拔线 ≤15 ms FAULT 对应 reason、EN 关、STATE 流不断；200 Hz 无 reboot | 手推耗时: 拔线耗时: |

## 阶段 D：标定值写回端侧

C3–C6 结果写入 `deploy/v2/edge/config.h`（会话推送值）：
`POLE_ZERO_URAD / POLE_INV / X_INV`（及实测微调的
`STEP_LOSS_UM`、`X_MARGIN_UM` 如需）。随后按 `docs/reviews/T10-acceptance.md` 走
端到端五用例（含 PC 端侧程序经 CH343P 直连的全形态 CLI）。
若 PC 侧串口设备名与 `CFG_SERIAL_PORT` 不符，一并更新。

## 未决事项出口（本清单完成时逐项关闭）

1. MT6835 帧格式 → A4 关闭（手册已核对，实测为准）。
2. 堵转检测 → 运行中堵转/丢步由 STEP_LOSS 判据捕获（READY 中
   |E−C| 越限即 FAULT）；验证见 C10。
3. `STEP_LOSS_UM` 初值 → C1/C7 实测回填。
4. 460800 联调 → A8 已过（200.7 Hz / UNKNOWN 0 / 漂移 20 µm，
   10 s 窗口）；余 C10 600 s 长跑复核，不过则 230400（两端同步改）。
5. 步进 ISR 吞吐上限 vs VEL_MAX（默认 1000 mm/s ≈ 40 kHz 步频）：
   第四轮均值已回填——ISR avg ≈ 11k cycles ≈ 46 µs（三段均摊），
   **可持续 ≈ 22 kHz ≈ 550 mm/s（40 steps/mm）**；max 19.5k（81 µs，
   离群）。阶段 B/C（≤200 mm/s = 8 kHz）裕量充足，不受影响。
   控制器需要 >550 mm/s（摆上/急回收）之前，做 ISR 直写寄存器
   优化（GPIO W1TS/W1TC + 定时器 alarm 直写，预估合计 ~20 µs ≈
   50 kHz ≈ 1.25 m/s）；间隔采样自测为安全网。优化完成后本条关闭
   并记录实测新上限。
6. 标定重复性 → C8 实测回填（散布 ≤ 100 µm 为传感器噪声级）。
