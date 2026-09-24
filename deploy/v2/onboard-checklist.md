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
| A1 | 启动横幅 | `cartpole firmware up`（app_main），无 panic；EN 保持高（电机不使能） | |
| A2 | settings | `SELF-TEST DONE: 22/22 PASS`（默认微步 8 等向量） | |
| A3 | step_engine | `SELF-TEST DONE: 7/7 PASS`（含看门狗恢复、换向长拍、急停；ISR 最大周期数 < 5000） | ISR cycles: |
| A4 | mt6835 **首读关卡** | `MT6835 在线——首读 raw21=<非 0 且稳定>`；`SELF-TEST DONE: 3/3 PASS`（静止抖动 ≤ 2 count）。若 raw21 恒 0/乱码 → 核对 T4-brief.md 帧格式 | raw21: |
| A5 | telemetry | `SELF-TEST DONE: 2/2 PASS` | |
| A6 | cartproto | `SELF-TEST DONE: 3/3 PASS`；UART1 冒烟三行 `MODE DISABLED NONE` / `OK VEL_MAX 1000` / `ERR VEL_MAX`（460800，CH343P 口可见） | |
| A7 | cartfsm 核心 | `SELF-TEST DONE: 3/3 PASS` | |
| A8 | STATE 流 | 460800 抓 10 s：`state --secs 10` → 行率 199–201 Hz、UNKNOWN=0、静止 x_um 漂移 < 1000 µm | 行率: 漂移: |
| A9 | 手推方向 | 在 as5600 自测末尾的 **MANUAL CHECK（= A9）**段执行：转电机轴（= 经皮带移动小车）3 圈，日志给 `delta` 符号——记下转动方向与符号的关系，留 C4 交叉核对 | x_um 随移动单调变化；符号记入下表 | 符号: |
| A10 | 手转摆杆一圈 | 在 mt6835 自测末尾的 **MANUAL CHECK（= A10）**段执行：提示后 15 s 内将摆杆转一整圈回到出发点，自动判定 | `A10 判定：圈数守恒 OK` | |

## 阶段 B：flowtest 构建（开 EN；有 12V 时小车真实运动）

```powershell
idf.py -B build-flowtest -DSDKCONFIG=build-flowtest/sdkconfig `
       -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;tools/flowtest.defaults" build
idf.py -B build-flowtest flash monitor
```

| # | 检查项 | 预期 | 记录 |
|---|---|---|---|
| B1 | cartfsm 流程自测 | `SELF-TEST DONE: 5/5 PASS`（注入堵转→READY+原点 −265000 µm；超时→FAULT；AS5600 失联→FAULT（日志含耗时 ms，设计 ≤15）；丢步注入→FAULT；RESET 后重 HOME） | 失联耗时: |
| B2 | FAULT 次序 | 日志/MODE 行序：目标置 0 → EN 关 → `MODE FAULT <reason>` | |
| B3 | 行中 `\r` 注入 | 串口发送 `VE\rL 200\n` 与 130 字节乱码+`\n` | 均无任何命令执行、无输出（重同步生效） |

## 阶段 C：正式固件 + 台位标定（机械在位、12V）

```powershell
idf.py -B build build
idf.py -B build flash
```

工具 `py` = `python tools/cartpole_host.py --port <COM> `。

| # | 检查项 | 操作 | 预期 | 记录 |
|---|---|---|---|---|
| C1 | DIAG 极性（未决②） | `py home` 后低速手堵 | 堵转触发 FAULT STALL（或回零完成）；不堵无误触发 1 min | 极性: |
| C2 | SGTHRS 初标（未决③） | `py config SGTHRS <20..80>` 扫描 + 手堵 | 灵敏且空载不误报 | 选定: |
| C3 | 回零散布 | `py home` ×5（间隔 `RESET`），读静止 x_um | 散布 ≤ 2000 µm | 各次 x_um: |
| C3b | **E−C 域对齐关卡**（丢步判据结构前提：DIR 高时 AS5600 计数须增） | 回零后 `py vel 50 --hold 3`：x_um 增大（与 +x 命令同向）、全程无 STEP_LOSS 误报 | 方向一致、无误报；不符 = 机械/接线不对齐，先处理硬件（AS5600_INVERT_DIR）再继续 | |
| C4 | 符号约定（protocol.md §符号约定） | `py vel 200 --hold 1` 记实际方向 = +x；手掰摆顶同向看 theta | theta 同向增大；VEL −200 反向 | |
| C5 | 摆角零位 | 手扶摆杆机械直立，读 STATE theta | 读数 θ₀；标定值 = −θ₀ | θ₀: |
| C6 | 标定落地 | `py config POLE_ZERO_URAD <−θ₀>`（及 C1–C4 所得 POLE_INV/X_INV/SGTHRS）后复核 | 直立 theta≈0；VEL 200 方向与 theta 符号符合约定 | |
| C7 | 看门狗 | `py wdt-test` | ~100 ms 减速到 0；QUERY 回 `MODE READY NONE`；恢复发送继续跟踪 | |
| C8 | 收紧立即生效 | READY 中 `py config X_LIMIT 10000`，再 `py vel 200` | 越界即停向外分量 | |
| C9 | 拔线观察（台位可承受时） | READY 中拔 AS5600（再拔 MT6835） | ≤15 ms FAULT 对应 reason、EN 关、**STATE 流不断**（θ 恒有效） | 耗时: |
| C10 | 长稳 | `py state --secs 600` | 199–201 Hz、UNKNOWN=0、无 reboot | |

## 阶段 D：标定值写回端侧

C1–C6 结果写入 `deploy/v2/edge/config.h`（会话推送值）：
`POLE_ZERO_URAD / POLE_INV / X_INV / SGTHRS`（及实测微调的
`STEP_LOSS_UM`）。随后按 `docs/reviews/T10-acceptance.md` 走
端到端五用例（含 PC 端侧程序经 CH343P 直连的全形态 CLI）。
若 PC 侧串口设备名与 `CFG_SERIAL_PORT` 不符，一并更新。

## 未决事项出口（本清单完成时逐项关闭）

1. MT6835 帧格式 → A4 关闭（手册已核对，实测为准）。
2. StallGuard 低速可靠性（50 mm/s ≈ 1.25 rps）→ C1/C3 判定；
   不触发则提 HOME_VEL 或改 C−E 判据（protocol.md 未决事项路径）。
3. SGTHRS/STEP_LOSS_UM 初值 → C2/C3 实测回填。
4. 460800 联调 → A8/C10；不过则 230400（两端同步改）。
