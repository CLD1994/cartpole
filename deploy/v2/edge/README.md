# 端侧控制程序（cartpole_edge）

运行在 Hi3516CV610（Linux + NPU）上的单进程、单线程 C 程序，是新倒立摆装置的端侧大脑：正式运行时（`run`）以 200 Hz 从 ESP32-S3 固件读取状态遥测，调用 NPU 上的量化策略模型做推理，把动作换算为目标速度发回固件；不带模型时它是板端联调工具（交互控制台与一次性命令），覆盖协议全部控制面命令。它不直接驱动电机——电机由固件驱动，本程序只发出速度命令并做端侧安全门控。

```
训练侧（另一环境）                      本程序所在板                控制板
ONNX → 量化 → policy.om ──scp──>  Hi3516CV610                ESP32-S3 固件
                                   cartpole_edge              ├─ 步进电机（TMC2209）
                                      │  ↑ STATE 200 Hz 遥测   ├─ AS5600 小车位置
                                      │  ↓ VEL/HOME/CONFIG …   └─ MT6901 摆角（SPI）
                                      └───── UART 460800 8N1 ──┘
```

职责划分：本程序负责策略推理、控制节奏与端侧门控；固件负责斜坡跟踪、命令看门狗、软限位、模式机与回零。两侧的安全语义是分层的，任何一侧的修改都应以 `deploy/v2/protocol.md` 为共同基准。

## 权威文档

本文件描述程序结构、构建与运行；跨端行为以三份权威文档为准，冲突时以下列表为准：

| 文档 | 管辖范围 |
|---|---|
| `../protocol.md` | 串口协议（消息格式、固件模式机、命令 × 模式矩阵、参数注册表、演示日操作流程） |
| `../sim2real-spec.md` | 训练侧契约（观测顺序与单位、动作语义与 `V_MAX`、执行器模型、随机化范围） |
| `npu/README.md` | 推理库依赖（接口契约、SDK 链接、工具链位置） |

## 目录结构

| 文件 | 职责 |
|---|---|
| `main.c` | 命令行分发、策略模式（`run`）启动序列与控制环、一次性子命令 |
| `config.h` | 本台位全部常量：串口、控制频率、动作尺度、门控阈值、推送给固件的参数表 |
| `rig.h/.c` | 台位会话：串口 + 协议接收 + 遥测快照 + 模式跟踪 + 共用运动流程，三种运行形态共用 |
| `console.h/.c` | 交互控制台：命令解析（纯函数）、单命令执行器（与一次性子命令共用）、行循环与速度保持 |
| `uart.h/.c` | 串口封装（termios 原始模式、460800、非阻塞读、阻塞写） |
| `proto.h/.c` | 协议行解析与格式化（`STATE`/`MODE`/`OK`/`ERR` 解析；`VEL`/`CONFIG` 等格式化；逐行接收器） |
| `policy.h/.c` | 策略推理封装：SVP ACL 运行时生命周期 + `cartpole_npu` 调用 |
| `proto_test.c` | 协议解析器的主机单元测试 |
| `policy_test.c` | 推理封装的主机单元测试（链接级替身，不接触 NPU） |
| `console_test.c` | 控制台命令解析的主机单元测试 |
| `npu/` | 外部推理库产物（`cartpole_npu` 头文件、ARM 静态库、参考程序，见其 README） |
| `CMakeLists.txt` | 交叉构建定义（由 `build.sh` 驱动） |
| `cmake/arm-linux.cmake` | CMake 工具链文件（从环境变量 `CROSS_COMPILE` 取编译器前缀） |
| `Makefile` | 主机单元测试构建 |
| `setup-wsl.sh` | 一次性：把交叉编译交接包导入 WSL 并恢复完整性（产物位于 WSL `~/cv610`） |
| `build.sh` | 日常交叉编译入口，含产物自检 |

## 程序行为

### 运行形态

三种形态共用同一个会话模块（`rig.c`）与统一退出路径：

| 形态 | 命令 | 用途 |
|---|---|---|
| 策略模式 | `run --model path.om [--no-home]` | 正式控制：加载模型、回零回中、200 Hz 控制环 |
| 交互控制台 | 无子命令或 `console` | 维护与板端联调：逐条发协议命令，实时看 MODE 事件与回执，不接触 NPU |
| 一次性子命令 | `home`/`center`/`vel`/`state`/`query`/`config`/`reset` | 脚本化单条命令，直接透传固件 |

参数推送策略：run 与 console 在会话开始推送 `CFG_FW_PARAMS` 表（参数权威在端侧，见 `config.h` 一节）；一次性子命令不推送——输入什么就发什么，便于隔离测试固件默认行为。

### 策略模式启动序列（run）

`run` 按以下顺序执行，任何一步失败都进入统一退出路径（发 `VEL 0`、释放资源、非零退出码）：

1. 解析命令行，打开串口。
2. `policy_init` 初始化 SVP ACL 运行时（`svp_acl_init` + `svp_acl_rt_set_device(0)`）并加载 `.om` 模型（`--model` 必填，加载失败即退出，不降级）。
3. 发 `QUERY` 同步固件当前模式（1 秒内无应答则判定固件未运行）。
4. 按次序推送 `CFG_FW_PARAMS` 全部参数（逐条 `CONFIG`，间隔 10 ms 防淹没；`ERR` 应答打印告警但不中断）。
5. 若固件处于 DISABLED 且未指定 `--no-home`：发 `HOME` 等 `MODE READY`（35 秒超时，对应固件 `HOME_TIMEOUT` 30 秒加裕量），随后以 `CFG_CENTER_VEL_MMS` 把小车从回零点（−265 mm）移回行程中点。
6. 进入控制环，提示操作者把摆杆扶到直立附近接管。

### 控制台与一次性命令

命令集（控制台内全小写，一次性子命令同名）：

| 命令 | 行为 |
|---|---|
| `vel <mm/s>` | 控制台：设定持续保持的目标速度，程序每 20 ms 重发喂固件看门狗（`WDT_MS` 100），直到改目标、离开 READY、STATE 过期或退出。QUERY/STATE 等只读命令不打断保持；HOME/CENTER/RESET/CONFIG 会先释放保持 |
| `vel <mm/s> --hold <s>`（仅子命令） | 定时保持：持续重发 s 秒（默认 1.0）后发 `VEL 0` 退出；期间离开 READY 或 STATE 过期立即停车并以非零码退出 |
| `home` / `home --center`（子命令） | 发 `HOME` 等 READY（35 秒超时；要求当前 DISABLED）；`--center` 追加回中 |
| `center` | 以 `CFG_CENTER_VEL_MMS` 移回行程中点（要求 READY） |
| `state` | 打印最新遥测快照（协议原单位 + 新鲜度 + 模式） |
| `query` | 发 `QUERY`，打印 `MODE <mode> <reason>` |
| `config` / `config NAME VALUE` | 全量 dump（发 `CONFIG?`，收集 0.5 秒应答）/ 设单参数并等回执 |
| `reset` | 清 FAULT 回 DISABLED（要求当前 FAULT） |

保持类速度命令启动前都会先等 STATE 流就位：「从未收到」与「已过期」是两个状态——前者等 `CFG_STATE_TIMEOUT_S` 后报无流退出，后者立即停车。控制台里 MODE 转换与 OK/ERR 回执始终打印，200 Hz STATE 流不刷屏（`state` 按需查看）。控制台与 run 退出（quit/Ctrl+D/Ctrl+C）统一发 `VEL 0`；一次性命令不在退出时干预电机（vel 自身管理起停）。

### 控制环（200 Hz）

控制周期 5 ms，用 `clock_nanosleep(TIMER_ABSTIME)` 对绝对时刻睡眠，不受单次处理耗时漂移影响。每个周期先泵串口更新遥测快照，然后依次过四道门，任何一道不过就不发本周期命令：

1. **固件模式门**：最后已知模式必须是 `READY`（否则不发——固件本也不会执行 `VEL`）。
2. **状态新鲜度门**：距最近一条 `STATE` 超过 `CFG_STATE_TIMEOUT_S`（0.2 秒）时发 `VEL 0` 并等待恢复。
3. **摆杆接管门**：`|θ|` 超过 `CFG_ENGAGE_ANGLE_RAD`（30°）时发 `VEL 0`——摆杆下垂或大倾角不驱动，等人手扶到直立附近。
4. **推理门**：组装观测调用 `policy_forward`；推理失败（含输出非有限值）视为致命错误，直接走统一退出路径——推理库契约规定失败后句柄失效，不允许带病继续。

过门后把动作钳位到 ±1、乘 `CFG_V_MAX_MPS` 换算为 mm/s 发出 `VEL`。发出前还有最后一道**轨道端头门**：`|x|` 超过 `CFG_TRACK_LIMIT_M` 时只封锁向外的速度、保留向内纠偏。

### 退出路径

SIGINT、SIGTERM、推理失败、启动序列任何一步失败，都汇入同一段清理：会话形态（run/console）先发 `VEL 0` 停机，再 `policy_deinit`（`close` → `reset_device` → `finalize` 退出梯子，未初始化时是幂等空操作），最后关闭串口；一次性命令不发退出 `VEL 0`（vel 自身管理起停）。固件侧另有命令看门狗（`WDT_MS` 100 ms 无有效 `VEL` 即减速停车）作为本程序死亡时的独立兜底，两层互不依赖。

## 配置（config.h）

换台位或调整参数只改 `config.h`，重新构建。各常量的含义与联动关系如下：

| 常量 | 默认值 | 含义与联动 |
|---|---|---|
| `CFG_SERIAL_PORT` | `/dev/ttyAMA1` | CV610 上连 ESP32-S3 的 UART 设备节点，**实机设备名待确认** |
| `CFG_BAUD` | 460800 | 协议规定的线路速率；联调不通过时与协议一起降 230400 |
| `CFG_CONTROL_HZ` | 200 | 与训练 `dt` 一致（`sim2real-spec.md`），改动意味着重训 |
| `CFG_STATE_TIMEOUT_S` | 0.2 | 状态新鲜度门限，须大于固件看门狗容忍度对应的流间隔裕量 |
| `CFG_V_MAX_MPS` | 1.0 | 动作尺度，**必须等于训练 `V_MAX_SIM` 随机化上界**（当前 0.8–1.0 取上界） |
| `CFG_TRACK_LIMIT_M` | 0.260 | 端侧软限位，与固件参数 `X_LIMIT` 保持一致（端侧再挡一层向外速度） |
| `CFG_ENGAGE_ANGLE_RAD` | 0.524（30°） | 策略接管门限；大于训练可捕获角（约 ±15–20°），仅作为驱动许可而非恢复能力 |
| `CFG_CENTER_VEL_MMS` | 100 | 回零后把小车移回行程中点的速度 |
| `CFG_FW_PARAMS[]` | 见 `main.c` | 会话启动时经 `CONFIG` 推送的固件参数表。**参数权威在端侧**：固件只持默认值；表中每项的值必须落在协议注册表的合法范围内，否则固件回 `ERR` |

固件参数表当前推送 7 项（`VEL_MAX` 1000、`ACC_MAX` 5000、`WDT_MS` 100、`WDT_DEC` 5000、`HOME_VEL` 50、`HOME_TIMEOUT` 30000、`X_LIMIT` 260000）。新增固件参数需要三处联动：本表、`../protocol.md` 注册表、固件实现。

## 观测与动作的对齐

观测顺序、单位与符号以 `../sim2real-spec.md` 为准：`[x m, θ rad, ẋ m/s, θ̇ rad/s]`，直立为零、+x 方向按协议符号约定。`STATE` 行的整数（µm / µrad / mm/s / mrad/s）在控制环内换算为 SI 后送推理。策略输出是未缩放的原始动作（训练值域约定 ±1），本程序钳位后乘 `CFG_V_MAX_MPS` 得到 m/s，再换算为整数 mm/s 发 `VEL`。改观测顺序、单位或动作尺度等同于改模型输入输出契约，必须与训练侧同步并重训重导。

## 构建与测试

全部构建活动在 WSL（Ubuntu）内进行；Windows 侧只做源码编辑与 git。仓库 `.gitattributes` 已强制 `*.sh` 为 LF，脚本可直接在 WSL 执行。

### 主机单元测试（不依赖硬件与 SDK）

```sh
cd deploy/v2/edge && make test
```

`proto_test` 覆盖协议解析的合法/非法分支（STATE/MODE/OK/ERR、垃圾行、格式化拒绝 NaN 与超界、逐行接收的分帧与超长截断）；`policy_test` 用链接级替身按 `cartpole_npu.h` 的注释语义模拟推理库与 SVP ACL，覆盖初始化三处失败、打开残留 close-only 句柄、close 失败扣住运行时、推理失效后不得复用、非有限观测/动作、重复初始化拒绝等 11 个用例；`console_test` 覆盖控制台命令解析的合法/非法分支（vel 数值校验、config 参数名与值约束、未知命令、多余参数、大小写敏感）。

### 依赖导入（一次性）

```sh
bash deploy/v2/edge/setup-wsl.sh /mnt/c/<交接包路径>/cross-compiling
```

把 CV610 侧项目提供的交叉编译交接包（工具链 + SVP ACL SDK）复制进 WSL ext4 `~/cv610`，重建符号链接、恢复 NTFS 拷贝受损的内核头（取回内容须与交接清单 sha256 一致才落盘），最后跑完整 `SHA256SUMS` 校验。脚本幂等，可重复执行。

### 交叉编译（日常）

```sh
bash deploy/v2/edge/build.sh
```

默认从 `~/cv610` 取依赖（`CROSS_ROOT` 可指向其他交接包位置），每次全新构建以避免陈旧缓存，编完自动做产物自检：ELF32 / ARM / soft-float ABI / musl 解释器、8 个 SDK 动态依赖齐全、无 RPATH。产物为 `build-arm/cartpole_edge`，与 `policy.om` 一起 scp 上板。可选的动态加载检查（仅证明库加载，不代表 NPU 就绪）：用 `qemu-arm-static` 携带 `~/cv610/sdk/lib` 与工具链 target 库运行二进制，预期到达程序自身逻辑（如串口打不开的错误）。

## 上板部署与运行

板端需已配置匹配的 SDK 驱动、服务与 musl 运行库（沿用 CV610 侧项目已验证的启动环境）。部署：`scp build-arm/cartpole_edge <board>:`，要跑策略时再带上 `policy.om`（控制台与一次性命令不需要它）。运行：

```sh
./cartpole_edge [--port /dev/ttyXXXX] [command [args]]
```

`--port` 是全局参数，必须位于子命令之前，省略时用 `config.h` 的 `CFG_SERIAL_PORT`。子命令：

| 命令 | 说明 |
|---|---|
| （无子命令）/ `console` | 交互控制台（默认形态，见「控制台与一次性命令」） |
| `run --model path.om [--no-home]` | 策略模式（`--model` 必填） |
| `home [--center]` | 一次性：回零等 READY，`--center` 追加回中 |
| `center` | 一次性：移回行程中点（要求 READY） |
| `vel <mm/s> [--hold s]` | 一次性：保持速度 s 秒（默认 1.0）后停车 |
| `state` | 一次性：打印最新遥测快照 |
| `query` | 一次性：打印固件当前模式 |
| `config [NAME VALUE]` | 一次性：设单个参数，或无参数全量 dump |
| `reset` | 一次性：清除 FAULT |

控制台内命令为同名全小写形式（`vel`/`home`/`center`/`state`/`query`/`config`/`reset`/`help`/`quit`）。正常演示流程（含 FAULT 恢复）见 `../protocol.md` 的「操作流程」一节。

## 维护者须知

以下不变量跨多个文件或跨端生效，改动前先核对对应权威文档：

1. **观测/动作契约与训练绑定**：顺序、单位、符号、`CFG_V_MAX_MPS` 任一变动都必须与训练侧同步（`../sim2real-spec.md`），否则是静默的性能劣化。
2. **推理失败即致命**：`cartpole_npu` 契约规定 infer 失败或输出非有限值后句柄只能 close。`main.c` 据此把推理失败实现为停机退出，不要把它改成跳过或重试。
3. **`npu/` 是外部产物**：来自 CV610 侧项目，接口契约以 `npu/include/cartpole_npu.h` 注释为准，本仓库不修改其内容；升级时整套（include + lib + src）一起替换并重跑 `policy_test`。
4. **双层安全语义**：端侧门控（模式/新鲜度/接管角/端头）与固件兜底（看门狗/软限位/模式机）各管一层，新增或放宽任何一层前先读 `../protocol.md`，确认另一层仍覆盖对应失效模式。
5. **参数三处联动**：`CFG_FW_PARAMS`、协议注册表、固件实现必须同时更新，值不得超出注册表合法范围。
6. **「STATE 从未收到」与「STATE 已过期」是两个状态**：保持与控制路径不得把起始竞态当作流过期处理（先等 `CFG_STATE_TIMEOUT_S` 判无流，收到过才谈过期）。vel 保持的重发间隔（20 ms）必须显著小于固件 `WDT_MS`。

## 未决事项

- `CFG_SERIAL_PORT` 的实机设备名（`/dev/ttyAMA1` 为占位）。
- 460800 两端联调；不通过则连同协议降 230400。
- 固件速度估计器定型后，训练侧观测噪声与端侧新鲜度门限的复核（见 `../sim2real-spec.md` 未决事项）。
- `policy.om` 到位后的板端端到端联调与推理耗时复测。
