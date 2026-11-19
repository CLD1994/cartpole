# CV610 端侧推理与控制

本目录在 Hi3516CV610 上显式选择 CPU FP32 或 NPU `.om` 策略，读取 ESP32 的串口状态并发送速度目标。训练和模型事实来自正式 ONNX 目录的 `model.json`，端侧模型包由工具生成。程序不自动修改固件配置、使能或回中。

## 构建

默认 CPU-only 构建只需要 ARM32 musl 交叉编译器，不依赖 NPU SDK。启用 NPU 后端才需要配套 SVP ACL SDK 和本目录的真实 NPU 静态库。交接包放在 WSL 内部 ext4 文件系统，不能把工具链长期放在 NTFS。

```sh
bash deploy/v2/edge/setup-wsl.sh /mnt/c/<交接包目录>
bash deploy/v2/edge/build.sh  # 默认 EDGE_ENABLE_NPU=OFF
EDGE_ENABLE_NPU=ON bash deploy/v2/edge/build.sh  # 同时包含 CPU/NPU
```

默认依赖目录为 `~/cv610`，可用 `CROSS_ROOT` 指定。产物是 `deploy/v2/edge/build-arm/cartpole_edge`。构建脚本检查 ARM32、musl 解释器、SDK 动态依赖和 RPATH；这些检查不代表真实 NPU 或控制性能已验收。导入时必须检查完整性报告，不能忽略工具链或 SDK 文件校验失败。

模型转换工具 ATC 与交叉编译 SDK 是不同依赖。须使用与板端匹配的 Hi3516CV610 工具包，将当前 ONNX 转换为 OM；ONNX 的输入是 FP32 `[1,4]` 状态而不是图像，不能直接套用图像预处理示例。转换产物须通过同输入的 ONNX/OM 数值回放，再在板端测量推理耗时。参见 [NPU 接口](npu/README.md)。

## 准备模型包

在安装了 `onnx` 的 Python 环境执行（`--om` 可省略，生成 CPU-only 包）：

```sh
python deploy/v2/edge/prepare_package.py \
  --model /path/to/exported \
  --om /path/to/policy.om \
  --output /path/to/new-edge-package
```

输入目录须包含 `model.json`、`policy.onnx` 和其外部权重。工具检查来源散列和输入输出形状，派生频率、动作尺度和运行限值，导出 FP32 权重 `policy.fp32`，可选复制 OM，生成 JSON `edge-package.json`（`schema_version: 2`）。CPU 仅支持明确验证过的前馈算子图，未知结构直接拒绝；不隐式更换模型。它不能证明提交的 OM 确实由该 ONNX 转换；模型转换和数值对照仍是部署验收的一部分。包内 SHA256 是来源记录，C 运行器不进行运行时散列校验；生成后应整体部署，不单独替换文件。

## 板端使用

CPU 后端不需要 NPU 服务或动态库。NPU 后端需要匹配的驱动、SDK 动态库与已启动的 `sdk_server`。若板端现有服务未运行，可手动执行 `/usr/bin/sdk_server &`；不自动设自启，不同时重复启动服务。复制生产程序和整个模型包目录；不要部署 `.test-build/` 中任何文件。

```sh
./cartpole_edge --port /dev/ttyXXXX query
./cartpole_edge --port /dev/ttyXXXX state --secs 1
./cartpole_edge --port /dev/ttyXXXX enable
./cartpole_edge --port /dev/ttyXXXX run --backend cpu --package /path/to/new-edge-package/edge-package.json
```

`--backend cpu|npu` 必须显式选择，缺模型或该构建不支持时直接失败，不静默回退。

`--port` 放在子命令之前；默认 `/dev/ttyAMA1` 只是可配置默认值，实际 UART 节点必须按接线确认。波特率固定 460800。

启动策略前，固件须已完成左参考确认、处于 `READY NONE`，速度和加速度须匹配模型包，`WDT_MS=100`。程序回读 `CALIB?` 和 `CONFIG?`，检查校准存储状态和轨道长度；不发送 `CONFIG`、`RESET`、`ENABLE` 或自动回中命令。预检不符时报告失败，不尝试替用户修正台位。

### 控制行为

- 状态协议为 `STATE flags x theta vx vtheta`。位 0 表示小车有效，位 1 表示摆杆有效；无效占位数值不送推理。
- 四维观测为 `[x m, theta rad, vx m/s, angular_velocity rad/s]`；摆角及角速度方向映射由模型包固定。动作先限幅为 ±1，再按模型尺度换算 mm/s，并限制到部署速度上限。
- 根据模型包频率做单线程绝对时间调度。错过的周期跳过，不突发补发历史动作；同一状态快照不重复推理。
- 超过 ±20° 暂停发送速度，回到 ±18° 内恢复。摆杆暂时无效也暂停；小车无效、固件离开 READY、I/O 失败或长时间无状态则退出。短暂积压等待排空，过期推理结果不发送。暂停期间固件看门狗负责减速。
- 轨道边界来自固件校准长度和软件余量；端侧只抑制越界方向动作，保留向内动作。
- Ctrl+C 或控制失败后请求零速并确认 `DISABLED`。观察到 `FAULT` 保留故障，明确报告未确认 DISABLED，不自动 RESET。无法确认停机返回非零退出码。

### 维护命令

无子命令进入交互控制台。`query`、`state`、`config`（不带参数）为读取；`config NAME VALUE`、`enable`、`reset`、标定命令需要操作者明确调用。`center`、`vel <mm/s> --hold <秒>`、`wdt-test` 会实际运动，不属于策略启动步骤。

`state --secs` 报告接收行率及采集到的有效快照范围，不声称覆盖每个设备样本的极值；无效摆杆通道显示 unavailable，不将占位 0 当作真实角度。`vel` 是定时保持工具，不与 Python 固件工具的按轨道行程模式等同。

## 无硬件验证

```sh
make -C deploy/v2/edge test-unit
make -C deploy/v2/edge test-integration PYTHON=/path/to/python
```

`test` 默认只运行单元测试。设备注入单元测试使用受能力接口约束的替身，不链接 UART/协议实现，覆盖实例与时钟隔离、逐事件门控和配置瞬态；串口适配器测试验证业务事件、通信统计和故障历史。集成测试 Python 环境需要安装 `onnx`（模型包构建测试）。集成测试在 Linux/WSL 使用 PTY 和 socketpair，不打开真实串口，不需要 NPU。它以真实 CLI、协议、会话和 UART 实现配合测试固件及 NPU 替身，覆盖 MODE 回执、STATE 有效位、参数预检、启动写入边界及退出确认。

测试程序明确命名 `.test-build/edge_TEST_ONLY`，启动推理时输出测试标识。生产 CMake 没有测试替身选项；CPU-only 无 SDK 依赖，开启 NPU 时缺 SDK 头文件或链接库会构建失败；可使用厂商交叉链接 stub，板端使用匹配的真实运行库。测试通过不能代替真实 ARM 链接、目标 SDK 加载、模型精度、400 Hz 板端时序或真机验证。

## 模块边界

| 模块 | 职责 |
|---|---|
| `main.c` | CLI、串口与业务实例装配、模型选择、部署预检、推理回调与退出 |
| `control.c` | 控制门控、速度换算与周期推进 |
| `package.c` / `prepare_package.py` | 模型包读取与从正式模型目录派生 |
| `cartpole_format.c` / `cartpole_format.h` | 枚举展示转换（`*_to_string`），与设备事务分离 |
| `cartpole.c` / `cartpole.h` | 基本设备会话生命周期、查询、类型化事件与设备事务 |
| `cartpole_device.h` | 业务定义的设备能力，由实际设备适配器实现 |
| `serial_cartpole.c` / `serial_cartpole.h` | 串口实例、固件协议事务、接收缓存与通信统计 |
| `cartpole_schedule.c` / `cartpole_control.h` | 上层有界业务调度与控制操作接口 |
| `cartpole_clock.c` / `cartpole_clock.h` | 可共享的纯单调时钟与绝对等待服务 |
| `cartpole_controller.c` | 正式控制调度、推理前后门控与动作提交 |
| `cartpole_motion.c` | 定时速度保持、回中等完整维护操作 |
| `console.c` | 维护命令解析、交互输入和人类可读显示 |
| `proto.c` / `uart.c` | 严格行解析、串口读写与有限写等待 |
| `policy.c` / `cpu_policy.c` / `policy_npu.c` | 显式后端选择、FP32 CPU 与 NPU 生命周期及原始动作推理 |

真实板端验收仍需匹配工具链、当前模型 OM、实际 UART 节点、模型数值对照与闭环时序证据。

清单解析使用固定版本 yyjson 静态编入，无板端 JSON 动态库依赖。清单上限 8 KiB，拒绝重复/未知字段、类型错误及不完整包；字段顺序无关。旧自定义文本清单不保留兼容。

## 倒立摆业务接口

`cartpole.h` 只描述倒立摆的测量、设备状态、配置与基本设备操作。端口和波特率属于 `serial_cartpole` 适配器的构造参数；控制器和维护业务不依赖 UART 或固件协议。

装配入口初始化 `cartpole_clock_t`，用它打开串口适配器，将
`serial_cartpole_get_device()` 返回的抽象设备能力注入 `cartpole_create()`。
生命周期函数声明在 `cartpole.h`；会话借用设备及其时钟，不负责选择或构造串口。
会话计算截止时间和适配器判断超时使用同一个单调时钟。
释放顺序是有界停止确认、`cartpole_destroy()`、`serial_cartpole_close()`。
一个设备只能绑定一个会话，但多个设备/会话可以共享同一时钟。

`cartpole_device_ops_t` 是基本设备层定义的实现契约，由串口适配器或测试替身实现。
上层控制、推理和维护操作的入口在 `cartpole_control.h`，实现仅通过 `cartpole.h`
的公开基本能力使用设备，不访问内部结构或适配器函数表。

时钟仅提供单调读时和绝对等待，不包含取消、回调或独占状态。
这些可变状态由每个 `cartpole_t` 私有持有。
外部取消经 `cartpole_set_cancellation_source()` 注入查询回调；信号处理器只修改
应用自己的信号标志，不写设备模块内部字段。适配器绑定对应会话的取消查询，
销毁时同步解绑；一个会话取消不会污染共享时钟上的另一个会话。

`cartpole_execute_operation()` 给上层操作提供同步独占作用域及逐事件观察，
返回时自动解绑临时观察器并解除独占，操作完成不等于停车。
`cartpole_evaluate_measurement()` 约束应用计算回调只能读取状态或请求取消，
违规不会被回调忽略返回值而掩盖。控制时序和门控仍属于上层控制模块。

查询按职责分开：

- **最新测量**：位置、摆角、两路速度、各自有效性与主机接收时间。新测量标识用于区分后续收到的数据，不是设备采样序号。
- **会话状态**：连接是否打开、是否已收到设备模式、最新模式及原因、会话失败及历史故障。打开连接不证明设备在线，未知模式也不等于已确认禁用。
- **设备配置**：最近已获取的标定状态、轨道长度与部署参数，未完整获取时不能据其开始控制。

`cartpole_get_latest_measurement()`、`cartpole_get_session_state()`、`cartpole_get_configuration()` 分别返回上述三类数据。通信统计由适配器的 `serial_cartpole_get_statistics()` 提供，只供维护工具计算行率和异常行数，不放入业务测量。

这些查询均返回本地已接收值，不隐式访问设备。只有测量报告刷新测量接收时间；模式和配置回执不会使旧测量重新变新。无效通道不能参与推理或停车判定。时间来自主机单调时钟，不是编码器采样或固件执行时刻。

### 单线程执行与事件

`cartpole_run_controller()` 管理正式控制的绝对周期、有限接收、新测量去重及候选动作提交。应用提供模型推理回调和部署参数，不自行安排串口接收和发送。推理后先处理新到达事件，再核验模式、有效性、控制门状态和候选年龄；过期动作不补发，错过的周期不追赶。

测量和模式事件逐条交付，配置事件仅在部署配置值发生变化时交付；配置查询尚未完整时事件里的 `complete` 为假。事件同步交付，不能将同一批数据里的无效→有效或故障→就绪折叠为最后状态。内部控制门与外部业务订阅彼此独立，订阅用于观测，不替代故障锁存。没有额外接收线程，也没有无限事件队列；推理阻塞期间串口数据仍只能在回到事件循环后处理，固件看门狗仍是独立保障。

事件和推理回调只读取/复制数据或请求取消，不递归进入事务、循环、连接变更或停车。`cartpole_run_loop()` 的维护回调返回观察、速度目标或结束意图，执行命令由会话层在正确阶段处理；回调返回结束或循环到期本身不等于电机停稳。`cartpole_hold_velocity()`、`cartpole_move_to_center()` 封装各自完整运动过程，最终退出仍由应用统一调用停止确认。退出回调后再执行有界停止确认。

### 事务与停车

`cartpole_enable()`、`cartpole_reset()`、`cartpole_mark_endpoint()`、`cartpole_configure()` 是完整业务事务。调用者不拼接底层发送、清积压和等回执；一次绝对期限覆盖全部阶段。返回结果区分取消、超时、传输失败、协议异常、设备拒绝、故障、回执值不符及非法参数，错误从发生处传递，不根据最终状态猜测。控制过程还区分测量不可用、会话不可用、控制期限、部署配置改变和应用回调错误，不把这些原因冒充成设备拒绝。

超时只表示未在期限内确认，命令可能已经送达甚至执行，不能自动重试使能、标定或配置。写入成功也不等于设备执行；完整写入但已过期与未写入不是同一结果。

连接关闭不等于停车。运动退出先完成有界零速、停稳与禁用确认，再关闭连接；故障保持故障，不伪装成禁用。停止确认必须使用新收到的证据，不能复用进入停车流程前的零速或模式。取消和既有流错误不妨碍必要的清理尝试。

每次打开适配器创建新的缓存、统计和故障锁存；关闭后该实例不可再访问。销毁业务会话解除它的事件观察器，但不关闭借用的适配器。调用方保留的旧值只是一份旧副本，不代表当前设备状态。

公开命名区分职责：缓存查询用 `get_*`，年龄计算用 `cartpole_calculate_measurement_age_s()`，取消谓词用 `cartpole_is_stop_requested()`。单调时钟查询为 `cartpole_get_monotonic_time_s()`；展示层显式包含 `cartpole_format.h` 使用枚举的 `*_to_string()`。串口配置词汇由 `serial_cartpole_setting_to_string()` / `serial_cartpole_parse_setting()` 转换，不属于业务会话接口。

### 文本结果的只读解析

协议层的 `proto_parse_command()` 校验完整 LF 结尾命令并提取命令类型、参数和值，
失败不修改输出，不发送命令。它与设备业务接口分离，可用于线路记录分析。
`cli_report.py` 的 `parse_state_window_summary()` 校验四行 STATE 摘要，
`parse_state_output()` 校验包含已定义 MODE/STEP_LOSS 事件的完整输出。
二者返回类型化结果，不把任意未知文本当作可忽略日志。
制包单位错误通过 `parse_firmware_unit_error()` 提取错误代码与字段。
其余面向人的诊断说明并非统一机器结果协议，不应依赖整段措辞判断运行成功。
