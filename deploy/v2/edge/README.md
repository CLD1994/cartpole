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

`test` 默认只运行单元测试。集成测试 Python 环境需要安装 `onnx`（模型包构建测试）。集成测试在 Linux/WSL 使用 PTY 和 socketpair，不打开真实串口，不需要 NPU。它以真实 CLI、协议、会话和 UART 实现配合测试固件及 NPU 替身，覆盖 MODE 回执、STATE 有效位、参数预检、启动写入边界及退出确认。

测试程序明确命名 `.test-build/edge_TEST_ONLY`，启动推理时输出测试标识。生产 CMake 没有测试替身选项；CPU-only 无 SDK 依赖，开启 NPU 时缺 SDK 头文件或链接库会构建失败；可使用厂商交叉链接 stub，板端使用匹配的真实运行库。测试通过不能代替真实 ARM 链接、目标 SDK 加载、模型精度、400 Hz 板端时序或真机验证。

## 模块边界

| 模块 | 职责 |
|---|---|
| `main.c` | CLI、预检、控制循环及退出 |
| `control.c` | 控制门控、速度换算与周期推进 |
| `package.c` / `prepare_package.py` | 模型包读取与从正式模型目录派生 |
| `cartpole.c` / `cartpole.h` | 单设备会话、完整命令事务、状态快照、逐帧事件及停止确认 |
| `console.c` | 维护命令、回中流程和人类可读显示 |
| `proto.c` / `uart.c` | 严格行解析、串口读写与有限写等待 |
| `policy.c` / `cpu_policy.c` / `policy_npu.c` | 显式后端选择、FP32 CPU 与 NPU 生命周期及原始动作推理 |

真实板端验收仍需匹配工具链、当前模型 OM、实际 UART 节点、模型数值对照与闭环时序证据。

清单解析使用固定版本 yyjson 静态编入，无板端 JSON 动态库依赖。清单上限 8 KiB，拒绝重复/未知字段、类型错误及不完整包；字段顺序无关。旧自定义文本清单不保留兼容。

## 会话接口的使用边界

`cartpole.h` 是设备会话边界，不是模型或完整控制器。会话仅支持一个串口、一个调用线程；调用方主动推进有界接收，没有后台接收线程。信号处理器只请求停止，实际停车与确认在正常退出路径完成。

- 遥测数值、有效位、模式及接收时刻属于同一快照。读取快照不会访问串口；没有推进接收就没有新数据。无效通道的零占位不能作为真实测量。
- 时间来自主机单调时钟，表示本进程接收/解析时刻，不是编码器采样时刻；序号也是本会话接收计数，不是设备序号。未知模式不能当作已确认 DISABLED。
- 逐帧事件用于观察同一批接收里的中间状态，不能用最后一帧覆盖掉无效、FAULT 或模式转换。事件处理不得重入接收或发送命令。
- 缓存读取与设备查询分开。完整事务负责发送、回执匹配及超时，调用者不自行拼接底层发送和等待。
- 正常维护发送与控制周期的单次非阻塞发送分开；“完整写入后已过期限”不能等同于“没有发送”。写入成功只证明内核接受，不证明固件已经执行。
- 关闭串口不等于停机。运动会话先执行有界停止确认，再关闭连接；FAULT 保留故障，不伪装成 DISABLED。控制门控和模型推理仍由 `control.c`、`main.c`、`policy.c` 负责。

典型调用顺序是 `cartpole_open` → `cartpole_sync_mode` / `cartpole_refresh_metadata` → `cartpole_wait_state` → 周期 `cartpole_pump`，然后一次 `cartpole_snapshot()` 复制当前状态供该轮使用。推理后的再次接收必须读取新的快照，不能把推理前的快照当作最新值。`cartpole_cached_value` 仅查询已收到的参数；`cartpole_enable`、`cartpole_reset`、`cartpole_mark`、`cartpole_configure` 各自完成一次请求事务，不需要调用者处理裸协议回执。运动退出使用 `cartpole_stop_and_confirm`，最后 `cartpole_close`。

完整事务用具名结果区分取消、超时、传输失败、协议异常、固件拒绝、FAULT、回执值不符及非法参数。错误在对应阶段产生并直接传递，上层不根据最后一份状态反推原因。CLI 保留操作名称，不把全部失败写成“超时”。超时只说明未在期限内确认，命令可能已经送达甚至执行；不会因此自动重发使能、标定或配置事务。传输失败也不证明设备没有执行，不应据此盲目重试。

快照字段直接说明其来源和生命周期：

| 字段 | 含义与边界 |
|---|---|
| `is_serial_open` | 本进程打开了串口；不代表设备在线或通信健康 |
| `is_mode_received` | 本次会话已解析过 MODE；不是 READY 判断 |
| `reported_mode` / `reported_mode_reason` | 最后收到的模式与原因；必须结合 `is_mode_received` |
| `measurement_validity_bits` | 小车/摆杆有效位，使用 `CARTPOLE_MEASUREMENT_CART_VALID` / `CARTPOLE_MEASUREMENT_POLE_VALID`，不直接写数字掩码 |
| `last_state_received_host_monotonic_s` | 最近 STATE 在本机被解析的单调时刻；未收到为 -1，MODE/OK 不刷新它 |
| `received_state_count` | 本会话已解析 STATE 的累计数，包括通道无效但格式正确的帧；不是设备采样序号 |
| `has_received_fault_mode` | 本次会话曾收到 MODE FAULT；后来的 READY 或 RESET 不清除该记录 |
| `has_stream_failure_latch` | 会话失败锁存，禁止继续非零驱动；仍允许零速与停止确认的清理路径，不仅表示串口 IO 错误 |
| `received_unknown_line_count` | 无法识别的已接收行累计数，不是测量无效帧数 |

open 建立新会话，close 清除快照与缓存；关闭后保存于调用方的旧快照只是旧值，不代表当前设备状态。`cartpole_state_age_s` 计算 STATE 接收年龄，`cartpole_host_monotonic_s` 提供同一主机时钟。
