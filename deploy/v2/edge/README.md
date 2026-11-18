# CV610 端侧推理与控制

本目录在 Hi3516CV610 上加载 `.om` 策略，读取 ESP32 的串口状态并发送速度目标。训练和模型事实来自正式 ONNX 目录的 `model.json`，端侧模型包由工具生成。程序不自动修改固件配置、使能或回中。

## 构建

生产构建需要 ARM32 musl 交叉编译器、配套 SVP ACL SDK 和本目录的真实 NPU 静态库。交接包放在 WSL 内部 ext4 文件系统，不能把工具链长期放在 NTFS。

```sh
bash deploy/v2/edge/setup-wsl.sh /mnt/c/<交接包目录>
bash deploy/v2/edge/build.sh
```

默认依赖目录为 `~/cv610`，可用 `CROSS_ROOT` 指定。产物是 `deploy/v2/edge/build-arm/cartpole_edge`。构建脚本检查 ARM32、musl 解释器、SDK 动态依赖和 RPATH；这些检查不代表真实 NPU 或控制性能已验收。导入时必须检查完整性报告，不能忽略工具链或 SDK 文件校验失败。

模型转换工具 ATC 与交叉编译 SDK 是不同依赖。须使用与板端匹配的 Hi3516CV610 工具包，将当前 ONNX 转换为 OM；ONNX 的输入是 FP32 `[1,4]` 状态而不是图像，不能直接套用图像预处理示例。转换产物须通过同输入的 ONNX/OM 数值回放，再在板端测量推理耗时。参见 [NPU 接口](npu/README.md)。

## 准备模型包

已有配套工具转换生成的 OM 后，在安装了 `onnx` 的 Python 环境执行：

```sh
python deploy/v2/edge/prepare_package.py \
  --model /path/to/exported \
  --om /path/to/policy.om \
  --output /path/to/new-edge-package
```

输入目录须包含 `model.json`、`policy.onnx` 和其外部权重。工具检查来源散列和输入输出形状，派生频率、动作尺度和运行限值，复制 OM，生成 `edge-package.txt`。它不能证明提交的 OM 确实由该 ONNX 转换；模型转换和数值对照仍是部署验收的一部分。包内 SHA256 是来源记录，C 运行器不进行运行时散列校验；生成后应整体部署，不单独替换文件。

## 板端使用

板端需要匹配的 NPU 驱动与 SDK 动态库。复制生产程序和整个模型包目录；不要部署 `.test-build/` 中任何文件。

```sh
./cartpole_edge --port /dev/ttyXXXX query
./cartpole_edge --port /dev/ttyXXXX state --secs 1
./cartpole_edge --port /dev/ttyXXXX enable
./cartpole_edge --port /dev/ttyXXXX run --package /path/to/new-edge-package/edge-package.txt
```

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

测试程序明确命名 `.test-build/edge_TEST_ONLY`，启动推理时输出测试标识。生产 CMake 没有测试替身选项，缺 SDK 头文件或链接库时构建失败；可使用厂商交叉链接 stub，板端使用匹配的真实运行库。测试通过不能代替真实 ARM 链接、目标 SDK 加载、模型精度、400 Hz 板端时序或真机验证。

## 模块边界

| 模块 | 职责 |
|---|---|
| `main.c` | CLI、预检、控制循环及退出 |
| `control.c` | 控制门控、速度换算与周期推进 |
| `package.c` / `prepare_package.py` | 模型包读取与从正式模型目录派生 |
| `rig.c` | 协议会话、回执、状态、参数及停止确认 |
| `proto.c` / `uart.c` | 严格行解析、串口读写与有限写等待 |
| `policy.c` / `npu/` | NPU 生命周期与原始动作推理 |

真实板端验收仍需匹配工具链、当前模型 OM、实际 UART 节点、模型数值对照与闭环时序证据。
