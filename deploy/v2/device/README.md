# 设备会话与单动作闭环运行器

本目录提供 Python/C 设备会话接口，以及 Python 单动作 ONNX 真机运行器。当前运行器版本为 21；它不自动 ENABLE、不归零、不改标定或固件配置。双动作参考入口 `../pc_runner.py` 与 `../edge/main.c` 不适用于此处的单动作模型。

## 准备模型与启动

在仓库根目录运行。先用与训练任务一致的导出流程取得 ONNX，再生成模型清单：

```powershell
& $python scripts/rsl_rl/prepare_model.py --onnx $onnx --checkpoint $checkpoint
$model = Split-Path $onnx
```

导出目录必须包含 `policy.onnx`、所需外部权重与 `model.json`。准备工具读取训练目录 `params/env.yaml`，记录训练配置及文件散列；缺失或不支持的配置明确报错，不根据文件名猜参数。无需手工维护 contract/profile 两套 JSON。

只读核对（需要 `pyserial`）：

```powershell
& $python deploy/v2/device/check.py --port COM11 --model $model --seconds 2
```

只发送查询，不发送运动或配置写入。`preflight_reason=null` 表示本次接收值满足接口条件，不是闭环性能验证。默认终端显示摘要，`--json` 输出机器可读摘要。

现场完成标定、确认参考并使能到 READY 后，当前短延迟容错实验命令为：

```powershell
& $python deploy/v2/device/run_policy.py --port COM11 --model $model `
  --continuous --start-confirmed --command-gap-hard-ms 25 `
  --retry-slow-inference --thread-priority above-normal --gap-timeline
```

**25 ms 是显式实验选项，不是默认值或实时保证。** `--seconds 5` 可替换持续运行；Ctrl+C 请求结束。默认报告写入模型目录的 `runs`，也可用 `--output` 指定不存在的新文件。运行期间不得替换模型文件或让其他程序占用同一串口。

`--diagnostic-timing --seconds 5` 为 DISABLED 下的同路径只读诊断，不需要 `--start-confirmed`，不会真实发送 VEL/DISABLE，不作为运动或停机验证。

## 模型与设备参数

模型观察顺序为 `[x, theta, xdot, thetadot]`，动作是一个归一化目标速度。频率、动作倍率和训练加速度来自 `model.json`；台位波特率、看门狗、边距和方向约定在 `experiment_setup.py`。运行器读取当前标定长度与摆角零位作为会话快照，运行中变化会停止。

默认要求固件速度、加速度与模型匹配。Python 运行器可显式指定：

- `--velocity-limit-mm-s 900`：保留训练动作倍率，计算目标速度后饱和至部署上限，不重新缩放整个动作区间。上限不得超过模型倍率。
- `--acceleration-limit-mm-s2 10000`：选择部署加速度，仍记录原训练值；不修改模型或固件。

预检及运行中读回必须与选定参数精确匹配。部署限幅不证明模型在该配置下训练过或已经稳定。其他检查脚本和 C 接口不自动继承这些覆盖选项。

标定须为 `CAL_STATE=2`、`CAL_SAVED=1`、`CAL_STORAGE=0`，且长度在支持范围内。`CAL_STATE=3` 表示保存记录存在但本次参考未确认，应通过固件工具在实物位置执行 `confirm-left`，不能用坐标数字代替实际定位。

## 状态协议与有效性

```text
STATE flags x_um theta_urad vx_mm_s vth_mrad_s
```

六个 token 使用单空格、规范十进制整数。`flags` 为 0..3：bit0 小车有效，bit1 摆杆有效。四个量为 int32，摆角范围 ±3141592 µrad；无效路的两个值必须为 0，消费者解释为无效而不是真实零。旧四值 STATE、独立 VEL_VALID 与旧紧凑时间帧不兼容。

固件通常以 500 Hz 上报。每行含 LF 为 16..53 字节，500 Hz 最坏约 26500 B/s；460800 波特、8N1 下占约 57.5% 线速。这是吞吐预算，不保证延迟。

帧不包含启动标识、采样时间、年龄或序号。重复数值可能是静止，不能据此判旧样本。所有主机接收年龄和排空边界仅描述主机观测，不证明设备采样新鲜度，也不能可靠识别所有设备重启。

短暂摆杆读帧失败由配套固件在完整成功窗口、健康状态与设备侧限龄内保留有效估计。主机收到双路有效 STATE 后正常推理，即使数值重复也不断发 VEL。主机不自行把无效占位零或缓存旧值伪装成有效数据。

正式闭环遇到小车有效、摆杆无效的 `STATE 1` 时等待恢复，不立即以 `measurement_invalid` 退出；期间停发 VEL、继续收帧和刷新元数据。新鲜双路有效帧通过接收策略切换及排空确认后重新推理。固件 FAULT、小车无效、坏帧或约 100 ms 无 STATE 仍结束会话。等待超过固件 WDT 时会触发其减速策略，主机不自动 ENABLE。只读诊断和通用 Session 不启用此特定摆杆恢复策略。

## 调度、时限与短延迟恢复

400 Hz 模型使用 2.5 ms 绝对时间栅格选择新接收观测；错过栅格跳过，不补发突发命令。推理使用 CPU 单线程，不能消除操作系统调度、驱动或其他负载造成的暂停。

| 配置 | 命令间隔硬限 | 候选观测年龄 | 推理判据 |
| --- | --- | --- | --- |
| 默认 normal | 6 ms | 4 ms | 决策阶段超过 2 ms 判慢 |
| debug | 15 ms | 12 ms | 不施加 normal 的 2 ms 预算 |
| 当前 normal 实验命令 | 显式 25 ms | 仍为 4 ms | 配合 `--retry-slow-inference` 丢弃慢候选后恢复 |

`--command-gap-hard-ms` 只覆盖命令硬限，不改变固件 WDT 或观测期限。命令间隔不是累加或滑窗，是距上一条完整写入返回的时间。写入返回不代表固件已收讫。

当前 Python 运行器使用唯一接收线程。它按原顺序保存完整帧和接收时间；控制线程独占 Session 和写入。无效帧、FAULT、回执不得被最新有效帧覆盖。线程间 FIFO 容量为 64 帧，底层缓冲上限 1024 字节，单行上限 256 字节。

活动期短暂慢读、慢推理和超过 128 字节的积压统一进入恢复：

1. 记录警告，丢弃旧候选，不重复发送旧动作。
2. 按原顺序消费，等待队列、半帧和在途读取排空。
3. 记录排空边界，等待之后收到的有效帧，重新推理。
4. 恢复等待不延长原活动命令期限；达到选定硬限仍结束。

线程接收器读窗口超过 10 ms、轮询间隔超过 20 ms 但低于 100 ms 时记录慢窗口并恢复，而非立刻丢字节、失步或退出。100 ms 级持续延迟、坏字节、读契约错误、容量溢出与半帧超时仍是错误。活动期半帧期限为 20 ms。通用 `Receiver` 默认仍严格执行 10/20 ms；C 接口也不自动获得 Python 线程运行器的恢复语义。

写前保留 **1 ms** 经验预算。候选放不进剩余期限就丢弃，不能借新帧时间戳延长旧动作寿命。写入完成后才发现候选超过观测年龄时，命令无法撤回：记录 `write_budget.completion_age_warnings` 并重新排空、取新帧推理。命令硬期限仍独立生效。经验预算不保证后续写入一定按时完成。

`--thread-priority above-normal` 仅在本次运行提高控制和接收线程的 Windows 优先级，退出恢复；不设置实时优先级、不改变进程优先级，也不保证硬实时。

## 摆角暂停与恢复

默认 `abs(theta) > 20°` 暂停 VEL；新鲜有效观测回到 `abs(theta) <= 18°` 才重新推理恢复。可用 `--angle-pause-deg`、`--angle-resume-deg` 显式设置。当前 V2 训练在绝对摆角严格大于 20° 时终止；部署同样在严格超过 20° 时暂停、回到 18°（含）以内恢复。历史模型及其清单不会因训练默认值更新而自动改变，部署门控也不是从旧模型边界自动推导。

暂停不主动 DISABLE、不退出流程、不自动 ENABLE。只读诊断不启用摆角暂停。停发后固件在 WDT 到期才按配置减速，并非即时停止。

暂停期间旧观测仅等待，不能用旧低角度恢复；仍检查通信、元数据、坏帧、小车有效性与 FAULT。接收暂停策略使用 100 ms 读/轮询/半帧期限。恢复需排空并完成活动接收策略交接，使用交接后的新鲜双路有效观测。重新建立栅格及命令预算，不补发历史动作。暂停前已经超期不能借大摆角掩盖。

## 停止与故障

通用 Session 要求启动时近期 READY/NONE、有效标定、配置及双路测量；显式启动不发送 ENABLE。运行器定期刷新元数据，拒绝不一致的配置或模式变化。

停止流程请求 VEL 0，在可用停稳证据后请求 DISABLE，并等待其后的 `MODE DISABLED NONE`；BUSY 可在有界期限内重试。**只有 `disabled_confirmed=true` 才代表收到禁用确认。** FAULT 保留原故障时可能无法得到 DISABLED，完整写入 VEL 0 或进程退出均不能替代禁用确认。故障退出后不自动恢复控制。

位置保护由固件负责；ONNX 运行器不额外实施通用 Session 的位置方向门。归一化动作取整采用半单位远离零。持续运行需要现场监控，软件报告不能证明机械停稳、无打滑或真实加速度。

## 可选事件与报告

`--step-loss-monitor` 查询 RUN 身份，只记录匹配运行的 STEP_LOSS 及有界前后轨迹，不切换低速、不因事件本身停车。是否继续由固件模式与其他条件决定，FAULT 仍停止。需要固件明确支持事件协议；没有事件不能证明没有丢步。固件时钟与主机时钟不能直接相减。

控制结束后可使用配套固件 `tools/step_trace_capture.py` 读取诊断轨迹，勿与运行器同时占串口。轨迹版本必须按其协议解释，不能据软件 STEP 计数证明实际脉冲或机械位移。

重要报告字段：

- `error`、`session_failure`、`receive_error`：应用、会话与接收层原因。
- `first_invalid_state`：首个无效帧，不一定是最终退出原因。
- `pole_recovery`、`angle_pause`：有意等待和恢复。
- `threaded_receiver.slow_windows`、`backlog_recovery`：慢窗口与排空恢复证据。
- `slow_inference`、`write_budget`：被丢候选及写后年龄违约。
- `failure_snapshot.actual_command_gap_s`：快照时距最近一次完整写入的间隔；写后失败可能已更新起点，需结合样本 `active_command_gap_s` 判断。
- `run_totals`：已完成指令之间的统计；最大成功间隔不包含末尾未再完成写入的超期空档。

`--gap-timeline` 保存最近 4096 个主机阶段边界，退出后写报告，不在热循环打印。墙钟与线程 CPU 时间仅帮助定位调用区间，不能独立区分系统抢占、GIL、驱动等待或真实计算。线程模式报告中的控制时间线与接收线程统计不同；不能把控制线程时间当接收调用内部时间。

跨摆角或摆杆有意暂停的首条命令分别标记 `intentional_angle_pause_gap`、`intentional_pole_recovery_gap`。`active_command_gap_s` 从恢复后建立的活动预算起点计算，硬超限计数使用它，恢复首写也不能豁免硬限；其他软间隔计数排除有意暂停标记，原始命令间隔仍保留。报告许多明细为有界尾部，不是完整轨迹。

## 验证

```sh
python -m unittest discover -s deploy/v2/device -p 'test_*.py'
python -m unittest discover -s deploy/v2/device/integration_tests -p 'test_*.py'
make -C deploy/v2/device/edge test
```

Python 单元/组合测试使用注入时钟与模拟传输验证逻辑；C 测试验证其接口。它们不替代目标 Windows、实际 ONNX、USB 串口和配套固件的现场验证。目标设备时序及控制稳定性须另行验证，不能由测试通过或只读诊断退出码推导。
