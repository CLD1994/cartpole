# 当前设备的策略部署与会话接口

本目录提供只读检查、ONNX 影子推理、真实策略控制及禁用设备上的时序诊断。真实入口是 `run_policy.py`；旧 `../pc_runner.py`、`../edge/main.c` 是独立入口，不能直接替代当前 500 Hz 模型的部署路径。

## 入口与准备

- `check.py`：查询设备状态，不进行模型推理。
- `shadow.py`：只读观测与模型推理，不发送运动或禁用命令。
- `run_policy.py`：显式启动的限时或受监控持续控制；也支持只读诊断。
- `session.py`、`receiver.py`：状态机与有界增量接收。
- `edge/session.h`：独立 C 会话实现，不是 ONNX 策略控制器。
- `diagnose_timing.py`：专用调度对照工具；当前完整 runner 路径的诊断优先使用 `run_policy.py --diagnostic-timing`。

依赖 `pyserial`、`onnxruntime`、`numpy`、`onnx`，无需启动 Isaac Sim。ONNX 推理使用 CPU 单线程；模型须为四维 float32 输入、一个有限动作输出。保留完整导出目录及外部权重文件。不要在控制期间更换文件。

独占串口，关闭其他串口工具。入口在打开端口前设置 DTR/RTS 为低，但 USB/驱动仍可能导致设备复位。主机程序不会自动使能、回中、回零、配置设备、重连或重启控制。

## 模型合同与设备预检

`model-contract.example.json` 是待填写模板，不能直接启动。合同绑定主 ONNX 文件及实际外部权重哈希、观测 `[x, theta, xdot, thetadot]` 的 SI 单位、速度目标动作（幅值 1.25 m/s）、训练控制频率 500 Hz、坐标方向与固件 `POLE_ZERO_URAD / POLE_INV / X_INV`。角度方向不能从网络形状推断；声明和勾选不等于物理事实已验证。合同接受 UTF-8（包括 PowerShell BOM）。

真实控制要求操作者已独立使能，读回 `READY NONE`；有效且保存的标定 `CAL_STATE=2 / CAL_SAVED=1 / CAL_STORAGE=0`；长度与设备档案相符；坐标、速度、加速度、看门狗及软限位参数匹配。运行期间参数/标定变化、过期、无效速度、坏状态帧、FAULT 或通信错误会锁存停止，不因数据恢复重新运动。

`CAL_STATE=3` 只表示有保存记录、本次参考未确认，不能根据当前 x 数字自行认定位置。正确参考必须由操作者完成。只读时保持 `DISABLED`，其 `mode_not_ready` 不意味着需要提前使能。

## 行程保护的归属

当前 runner 按操作者选择，将行程保护交给已有固件：**不设上位机位置门，不做上位机预测提前制动，也不再次钳制越界向外目标**。`BoundSession.enforce_position_limits=False` 仅作用于此 runner；基础 `Session` 默认仍保留位置预检和向外目标钳制，其他客户端不受影响。

固件保持 `X_MARGIN_UM=10000`，不由 runner 修改。853325 µm 标定长度对应软限位约中心 ±416662 µm。固件越界向外目标置零并斜坡减速，不是提前制动或机械防撞保证：1.25 m/s、10 m/s² 下理想制动距离约 78 mm，10 mm 余量无法覆盖，实际还有延迟。不要将取消重复门控理解为已证明机械安全。

主机仍保留标定有效性和本次长度绑定、坐标/配置新鲜度、有限状态及动作、初始低速（|v|≤0.05 m/s、|ω|≤0.5 rad/s）、初始摆角 ±10°、运行摆角 ±20°、通信期限、FAULT 与停机确认。

## 当前调度与超时

- `perf_counter` / `perf_counter_ns` 同源高精度时钟；2 ms 绝对栅格，消费新观测，跳过错过的栅格，不补发旧动作。
- 成功命令完成间隔超过 4 ms 记录软告警，达到 6 ms 为硬停止；观测年龄上限仍为 4 ms，推理预算仍为 2 ms。
- 候选观测已过期时，仅在原上条命令的 6 ms 绝对预算和试验时限内丢弃候选、回到接收流程等待新有效帧重推。不发送旧候选、不续期旧观测、不重置上次命令时间；FAULT、无效状态及其他故障不能借此重试恢复。
- 命令前处理所有可用完整帧；未收齐半帧或未处理积压必须在原预算内完成，否则停机。新接收帧不等于设备新采样，固件没有采样时间戳。
- 长窗口/持续控制启动前验证 `CONFIG? NAME` 单项只读能力，不对旧固件降级为运行中全量查询。QUERY、CALIB?、九项配置每 50 ms 至多一组，单组在途，100 ms 回复期限，元数据 1 s 有效期由实际回执更新时间维护。

这不是硬实时系统，不保证每拍严格 500 Hz；相位恢复可能出现短于 2 ms 的动作间隔。不修改 Windows 实时优先级或系统计时器设置。

## 只读推理

在仓库根目录运行，路径变量指向已核实模型。输出文件必须尚不存在。

```powershell
python .\deploy\v2\device\shadow.py --port COM11 --onnx "$onnx" --seconds 10 --angle-sign negative --output ".\logs\deployment\shadow-$(Get-Date -Format yyyyMMdd-HHmmss).json"
```

示例负角方向仅适用于已独立核对该映射的台位；不确定时不要猜。省略方向参数会分别评估两个方向，不自动选取正确方向。只读白名单不发送 VEL、ENABLE、DISABLE 或配置。

## 真实控制：5 秒与持续模式

现场确认模型、参考、配置以及可独立停机措施；杆近直立且低速，手离开小车运动路径。合同中的 `onsite_stop_ready=true` 必须来自本次真实现场准备，不能自动当作已有保证。由操作者通过固件工具使能后运行：

```powershell
python .\deploy\v2\device\run_policy.py --port COM11 --onnx "$onnx" --contract "$contract" --start-confirmed --seconds 5 --output "$exportDir\trial-$(Get-Date -Format yyyyMMdd-HHmmss).json"
```

不使用任意时限截断的受监控持续模式：

```powershell
python .\deploy\v2\device\run_policy.py --port COM11 --onnx "$onnx" --contract "$contract" --start-confirmed --continuous --output "$exportDir\continuous-$(Get-Date -Format yyyyMMdd-HHmmss).json"
```

`--continuous` 与 `--seconds`、`--diagnostic-timing` 互斥。未指定时长的默认仍为 0.5 秒；有限真实控制最多 5 秒。持续模式仅取消总时限，全部剩余门控保持，不可无人值守。

## 停机与退出

按一次 Ctrl+C 请求正常结束：发送 VEL0，等待新鲜零速观测，再请求 DISABLE，只有后续 `MODE DISABLED NONE` 才确认禁用。READY 状态下的预检拒绝也走安全收尾；发送短写/超时可能已经到达设备，不能据异常宣称“未运动”。

模型、合同、依赖等在打开串口之前失败时，runner 无法替此前已使能的设备禁用。建议调用方使用 PowerShell `try/finally`，复用固件工具追加 `vel 0 --hold 0.2`、`disable`、`query`，最终检查 DISABLED。停机未确认必须现场处置，不自动重连、清故障或再次使能。

`stopped_by_user` 只在 Ctrl+C 后禁用确认且无会话/接收故障时返回。第二次中断、强杀、断电可能中止清理；固件 WDT 仅把目标置零并减速，不自动 DISABLED 或永久锁存。故障后即使禁用成功，控制会话仍是失败。

## 禁用设备上的同路径时序诊断

保持 DISABLED，诊断真实写入出口仅允许精确只读查询；VEL/DISABLE 被虚拟完成，不会使设备运动。

```powershell
python .\deploy\v2\device\run_policy.py --port COM11 --onnx "$onnx" --contract "$contract" --diagnostic-timing --seconds 10 --output "$exportDir\timing-$(Get-Date -Format yyyyMMdd-HHmmss).json"
```

诊断时序/物理门记录 `would_stop` 后继续反事实采集，真实 FAULT、无效状态和通信故障仍退出；过期观测不用于动作。它不包含真实动作写入和机械响应成本，`complete` 仅表示采集完成，不是闭环或期限合格证明。`would_stop` 是门控调用计数，可能重复，不是独立故障次数或失败率。

## 报告与验证边界

报告记录实际命令完成间隔、全部推理尝试（包括丢弃候选）、候选重推、原观测/命令预算、接收阶段、固件读回、停止请求与禁用确认。STATE 间隔是主机解析间隔，不是设备采样或 USB 到达间隔。

持续模式保留最近 10000 个动作/推理/间隔及 512 组回读记录，并维护独立全程计数/最大值；p95 等尾部统计不能当作全程分布。启动前写 manifest，活动期不做同步逐步磁盘写入；退出停机后写最终报告。崩溃可能只留下启动记录，不能据此推断停妥。

基本会话检查可运行 `python -m unittest discover -s deploy/v2/device -p 'test_*.py'`，但这些检查不验证 ONNX 策略、Windows 实时性或机械安全。既有现场有限时长结果也不能代替持续运行及不同工况的验收。
