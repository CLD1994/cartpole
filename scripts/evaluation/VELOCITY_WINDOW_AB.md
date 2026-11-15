# 冻结策略的速度观测 A/B（仅仿真）

复用 evaluate_policy.py，不训练、不改奖励/执行器/物理步长、不接设备。

- A: `--velocity-observation-window-ms 0`，保留原始策略观测。
- B: `--velocity-observation-window-ms 16`，仅将策略两路速度改为原始连续关节位置16ms首末差/16ms，保留既有速度噪声；位置输入不变。
- 物理仍1ms、策略仍2ms。9个控制采样端点的理想16ms窗近似固件17个1ms（或33个0.5ms）点的端点估速；不复制量化、采样抖动、失效或通信延迟。使用未包角关节位置，避免±π显示转换引入假跳变。
- 每次reset清历史，前16ms保留瞬时速度，不凭空制造历史或改变初始物理状态。完成槽位自动reset也清历史，不跨回合做差。
- 同模型、同seed、同num_envs、同其他参数运行两组。报告含初态及随机执行器参数，应先核对匹配；轨迹及终止分歧可能改变后续随机数消费，因此不声称后续逐点噪声严格配对。
- 指标仍读取无噪声物理状态。新增 action_delta_rms（相邻归一化动作差的RMS）及 pole_rms_velocity_rad_s；结合原车速RMS、角度RMS、峰值与失败判断。它们不直接定位14Hz频率，不能凭成功率不变就说抖动无变化。

PowerShell（`$checkpoint`设为实际已选的model_1050.pt完整路径；不使用ONNX路径）：

```powershell
$ab = ".\logs\velocity-window-ab\$(Get-Date -Format yyyyMMdd-HHmmss)"
python .\scripts\evaluation\evaluate_policy.py --checkpoint "$checkpoint" --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --velocity-observation-window-ms 0 --output-dir "$ab\instant"
if ($LASTEXITCODE -ne 0) { throw 'A evaluation failed' }
python .\scripts\evaluation\evaluate_policy.py --checkpoint "$checkpoint" --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --velocity-observation-window-ms 16 --output-dir "$ab\window16"
if ($LASTEXITCODE -ne 0) { throw 'B evaluation failed' }
Get-ChildItem $ab -Recurse -Filter report.json | Select-Object -ExpandProperty FullName
```

不需要Kit进行统计评估；可另用Kit观察，但不要把评估耗时当性能基准。64回合/组是初步敏感性比较，不是真机验收，也不是14Hz抖动已证实由估速窗导致。当前仅基本静态/局部数值核查，Isaac Lab现场待运行。

## 已有真机指令的离线 PhysX 重放

不需要checkpoint，不运行策略、不连接串口。将同一5秒真实VEL序列作为外部输入，A固定tau20ms，B仅tau=0；两组均固定动作比例1.25m/s、加速度10m/s²、指令延迟6ms、名义资产与原力伺服。**20ms是当前代码名义值，不是已证明1050训练的唯一参数**；训练本来随机tau10–40ms。

```powershell
$trace = Get-ChildItem . -Recurse -File -Filter 'trial1050-gap6-20261115-104841.json' | Select-Object -First 1 -ExpandProperty FullName
if (-not $trace) { throw 'Please set $trace to the saved five-second motion report path' }
$replay = ".\logs\actuator-replay\$(Get-Date -Format yyyyMMdd-HHmmss)"
python .\scripts\evaluation\replay_device_actions.py --trace "$trace" --tau-s 0.020 --delay-ms 6 --viz none --device cuda:0 --output-dir "$replay\tau20"
python .\scripts\evaluation\replay_device_actions.py --trace "$trace" --tau-s 0 --delay-ms 6 --viz none --device cuda:0 --output-dir "$replay\tau0"
Get-ChildItem $replay -Recurse -Filter report.json | ForEach-Object { Get-Content $_.FullName -Raw -Encoding utf8 }
```

报告含逐拍物理状态、16ms窗速度、起始/中段/末段真实估速残差；终止则保留已完成片段并退出码2，绝不自动重置后继续拼接。两组初始实际状态应匹配，保存env.yaml用于核对。基于主机写完成时刻，命令在下一2ms控制边界生效，最多增加2ms量化；输入估速自身含16ms滤波，初始真实瞬时速度未知。host时间不是设备执行/采样时间，故残差含对时误差。

固件vel_ramp是逐步限加速度追踪且换向经过零速，不是tau一阶滤波。B只去掉仿真额外一阶参考滤波，仍有力伺服/摆杆耦合，不宣称复制ISR梯形规划，也不能用这次重放证明固件正常或闭环安全。当前未在开发端运行PhysX，供现场仿真联调。


### 重放初始化修正（replay_revision 2）

首轮两个报告请求初始摆角速度−0.169rad/s，reset实际为0，因此不能据这两份重放给tau排序。Isaac Lab的reset事件会用soft_joint_vel_limits夹速度；新版本记录该值与reset结果，随后仅在仿真中通过公开接口显式写入记录的初态，立即核对，失败则拒绝继续。没有改变训练任务、真实设备或终止条件。

即使初始化一致，开环倒立摆对模型/初态误差仍敏感，可能提前倒杆；这不是必须“修到完整5秒”的程序故障。报告保留终止，比较时只能用两组共同覆盖的时间段，不能把不同长度的startup RMSE直接排名，也不能把终止后的自动reset状态拼入结果。
