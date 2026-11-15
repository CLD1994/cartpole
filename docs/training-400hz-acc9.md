# 400 Hz、9 m/s² 从头训练

任务：`Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0`。
该任务继承 Window16-Tau0，旧任务与设备配置文件保持不变。

## 参数契约

- 策略周期 2.5 ms（400 Hz）；PhysX 物理子步 0.5 ms，decimation=5。
- 参考速度斜坡的名义加速度上限为 9 m/s²（9000 mm/s²），每个物理子步最多变化 0.0045 m/s。回合重置随机化为 8–9 m/s²，不能抽到高于 9 的值。实际关节仍由有限推力速度伺服驱动，不保证实际机械加速度恰为该值。
- 执行器一阶滤波 tau=0，重置范围也是零。速度观测保持 16 ms 位置差窗口。
- 16 ms 在 400 Hz 下为 6.4 个控制步。历史端点使用距今 15 ms 和 17.5 ms 的位置按 0.6/0.4 线性插值。它是端点近似，不是额外纯延迟或固件完整采样重放；非匀速时有插值误差。重置后的前 7 步用瞬时速度暖机，重复计算同一控制步的观测不追加历史。500 Hz 的整数窗口不插值。
- 动作尺度仍为 1.25 m/s，奖励、PPO 参数及动作延迟步数保留。120 步 rollout 的物理时长由 240 ms 变成 300 ms；相同 gamma、lambda 的物理时间视野增加 25%。延迟 3–8 步对应 7.5–20 ms（原来 6–16 ms）。终止奖励按步长积分、动作变化项按相邻策略动作计，因此物理时间下的奖励尺度也受频率变化影响。这不是其余物理时间特征完全相同的消融实验。
- 自动保留训练指标候选和并行冻结策略筛选继续使用原功能。新任务须用于训练、筛选、导出，不能用旧任务代替。

## Windows PowerShell 启动

不切换正在运行的工作区；新开终端执行：

```powershell
$repo = 'C:\Users\cldus\workspace\cartpole'
$tree = "C:\Users\cldus\workspace\cartpole-train-400hz-acc9-$(Get-Date -Format yyyyMMdd-HHmmss)"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
git -C $repo fetch origin feat/changqing-train-400hz-acc9
if ($LASTEXITCODE -ne 0) { throw 'fetch 失败' }
git -C $repo worktree add --detach $tree origin/feat/changqing-train-400hz-acc9
if ($LASTEXITCODE -ne 0) { throw '创建工作区失败' }
Set-Location $tree
$env:PYTHONPATH = "$tree\source\cartpole;$env:PYTHONPATH"
$env:PYTHONIOENCODING = 'utf-8'
$task = 'Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0'
$log = Join-Path $tree 'train-400hz-acc9.log'
& $python .\scripts\rsl_rl\train.py `
  --task $task --num_envs 512 --max_iterations 2000 `
  --run_name scratch_window16_tau0_acc9_hz400_candidates `
  --viz none --device cuda:0 --training-candidates *> $log
"训练退出码：$LASTEXITCODE"
Get-Content $log -Tail 50 -Encoding utf8
```

另一个终端可用 `Get-Content '完整日志路径' -Tail 40 -Wait -Encoding utf8` 查看进度。
筛选沿用 [候选筛选命令](../scripts/evaluation/TRAINING_CANDIDATES.md)，但将 `--task` 换成本页的新任务。

## 验证与部署范围

此变更不启动训练、不修改固件、不发送串口命令。基础配置与语法检查不代表 Isaac Lab 训练已通过。实际运行需核对导出的 env 配置为 sim.dt=0.0005、decimation=5、acc_max=9、随机化8–9、window_s=0.016及tau=0。

现有500 Hz部署入口不能直接用于此模型。部署时必须同步支持400 Hz的设备profile、模型合同与上位机周期，并读回固件ACC_MAX=9000；不能仅修改合同字段绕过检查。训练完成先筛选，再用相同任务导出并准备400 Hz联调。
