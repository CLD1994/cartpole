# 400 Hz 速度与加速度双动作对照

任务 `Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-Dual-v0` 在独立工作区从头训练，不能续用一维策略权重。固定加速度基线不变。

## 行为契约

策略输入仍为四维观测；输出两维，均先裁剪到 [-1,1]：

- 第一路乘以每环境速度尺度，名义目标速度范围 ±1.25 m/s。
- 第二路线性映射到 [0.1,9] m/s²，即 `0.1 + (action[1]+1)*4.45`。动作 0 对应 4.55 m/s²；它是加减速幅值，不是带方向的加速度。
- 两路作为一对进入原有动作延迟历史。一个策略步只更新一次历史，物理子步共享同一对延迟目标。回合重置清空两路历史、延迟目标和参考速度。
- 实际参考速度斜率上限为请求与逐回合 8–9 m/s² 随机能力的较小值。目标变化不重置参考速度。参考仍经 40 N/(m/s)、±8 N 力伺服驱动，并不保证机械加速度等于参考上限。
- 保留 400 Hz、0.5 ms 物理子步、16 ms 分数窗口、tau=0、原动作延迟和 PPO 配置。原 action_rate 奖励只计算速度动作，权重不变；不额外增加加速度惩罚。

第二路增加探索自由度；保持 PPO 参数并不意味着相同 KL 更新幅度。二维的 entropy 和聚合 action std 不应与一维直接比较。应以相同任务条件下的冻结评估、完整回合成功率及运动质量判断，单个随机种子不能证明架构优越。

## 从头训练

在新的 Windows PowerShell 窗口执行，不切换运行中的工作区：

```powershell
$repo = 'C:\Users\cldus\workspace\cartpole'
$tree = "C:\Users\cldus\workspace\cartpole-train-dual-$(Get-Date -Format yyyyMMdd-HHmmss)"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
git -C $repo fetch origin feat/changqing-train-dual-action
if ($LASTEXITCODE -ne 0) { throw 'fetch 失败' }
git -C $repo worktree add --detach $tree origin/feat/changqing-train-dual-action
if ($LASTEXITCODE -ne 0) { throw '创建工作区失败' }
Set-Location $tree
$env:PYTHONPATH = "$tree\source\cartpole;$env:PYTHONPATH"
$env:PYTHONIOENCODING = 'utf-8'
$task = 'Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-Dual-v0'
$log = Join-Path $tree 'train-dual-action.log'
& $python .\scripts\rsl_rl\train.py `
  --task $task --num_envs 512 --max_iterations 2000 `
  --run_name scratch_window16_tau0_acc9_hz400_dual_candidates `
  --viz none --device cuda:0 --training-candidates *> $log
"训练退出码：$LASTEXITCODE"
Get-Content $log -Tail 50 -Encoding utf8
```

训练候选保存和并行筛选沿用原入口，但筛选/导出必须使用本任务。该策略不能送入现有一维真机 runner；旧 runner 的输出形状检查会拒绝它。真机部署需原子速度/加速度协议和对应双动作适配器，固定 ACC_MAX 9000 mm/s² 为硬上限，WDT_DEC 保持独立，不通过每周期 CONFIG 更改。

## 验证范围

本环境只能进行语法、配置与离线行为检查，不提供 Isaac Lab / GPU 训练或真机验收。首次运行需要确认 action space 为 2、观测为 4，且自动候选正常生成。
