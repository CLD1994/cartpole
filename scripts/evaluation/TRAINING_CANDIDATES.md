# 训练候选保存与并行筛选

本流程包含完整回合指标、训练中候选保存、400 Hz 单动作任务及并行冻结策略评估。
不需要在训练分支与筛选分支之间切换。功能只对新启动的进程生效，不修改正在运行的训练。

## 训练中保留候选

训练加 `--training-candidates` 后，每轮更新并记录日志后检查最近 100 个 rollout
（采样批次）的完整回合。至少累计 256 个完整回合才参与排名。首先比较
`sum(successes) / sum(completed)`，相同则比较控制台 `Mean episode length`、
`Mean reward`，仍相同保留较早轮次。默认保留前三名，不限于 50 的整数倍。
候选替换只删除本功能生成的候选文件，不删除周期检查点。

成功率统计排除起始随机截断回合；长度与奖励来自官方日志最近 100 个结束回合，
可能包含起始部分回合。这是两个不同窗口，长度与奖励仅用于同成功率下排序。
训练回合可能跨多次策略更新，触发指标来自更新前采样，候选保存的是该轮更新后
策略、价值网络和优化器状态。因此训练指标候选不是冻结策略评估冠军，更不是真机验收。
候选窗口可以重叠、轮次可以相邻，不能视为独立试验。

产物位于每个新训练目录的 `training-candidates/`：

- `candidates.json`：保留的候选、排序指标、统计范围和当前训练指标首选。
- `candidate-history.jsonl`：保存与淘汰历史。
- `candidate_<iteration>.pt`：可续训的完整检查点。

`training-checkpoints.jsonl` 同时记录候选保存事件，筛选器用它关联原始回合计数。
正常结束输出 `TRAINING_METRIC_BEST`。每次续训重新累计窗口，不继承旧训练统计。
当前候选功能仅支持单进程训练；不能将额外候选保存 I/O 当作吞吐性能基准。

## 在一个独立工作区使用完整流程

以下训练命令仅供新实验使用，不要为了启用此功能重启已有实验。

```powershell
$repo = 'C:\Users\cldus\workspace\cartpole'
$tree = "C:\Users\cldus\workspace\cartpole-training-selection-$(Get-Date -Format yyyyMMdd-HHmmss)"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
git -C $repo fetch origin main
if ($LASTEXITCODE -ne 0) { throw 'fetch 失败' }
git -C $repo worktree add --detach $tree origin/main
if ($LASTEXITCODE -ne 0) { throw '创建工作区失败' }
Set-Location $tree
$env:PYTHONPATH = "$tree\source\cartpole;$env:PYTHONPATH"
$env:PYTHONIOENCODING = 'utf-8'
& $python .\scripts\rsl_rl\train.py `
    --task Hexchip-Cartpole-Real-V2-Window16-Tau0-v0 `
    --num_envs 512 --max_iterations 2000 `
    --run_name scratch_window16_tau0_candidates `
    --viz none --device cuda:0 --training-candidates
if ($LASTEXITCODE -ne 0) { throw '训练未正常完成，请先检查日志' }
```

可调参数：`--candidate-window 100 --candidate-min-episodes 256 --candidate-keep 3
--candidate-interval 1`。默认 interval=1，不跳过符合条件的轮次。
续训时额外传 `--checkpoint '绝对检查点路径'`；`--max_iterations` 表示追加轮次。

## 训练结束后并行评估

`$runDir` 指向已经结束的、包含训练日志和检查点的具体运行目录。
它可以位于其他工作区；不要指向 `cartpole_real_v2` 父目录。
存在训练候选时，筛选器要求 manifest 状态为 `complete` 且文件清单一致；
缺失记录、运行中或中断状态不参与筛选。此检查不替代确认原训练进程已经退出。

```powershell
$runDir = '替换为已经结束的具体训练运行目录绝对路径'
& $python .\scripts\evaluation\select_checkpoint.py `
    --run-dir $runDir `
    --task Hexchip-Cartpole-Real-V2-Window16-Tau0-v0 `
    --parallel-workers 3 `
    --num_envs 32 --seeds 73117 73123 --holdout-seeds 83117 83123 `
    --device cuda:0 --output-dir .\logs\checkpoint-selection
```

默认 `--nomination saved` 只评估 `candidates.json` 中已保存的动态候选。
训练默认保留 3 个，只有 1 或 2 个时只评估现有候选，不补入周期或末模型。
如果训练自行设置更大的 `--candidate-keep`，则评估清单全部候选。
缺失或空清单清晰报错，不静默回退。`--parallel-workers 3` 仅限制同时评估数，其余排队。
旧策略显式使用 `--nomination temporal --candidates 3 --window 20`：按三个训练时段提名、
额外纳入末检查点及所有动态候选，去重后可能超过四个；`--candidates` 不是动态候选数量。
每候选输出与日志独立；默认不传此参数仍串行。所有候选成功且配置一致后才排名，
随后只有赢家执行独立 holdout；任一评估失败不会产生完整成功报告。
holdout 是复核，不参与重新选冠军，也未设置自动真机放行门槛。
多个进程共享 GPU，显存足够不保证更快，争用或显存不足时降到 2 或 1。
详见 `docs/training-episode-metrics.md` 的并行取消及进程清理边界。

默认已经只评估动态候选，无需额外参数。需要手动挑选时可加 `--checkpoints $chosen`，
它覆盖提名策略且不追加其他模型。例如显式传入动态候选清单：

```powershell
$manifest = Get-Content (Join-Path $runDir 'training-candidates\candidates.json') -Raw | ConvertFrom-Json
$chosen = @($manifest.candidates | ForEach-Object { $_.checkpoint })
if ($chosen.Count -eq 0) { throw '没有符合统计条件的训练候选；请显式使用 --nomination temporal 或指定周期检查点' }
```

允许的文件布局仅为 `<run>/model_<iteration>.pt` 与
`<run>/training-candidates/candidate_<iteration>.pt`，解析符号链接后检查。
历史已淘汰候选保存记录保留有效，但不存在的文件不会参与评估。
周期检查点和候选可以同轮次共存，输出目录不会冲突。
没有启用训练候选的旧实验需显式使用 `--nomination temporal` 或 `--checkpoints` 筛选，无法补回未保存轮次的权重。

任务必须匹配训练目标：原始 V2 用 `Hexchip-Cartpole-Real-V2-v0`；
保留原执行器滤波的 Window16 用 `Hexchip-Cartpole-Real-V2-Window16-v0`；
Window16 且去掉执行器滤波用 `Hexchip-Cartpole-Real-V2-Window16-Tau0-v0`。
400 Hz、9 m/s² 单动作实验必须使用 `Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0`，
不能用 500 Hz Tau0 任务替代。此评估器尚不支持双动作任务。
选择阶段与 holdout 使用相同任务并校验报告身份。

交付环境只做基础语法、参数、生产消费链与替身检查，未运行 Isaac GPU 训练或并行评估。
