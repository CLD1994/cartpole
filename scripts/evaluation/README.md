# 冻结策略的完整回合评估

本工具按 `--task` 选择当前代码中的单动作 V2 训练配置来评估指定检查点，不继续训练、不改变训练配置文件、不导出或覆盖模型、不连接真机。默认保留观测噪声与全部训练随机化；它不是恢复旧训练目录的 `env.yaml`。

## 单个检查点评估

在已经激活的 Isaac Lab 3.0 环境及仓库根目录执行。GPU 与 Kit 可保留；统计按仿真时间，不要求实时运行。

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$ckpt = '替换为本次训练检查点的绝对路径'
$task = 'Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0'
python .\scripts\evaluation\evaluate_policy.py `
  --checkpoint "$ckpt" --task $task --num_envs 2 --seeds 101 202 `
  --device cuda:0 --viz kit --info --render-interval 20 `
  *> .\policy-evaluation.log
"退出码: $LASTEXITCODE"
Get-Content .\policy-evaluation.log -Tail 30 -Encoding utf8
```

示例任务必须换成检查点实际使用的任务；不指定 `--task` 时默认是原始 500 Hz V2，而不是自动读取训练目录。上面的 4 回合只验证工具启动、跨种子切换和报告完整性，不是策略验收。正式样本量由启动条件、失败情况和所需置信度决定，不要求固定500回合；请显式指定 `--num_envs` 和 `--seeds`。每回合使用训练配置的10仿真秒。`--horizon-s 60` 只延长本次评估实例的时长，可用于后续长程检验；不得与10秒成绩混在一起。若只做批量统计，可自行选择 `--viz none`，物理后端仍为 PhysX。

输出保存在新的 `logs/evaluation/evaluation-<唯一编号>`，包含 `report.json`、解析后的 `env.yaml` 与 `agent.yaml`。终端首条 `[OUTPUT]` 给出位置。保存 checkpoint 哈希、代码提交与脏状态、种子、实际 dt/后端/设备等。代码提交和配置快照仍不足以证明旧模型与当前物理资产相同，应单独核对训练版本。检查点格式不兼容或运行异常会产生错误报告并以非零退出。

## 任务与速度观测

评估器支持原始 V2、Window16、Window16-Tau0 和 Window16-Tau0-Acc9-Hz400 四种单动作任务。任务名分别为 `Hexchip-Cartpole-Real-V2-v0`、`Hexchip-Cartpole-Real-V2-Window16-v0`、`Hexchip-Cartpole-Real-V2-Window16-Tau0-v0`、`Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0`；此评估器尚不支持双动作任务。Window16 任务本身包含 16 ms 速度观测，不得再叠加 `--velocity-observation-window-ms 16`。该参数用于原始 V2 的显式观测对照，不会自动把原始任务变成 Window16 训练配方；对照结果应与原条件分别记录。

## 仅评估的启动条件

不指定参数时保留训练的初速度范围。若本次验证真机可控的近静止启动，可显式添加：

```powershell
--initial-cart-velocity-range 0 0 `
--initial-pole-velocity-range 0 0
```

单位分别为 m/s、rad/s；也支持有序非零区间。只修改本次评估配置副本，不改训练文件、位置范围或其他随机化。零速度场景与原随机初速度场景应分别报告，不把较容易场景的成功率替代原场景结果。

报告的 `reset_conditions` 保存实际采用的四项初始化范围。每个回合增加 `initial_state`；每个种子的 `initial_conditions` 按 env_id 保存真实初态和抽样后的速度上限、跟踪时间常数、加速度上限、命令延迟参数。初态在显式 reset 返回后、首个控制步之前直接读取无噪声关节状态，不重新计算观测、不额外采样随机数，不计入步数或角度 RMS。跟踪参数是仿真模型内部值，不是实物测量值；尚未记录逐回合质量、惯量等全部随机物理属性。

## 统计口径

- 每个种子的 reset、策略状态重置、观测获取与步进使用相同的 `torch.inference_mode` 作用域，避免跨种子原地修改推理缓存时报错。
- 每个向量槽位只计从显式 reset 起的**第一个完整回合**，它自动重置后的重复回合不计入。所有槽位均完成才结束，避免快速失败者被重复计数。
- 成功定义为超时且同一步没有任何失败终止。这是当前任务的生存定义，不是精确姿态验收。超时与出界同时发生算失败，终止原因全部保留，原因统计不能要求互斥。
- 使用 Isaac Lab 的公开 `RecorderTerm.record_post_step` 钩子，在终止计算之后、自动 reset 之前取原始状态。不会用 reset 后位置伪装终态。
- 输出各回合时长、终态、角度 RMS、位置/角度/动作绝对峰值。角度以直立为零，单位 rad；原始状态不含观测噪声。峰值只在控制步末采样，不保证覆盖物理子步峰值。动作是策略送给动作管理器的标量，不是电机实际速度。
- 最终报告在 Kit 关闭前写入。只有全部请求种子、每个槽位的首回合均完成时，顶层 `status` 和 `summary.status` 才同时为 `complete`；`running`、`incomplete`、`error` 均不发布全局成功率。先检查两个状态与退出码，再解释策略成绩。
- 整体成功率按回合数加权，并给出 Wilson 95% 区间；不是对 reset 时日志均值再求平均。区间是近似二项式统计，不证明各随机化工况都有相同成功率。
- 评估期间不要点击GUI的reset；若未完成槽位被外部重置，工具中止并保留错误与已有回合数据，不跨reset拼接。
- 任何槽位超出步数护栏仍未完成时，报告 `incomplete`、不发布完整集合成功率、退出码非零。退出码0只表示评估数据采集完整，**不表示策略达到要求**。
- Isaac Lab 3.0 的 recorder 路径会额外计算一次观测，因此同一个 seed 不保证与普通 play 消耗相同的随机数序列。Window16 任务已有有状态速度窗口，同一控制步的重复观测不会追加历史，reset 后按窗口长度暖机。其他新历史观测仍需单独核对 recorder 的重复计算行为。
- 指标采样涉及 GPU→CPU 传输，会增加耗时，因此不可拿此工具的墙钟耗时替代性能诊断。

## 如何判断

训练中的成功率可作为主要进度指标，但必须明确成功定义以及初始条件。初始分布应对应真机能够可靠进入的启动范围，而不是用不合理的高难度初始化代替实际需求。独立评估用于复核这些条件下的成功率及失败原因，不规定统一样本量或武断阈值。结合每回合初态、失败时间、终态与跟踪参数区分初始恢复失败和保持直立后小车漂移。没有全程轨迹时不能仅凭失败时间断言原因；旧schema1报告没有初态，不能追溯补造。进入真机前还需确认实物执行器、观测、通信时延及保护机制。

检查点文件名不包含物理模型版本。应保留训练提交、任务名与配置；单次画面中的直立观察不能代替完整回合统计。

## 本地单元测试

```bash
python -m unittest discover -s scripts/evaluation -v
```

测试隔离仿真器并通过公开接口注入数据/回调；没有新增集成测试。真实 Isaac Lab / GPU 的运行结果应另行记录，单元测试不能替代它。

## 训练中保留候选

本流程包含完整回合指标、训练中候选保存及冻结策略评估，不需要在训练分支与筛选分支之间切换。功能只对新启动的进程生效，不修改正在运行的训练。

训练加 `--training-candidates` 后，每轮更新并记录日志后检查最近 100 个 rollout
（采样批次）的完整回合。必须先积满 100 个 rollout，且窗口内至少累计 256 个完整回合，才参与排名。首先比较
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

### 新训练的命令示例

以下示例新建 400 Hz / Acc9 单动作实验，不要为了启用此功能重启已有实验。其他配方应在训练、筛选及导出时统一更换任务名。

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
    --task Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0 `
    --num_envs 512 --max_iterations 2000 `
    --run_name scratch_window16_tau0_acc9_hz400_candidates `
    --viz none --device cuda:0 --training-candidates
if ($LASTEXITCODE -ne 0) { throw '训练未正常完成，请先检查日志' }
```

可调参数：`--candidate-window 100 --candidate-min-episodes 256 --candidate-keep 3
--candidate-interval 1`。默认 interval=1，不跳过符合条件的轮次。
续训时额外传 `--checkpoint '绝对检查点路径'`；`--max_iterations` 表示追加轮次。

## 训练结束后选择检查点

`select_checkpoint.py` 只处理一个已经结束的训练目录。默认读取训练中保存的动态候选，再用冻结策略完整回合评估排名，最后用不同种子复核第一名。它不会覆盖模型，也不会控制训练进程。操作期间不要继续向该训练目录写入模型或切换评估代码。

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$run = '替换为已经结束的具体训练运行目录绝对路径'
$task = 'Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0'
python .\scripts\evaluation\select_checkpoint.py `
  --run-dir "$run" --task $task --parallel-workers 3 `
  --num_envs 32 --seeds 101 202 --holdout-seeds 1001 2002 `
  --device cuda:0 *> .\checkpoint-selection.log
"退出码: $LASTEXITCODE"
Get-Content .\checkpoint-selection.log -Tail 30 -Encoding utf8
```

示例中每个候选评估 64 个回合，第一名再复核 64 个回合。`--parallel-workers` 只限制并发数，不改变候选清单；不传时串行执行。多个进程共享 GPU，显存足够不保证更快，争用或显存不足时将并发降到 2 或 1。任一候选失败不会产生完整成功报告；并行取消和进程清理边界见 [训练指标](../../docs/training/episode-metrics.md#并行筛选检查点)。批量评估固定 `--viz none`，使用 GPU PhysX。评估条件沿用所选任务的当前配置，可用 `--horizon-s` 和两个初速度区间参数显式覆盖；每个候选使用相同条件，不改变训练配置文件。

### 候选来源与排名

| 参数 | 候选来源 |
| --- | --- |
| 默认 `--nomination saved` | 只评估 `training-candidates/candidates.json` 中保留的动态候选。训练默认保留 3 个；不足 3 个只评估现有候选，不追加周期或末检查点。训练自定义更大的 `--candidate-keep` 时评估清单全部候选。 |
| `--checkpoints <路径...>` | 只评估显式指定的同一训练目录内检查点，覆盖提名策略，不追加模型。 |
| `--nomination temporal --candidates 3 --window 20` | 将可关联训练记录的检查点按迭代分成 3 个时段提名，并额外保留末检查点与动态候选，去重后可能超过 4 个。用于没有动态候选的旧训练或显式的时段对照。 |

只要目录中存在动态候选文件或 manifest，就要求 manifest 为 `complete` 且清单与文件一致，包括手动指定候选时。默认模式缺失或空清单会报错，不静默回退到时段提名。此检查不替代确认原训练进程已经退出；运行中、中断、缺失记录或文件不一致的清单不能用于筛选。

允许的文件布局仅为 `<run>/model_<iteration>.pt` 与 `<run>/training-candidates/candidate_<iteration>.pt`，解析符号链接后检查。历史已淘汰候选的保存记录仍保留，但不存在的文件不会参与评估；周期检查点和候选可同轮次共存，输出目录不会冲突。`--run-dir` 必须指向具体运行目录，不能指向 `cartpole_real_v2` 父目录。旧实验未保存的轮次无法补回权重。

所有模式都读取检查点内部 `iter` 并核对文件轮次，只接受可信的本项目训练产物，不使用任意 pickle 兼容回退。文件编号用于标识而不是质量评分。

显式 `temporal` 模式优先使用 `training-episodes.jsonl` 与 `training-checkpoints.jsonl`：按检查点前 `--window` 轮的完整成功数 / 完整结束数提名，同分优先样本更多、然后迭代更早；零完成回合不当作成功率。只有两份新日志都不存在时，才从 TensorBoard 的 `Train/mean_reward` 窗口均值提名，同分比较 `Train/mean_episode_length`。日志不完整或互相矛盾时明确报错。这些训练统计跨越策略更新，不能当作冻结权重成绩。

冻结评估后的排名规则与候选来源无关：

1. 成功率高者优先。
2. 成功率相同时，比较**成功回合**平均绝对车速，再比较绝对首尾位移，较小者优先；仍相同时取较早迭代。不会让短时间失败因车程小而获益，也不要求回到中心。速度指标尚未划分捕获与稳定保持阶段。
3. 仅对第一名运行 `--holdout-seeds`，不再用复核结果重新选模型。复核失败率照实展示，没有自动“可上真机”门槛。所有候选均无成功回合时不推荐模型。

结果只称为**已评估候选中的最佳**。提名可能漏掉更好的模型；只按保存的有限检查点比较也不能证明训练过程全局最优。选择种子与复核种子必须互不重复；反复针对复核集合调参后，应换用新的保留种子。相同初始种子不保证不同策略后续自动重置消耗完全相同的随机流，质量/惯量等全部抽样目前也未逐回合记录。

需要比较指定模型时，可人工指定同一个训练目录里的检查点；仍须匹配任务和候选 manifest 的完整性要求：

```powershell
python .\scripts\evaluation\select_checkpoint.py `
  --run-dir "$run" --task $task `
  --checkpoints "$run\model_400.pt" "$run\model_800.pt" "$run\model_999.pt" `
  --num_envs 32 --seeds 101 202 --holdout-seeds 1001 2002 --device cuda:0
```

输出位于独立的 `logs/checkpoint-selection/selection-<编号>/selection.json`，包含提名映射、日志及模型哈希、逐模型报告路径、排名、独立复核和错误信息。子进程的原始日志和配置也保留在该目录。顶层 `status=complete` 只表示流程完成，不表示成功率达到可用标准。报告中的 `nomination.method` 区分保存候选、手动指定、完整回合时段提名与 TensorBoard 时段提名。

评估报告 schema 3 增加 `cart_mean_abs_velocity_m_s`、`cart_rms_velocity_m_s`、`cart_travel_m` 和 `cart_net_displacement_m`。行程是控制步末绝对速度的矩形积分；首尾位移是终态位置减实际初态位置，两者都不是回中心误差。

进程清理只针对本次运行；已退出父进程遗留的后代进程不保证全部回收。清理异常需按报告的 PID 手动核查。

### 匹配训练时的执行器覆盖

如果训练命令覆盖了动作速度、加速度或时间常数，选模时也应传入相同的覆盖。
先核对训练目录的 `params/env.yaml`，动作参数和重置随机范围必须一起核对；下面的
0.9 m/s、10 m/s²、Tau0 仅是对应训练配置的示例，不是其他模型的默认值：

```powershell
python scripts/evaluation/select_checkpoint.py --run-dir "$run" `
  --task Hexchip-Cartpole-Real-V2-Window16-Tau0-Acc9-Hz400-v0 `
  --nomination saved --parallel-workers 3 --num_envs 32 `
  --seeds 101 202 --holdout-seeds 1001 2002 --device cuda:0 `
  --env-override 'env.actions.joint_effort.vel_max=0.9' `
  --env-override 'env.events.randomize_tracking.params.vel_max_range=[0.9,0.9]' `
  --env-override 'env.actions.joint_effort.acc_max=10' `
  --env-override 'env.events.randomize_tracking.params.acc_max_range=[10,10]' `
  --env-override 'env.actions.joint_effort.tau=0' `
  --env-override 'env.events.randomize_tracking.params.tau_range=[0,0]'
```

`evaluate_policy.py` 也接受相同的 `--env-override`。覆盖只允许上述六个配置键；
速度和加速度为正数，时间常数非负，随机范围为有序二元素数组。
候选和 holdout 使用同一组覆盖，报告记录请求值、生效的执行器配置及实际 reset 抽样。
解析后的完整配置仍保存在每次评估的 `env.yaml`。不指定覆盖时使用所选任务的当前配置，
不会自动恢复训练目录的配置，也不承诺仅这六个覆盖就能重建任意训练环境。
