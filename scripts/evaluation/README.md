# 冻结策略的完整回合评估

本工具在当前代码的 V2 训练环境配置下评估指定检查点，不继续训练、不改变训练配置文件、不导出或覆盖模型、不连接真机。默认保留观测噪声与全部训练随机化；它不是恢复旧训练目录的 `env.yaml`。

## 执行

在已经激活的 Isaac Lab 3.0 环境及仓库根目录执行。GPU 与 Kit 可保留；统计按仿真时间，不要求实时运行。

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$ckpt = 'C:\Users\cldus\workspace\cartpole\logs\rsl_rl\cartpole_real_v2\2026-11-15_00-22-27\model_999.pt'
python .\scripts\evaluation\evaluate_policy.py `
  --checkpoint "$ckpt" --num_envs 2 --seeds 101 202 `
  --device cuda:0 --viz kit --info --render-interval 20 `
  *> .\policy-evaluation.log
"退出码: $LASTEXITCODE"
Get-Content .\policy-evaluation.log -Tail 30 -Encoding utf8
```

上面的4回合只验证工具启动、跨种子切换和报告完整性，不是策略验收。正式样本量由启动条件、失败情况和所需置信度决定，不要求固定500回合；请显式指定 `--num_envs` 和 `--seeds`。每回合使用训练配置的10仿真秒。`--horizon-s 60` 只延长本次评估实例的时长，可用于后续长程检验；不得与10秒成绩混在一起。若只做批量统计，可自行选择 `--viz none`，物理后端仍为 PhysX。

输出保存在新的 `logs/evaluation/evaluation-<唯一编号>`，包含 `report.json`、解析后的 `env.yaml` 与 `agent.yaml`。终端首条 `[OUTPUT]` 给出位置。保存 checkpoint 哈希、代码提交与脏状态、种子、实际 dt/后端/设备等。代码提交和配置快照仍不足以证明旧模型与当前物理资产相同，应单独核对训练版本。检查点格式不兼容或运行异常会产生错误报告并以非零退出。

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
- Isaac Lab 3.0 的 recorder 路径会额外计算一次观测，因此同一个 seed 不保证与普通 play 消耗相同的随机数序列。当前任务没有观测历史项；若以后加入历史/有状态观测，应重新审计此采样方案。
- 指标采样涉及 GPU→CPU 传输，会增加耗时，因此不可拿此工具的墙钟耗时替代性能诊断。

## 如何判断

训练中的成功率可作为主要进度指标，但必须明确成功定义以及初始条件。初始分布应对应真机能够可靠进入的启动范围，而不是用不合理的高难度初始化代替实际需求。独立评估用于复核这些条件下的成功率及失败原因，不规定统一样本量或武断阈值。结合每回合初态、失败时间、终态与跟踪参数区分初始恢复失败和保持直立后小车漂移。没有全程轨迹时不能仅凭失败时间断言原因；旧schema1报告没有初态，不能追溯补造。进入真机前还需确认实物执行器、观测、通信时延及保护机制。

旧 `model_999.pt` 训练早于几何修正，应视为待复测候选；能在一个画面中直立是有效观察，但不能代替上述统计。

## 本地单元测试

```bash
python -m unittest discover -s scripts/evaluation -v
```

测试隔离仿真器并通过公开接口注入数据/回调；没有新增集成测试。真实 Isaac Lab / GPU 的运行结果应另行记录，单元测试不能替代它。

## 训练结束后选择检查点

`select_checkpoint.py` 只处理一个已经结束的训练目录。先从同目录 TensorBoard 日志提名少量检查点，再用冻结策略完整回合评估排名，最后用不同种子复核第一名。它不会覆盖模型，也不会控制训练进程。操作期间不要继续向该训练目录写入模型或切换评估代码。

```powershell
$env:PYTHONIOENCODING = 'utf-8'
$run = 'C:\Users\cldus\workspace\cartpole\logs\rsl_rl\cartpole_real_v2\你的训练目录'
python .\scripts\evaluation\select_checkpoint.py `
  --run-dir "$run" --candidates 3 --window 20 `
  --num_envs 32 --seeds 101 202 --holdout-seeds 1001 2002 `
  --device cuda:0 *> .\checkpoint-selection.log
"退出码: $LASTEXITCODE"
Get-Content .\checkpoint-selection.log -Tail 30 -Encoding utf8
```

批量评估固定 `--viz none`，使用 GPU PhysX。默认最多提名4个检查点，每个64回合，第一名再复核64回合。评估条件沿用当前代码，可用 `--horizon-s` 和两个初速度区间参数显式覆盖；每个候选使用相同条件，不改变训练奖励、随机化配置文件。

### 提名与排名规则

1. 安全读取检查点中的 `iter`，用它关联日志横轴，不把文件编号或文件时间当作质量分数。只接受当前用户训练的标准 RSL-RL 权重格式，不使用任意 pickle 兼容回退。
2. 将可匹配日志的检查点按时间顺序分成 `--candidates` 个时段。每个时段根据检查点迭代前 `--window` 轮（含本轮）的 `Train/mean_reward` 平均值提名一个，平分时比较 `Train/mean_episode_length`。额外保留最后保存的检查点并去重。报告保存每个窗口的样本数和确切步数。
3. 训练日志只用于提名：RSL-RL 5.5.1 的回报和长度是最近100个已结束回合的均值，回合可能跨越策略更新；日志迭代的采样发生在更新前，`model_it` 则在更新后保存。这里不是精确的冻结权重成绩。旧 `Metrics/success_rate` 不参与排名，新统计无法追溯补进旧日志。
4. 候选独立评估后严格按成功率降序排列。成功率相同时，依次比较**成功回合**平均绝对车速、绝对首尾位移，较小者优先；仍相同时取较早迭代。它不奖励回到中心，也不会让短时间失败因车程小而获得优势。速度指标包含整个回合，尚未单独划分捕获直立与稳定保持阶段。
5. 仅对第一名运行 `--holdout-seeds`，不再用复核结果重新选模型。复核失败率照实展示，没有预设“可上真机”门槛。所有候选均无成功回合时不推荐模型。

结果只称为**已评估候选中的最佳**。提名可能漏掉更好的模型；只按保存的有限检查点比较也不能证明训练过程全局最优。选择种子与复核种子必须互不重复；反复针对复核集合调参后，应换用新的保留种子。相同初始种子不保证不同策略后续自动重置消耗完全相同的随机流，质量/惯量等全部抽样目前也未逐回合记录。

缺少日志/必要标签、重复日志步数、评估异常或报告不完整均明确报错，不静默替代成功率。也可绕过提名，人工指定同一个训练目录里的候选：

```powershell
python .\scripts\evaluation\select_checkpoint.py `
  --run-dir "$run" `
  --checkpoints "$run\model_400.pt" "$run\model_800.pt" "$run\model_999.pt" `
  --num_envs 32 --seeds 101 202 --holdout-seeds 1001 2002 --device cuda:0
```

输出位于独立的 `logs/checkpoint-selection/selection-<编号>/selection.json`，包含提名映射、日志及模型哈希、逐模型报告路径、排名、独立复核和错误信息。子进程的原始日志和配置也保留在该目录。顶层 `status=complete` 只表示流程完成，不表示成功率达到可用标准。提名与冻结评估为独立函数，后续可用有明确口径的新训练统计替换提名，不猜测未知标签格式。

评估报告 schema 3 增加 `cart_mean_abs_velocity_m_s`、`cart_rms_velocity_m_s`、`cart_travel_m` 和 `cart_net_displacement_m`。行程是控制步末绝对速度的矩形积分；首尾位移是终态位置减实际初态位置，两者都不是回中心误差。
