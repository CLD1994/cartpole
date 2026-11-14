# 训练完整回合指标

使用 `scripts/rsl_rl/train.py --task Hexchip-Cartpole-Real-V2-v0 ...` 启动训练，`Metrics/success_rate` 出现在原有 PPO 指标列表并实时写入原训练 TensorBoard，横轴是 PPO iteration。

## 统计口径

- `Metrics/completed_episodes`：本 iteration 的 rollout 中结束的完整回合数。
- `Metrics/successful_episodes`：这些回合中 timeout 为真且 terminated 为假的数量。
- `Metrics/failed_episodes`：完整回合数减成功数。timeout 和 terminated 同时发生算失败。
- `Metrics/success_rate`：成功数量 / 完整回合数量，范围 0 到 1。
- 零完成回合时不写 success_rate 标量，表格后的提示显示 N/A；`completed_episodes=0` 和 `success_rate_defined=0` 照常写入。不能把缺失点当成成功或失败，也不沿用上一轮值。
- 一帧结束多个环境分别计数；同槽位多次结束分别计数；跨 iteration 的回合只在结束的 iteration 计一次。
- 开启 `init_at_random_ep_len` 时，每槽位首次结束排除，计入 `excluded_partial_episodes`，防止随机缩短的初始回合被当成完整存活。关闭该选项时首回合正常计入。
- `cumulative_success_rate` 是本次训练调用完整成功数之和 / 完整结束数之和，不是各 iteration 成功率的简单平均。恢复 checkpoint 后重新累计。

默认 120 个控制步、每步 2 ms，因此一轮统计覆盖 0.24 仿真秒；它不是 10 秒回合时限。短窗口可能只有失败结束，也可能无回合结束。跨多个 iteration 比较时应累计成功数、完成数后相除，不能平均不同分母的比例。

训练回合可能跨多次策略更新，采用随机动作；这些指标用于训练趋势和 checkpoint 候选筛选，不是固定 checkpoint 的独立评估。多 GPU 时与原 RSL Logger 一致，只输出 rank 0 的统计，不宣称所有 rank 的全局成功率。Kit 手动 reset 被上游标为 terminated，计为失败；统计训练时不要手动 reset。

## 机读结果与 checkpoint

每次 iteration 日志同时追加到本次 run 的 `training-episodes.jsonl`，包含 schema_version=2、绝对 run 目录、实际 iteration/start_iteration、控制步窗口、成功/失败/完整数量和累计量。没有完成回合时 JSON 成功率为 null。

`training-checkpoints.jsonl` 由真实 `save_model(path, it)` 回调写入，记录文件绝对路径、保存完成的真实 iteration 和该轮训练统计。最终 checkpoint 可能与周期保存重复写入同一路径，消费端应按路径取最后一条。恢复训练时使用 runner 实际的迭代编号，例如载入 iter=999 后首新迭代仍可能是 999，不能自行加一。

checkpoint 在本轮策略更新后保存，而指标来自更新之前的 rollout，且完整回合还可能跨更早的策略版本。关联表示保存前可用的训练证据，不代表保存后的策略已经取得该成功率。未完成或强制中断的 rollout 不补报成完整 iteration。

## 实现边界与副作用

项目训练 backend 的生命周期取自 Isaac Lab v3.0.0，复用其环境、启动、配置、保存与清理函数。项目只在 runner 构造后、writer 初始化前接入 Logger 子类；不修改 Isaac Lab 或 RSL-RL 安装文件，不修改奖励、采样、物理模型、策略更新或 checkpoint 内容。环境注册恢复官方 ManagerBasedRLEnv，play/evaluation 不参与统计。

Logger 适配明确限定 rsl-rl-lib 5.5.1；升级需要复核接口，不静默兼容。统计每步在设备上累计整数，每次 iteration 统一新增一次设备到主机传输；保留原 RSL Logger 自身同步和日志行为。有少量计算、文件写入和输出成本，不是零开销。原 TensorBoard 的默认写入缓冲仍有效。

已做语法和差异检查，未增加测试或声称完成 Windows GPU 验证。现场使用独立 run 联调。旧训练没有准确回合计数时不能从旧标量反推或伪造这些文件。
