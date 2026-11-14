# 训练完整回合统计

通过 `scripts/rsl_rl/train.py --task Hexchip-Cartpole-Real-V2-v0 ...` 启动的新进程自动启用统计。不改变正在运行的进程；直接使用 Isaac Lab 官方训练入口时，需加 `env.training_episode_log_window_s=1.0`。

## 统计契约

每个环境推进约 1 仿真秒输出一行 `[TRAIN EPISODES]` 和一行 `TRAIN_EPISODES` JSON。窗口按控制步划分，与 PPO iteration 无关；默认 500 Hz 时为 500 步。改频率后按 `round(window_s / step_dt)` 取整。停止并正常关闭环境时输出未满窗口的部分结果。

- `completed` 是本窗口结束的完整回合数，不是环境数、控制步数或 reset 批次数。
- `successes`：timeout 为真且 terminated 为假。两者同时发生算失败；`failures = completed - successes`。
- `success_rate = successes / completed`。没有完整结束回合时为 JSON `null`，短行显示 `N/A`，不是 0%，也不沿用上一窗口。
- 一次结束多个槽位，分别计数；一个槽位在窗口内结束多回合，分别计数。跨窗口回合在其结束窗口计一次。
- 每槽位首次结束排除，列为 `excluded_first_episodes`。因为 runner 可随机缩短初始 episode 长度，这样不把不足一个完整时限的 timeout 当成功。即使关闭初始长度随机化，首回合仍统一排除。
- 累计项只覆盖当前进程、当前 rank 已观测到的完整回合。恢复 checkpoint 后重新计数，不恢复旧日志计数。多个 rank 分别输出，未跨 GPU 汇总。
- 正常训练不要在 Kit 中手动 reset。上游将 UI reset 标记为 terminated，因此会按失败记录；显式调用环境 `reset()` 则重新排除各槽位首回合。

统计仅在设备上累加整数，每个窗口统一转主机一次。正常关闭时可能额外转移一次部分窗口。它不替换动作、奖励、reset、采样或优化逻辑；它仍有少量张量运算与日志开销，不是零成本。`env.training_episode_log_window_s=0` 可关闭。play 和 evaluation 默认不启用。

## 如何阅读

`success=95.0% (190/200), failures=10` 表示这个窗口结束了 200 个完整回合，其中 190 个成功。窗口内没有结束的回合不会进入分母。

窗口短于 10 秒回合时，早失败和 timeout 可能集中在不同窗口，应结合累计分母及连续多个窗口观察。累计结果包含此前不同训练阶段的策略，不能表示最新 checkpoint 的独立成功率。训练采用随机动作，且一个回合可跨多次参数更新；正式验收仍使用固定 checkpoint 的独立评估。

RSL 的 `Mean reward` / `Mean episode length` 仍是最近 100 个结束回合的缓冲均值；`Episode_Termination/*` 仍由 Isaac Lab 定义。这些窗口与本日志不同，不应互相当分母。完整回合统计直接输出控制台/重定向日志，不通过 RSL 的标量平均，也不写入 TensorBoard。

## 接口与验证范围

实现继承公开 `ManagerBasedRLEnv`，通过 Gym `entry_point` 注册；使用 `step()` 返回的 terminated/truncated。已核对 Isaac Lab v3.0.0 与 RSL-RL v5.5.1 源码，无安装目录修改。轻量语法和差异检查不代表已在 Windows GPU 后端验证，首次使用需现场联调。
