"""每个向量槽位只采集首个完整回合，避免短失败回合被重复加权。"""
import math


class EpisodeMetrics:
    """状态顺序为 x[m], theta[rad], xdot[m/s], thetadot[rad/s]。

    record 必须在自动重置之前采样，首次调用须从显式 reset 后的首步开始。
    统计仅含控制步结束时样本（包含终态），不是物理子步峰值。
    """
    def __init__(self, *, num_envs, seed, dt):
        if num_envs <= 0 or not math.isfinite(dt) or dt <= 0:
            raise ValueError('num_envs 和 dt 必须为正')
        self.num_envs, self.seed, self.dt = num_envs, seed, dt
        self.episodes = {}
        self.accumulators = [dict(steps=0, theta_sq=0.0, max_abs_x_m=0.0,
                                 max_abs_theta_rad=0.0, max_abs_action=0.0)
                             for _ in range(num_envs)]

    @property
    def complete(self):
        return len(self.episodes) == self.num_envs

    def record(self, *, states, actions, terminated, truncated, reasons):
        """接收一次完整向量步。已完成槽位的后续自动重置回合会被忽略。"""
        arrays = [states, actions, terminated, truncated, *reasons.values()]
        if any(len(values) != self.num_envs for values in arrays):
            raise ValueError('向量长度与 num_envs 不一致')
        for i in range(self.num_envs):
            if i in self.episodes:
                continue
            state, action = list(states[i]), float(actions[i])
            if len(state) != 4 or not all(math.isfinite(x) for x in [*state, action]):
                raise ValueError('状态和动作必须为有限数值，状态必须有四维')
            item = self.accumulators[i]
            item['steps'] += 1
            item['theta_sq'] += state[1] ** 2
            item['max_abs_x_m'] = max(item['max_abs_x_m'], abs(state[0]))
            item['max_abs_theta_rad'] = max(item['max_abs_theta_rad'], abs(state[1]))
            item['max_abs_action'] = max(item['max_abs_action'], abs(action))
            if terminated[i] or truncated[i]:
                self.episodes[i] = dict(
                    env_id=i, seed=self.seed, steps=item['steps'], duration_s=item['steps']*self.dt,
                    success=bool(truncated[i] and not terminated[i]),
                    terminated=bool(terminated[i]), truncated=bool(truncated[i]),
                    termination_reasons=[name for name, values in reasons.items() if values[i]],
                    terminal_state=state, theta_rms_rad=math.sqrt(item['theta_sq']/item['steps']),
                    **{name: item[name] for name in ('max_abs_x_m','max_abs_theta_rad','max_abs_action')})

    def check_reset(self, env_ids):
        """拒绝GUI等外部重置打断未完成回合；正常自动reset时该槽位已完成。"""
        if env_ids is None:
            env_ids = range(self.num_envs)
        elif isinstance(env_ids, slice):
            env_ids = range(self.num_envs)[env_ids]
        interrupted = [int(i) for i in env_ids if int(i) not in self.episodes]
        if interrupted:
            raise RuntimeError(f'未完成回合被外部重置，不能跨reset拼接统计: {interrupted}')

    def report(self):
        episodes = [self.episodes[i] for i in sorted(self.episodes)]
        successes = sum(item['success'] for item in episodes)
        return dict(seed=self.seed, status='complete' if self.complete else 'incomplete',
                    requested=self.num_envs, completed=len(episodes), successes=successes,
                    success_rate=successes/self.num_envs if self.complete else None,
                    incomplete_env_ids=[i for i in range(self.num_envs) if i not in self.episodes],
                    episodes=episodes)


def run_cohort(metrics, *, step, max_steps):
    """step 负责推进环境，并将重置前快照交给 metrics；不改变环境重置规则。"""
    if max_steps <= 0:
        raise ValueError('max_steps 必须为正')
    for _ in range(max_steps):
        if metrics.complete:
            break
        step()
    return metrics.report()


def summarize(cohorts):
    """按回合数汇总；任何未完成 cohort 都不能发布完整集合成功率。"""
    episodes = [episode for cohort in cohorts for episode in cohort['episodes']]
    n, successes = len(episodes), sum(item['success'] for item in episodes)
    complete = bool(cohorts) and all(item['status'] == 'complete' for item in cohorts)
    result = dict(status='complete' if complete else 'incomplete', episodes=n, successes=successes,
                  success_rate=None, wilson_95_low=None, wilson_95_high=None)
    if complete and n:
        p, z = successes/n, 1.959963984540054
        center = (p + z*z/(2*n))/(1+z*z/n)
        radius = z * math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
        result.update(success_rate=p, wilson_95_low=max(0.0,center-radius),
                      wilson_95_high=min(1.0,center+radius))
    return result
