"""按固定仿真时间窗口统计完整训练回合，不经过 RSL 的逐步标量平均。"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import torch
from isaaclab.envs import ManagerBasedRLEnv


class TrainingMetricsEnv(ManagerBasedRLEnv):
    """仅附加日志；动作、奖励、重置与观测均由官方环境执行。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        seconds = self.cfg.training_episode_log_window_s
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("training_episode_log_window_s must be finite and nonnegative")
        self._metrics_interval = max(1, round(seconds / self.step_dt)) if seconds else 0
        self._metrics_steps = 0
        self._metrics_total_steps = 0
        self._metrics_cumulative = [0, 0, 0]
        self._metrics_file = None
        if self._metrics_interval:
            if not self.cfg.log_dir:
                raise ValueError("episode metrics require cfg.log_dir")
            directory = Path(self.cfg.log_dir)
            directory.mkdir(parents=True, exist_ok=True)
            self._metrics_file = (directory / "training-episodes.jsonl").open("x", encoding="utf-8")
            # 首回合可能被 runner 随机缩短。每个槽位的首次结束只建立完整起点。
            self._metrics_ready = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            self._metrics_counts = torch.zeros((3, self.num_envs), dtype=torch.int64, device=self.device)

    def reset(self, *args, **kwargs):
        result = super().reset(*args, **kwargs)
        if getattr(self, "_metrics_interval", 0):
            # 显式 reset 不是自然完成；保留已完成计数，重新建立完整回合起点。
            self._metrics_ready.zero_()
        return result

    def step(self, action):
        result = super().step(action)
        if not self._metrics_interval:
            return result
        terminated, truncated = result[2], result[3]
        done = terminated | truncated
        complete = done & self._metrics_ready
        self._metrics_counts[0].add_(complete)
        self._metrics_counts[1].add_(complete & truncated & ~terminated)
        self._metrics_counts[2].add_(done & ~self._metrics_ready)
        self._metrics_ready.logical_or_(done)
        self._metrics_steps += 1
        self._metrics_total_steps += 1
        if self._metrics_steps == self._metrics_interval:
            self._flush_episode_metrics(partial=False)
        return result

    def _flush_episode_metrics(self, *, partial):
        # 唯一新增的设备到主机传输：每个窗口一次，而非每步或每次重置。
        completed, successes, excluded = self._metrics_counts.sum(dim=1).cpu().tolist()
        for i, count in enumerate((completed, successes, excluded)):
            self._metrics_cumulative[i] += count
        total, total_successes, total_excluded = self._metrics_cumulative
        payload = {
            "schema_version": 1,
            "run_dir": str(Path(self.cfg.log_dir).resolve()),
            "rank": int(os.environ.get("RANK", "0")),
            "window_start_step": self._metrics_total_steps - self._metrics_steps,
            "window_end_step": self._metrics_total_steps,
            "window_sim_s_per_env": self._metrics_steps * self.step_dt,
            "partial_window": partial,
            "completed": completed,
            "successes": successes,
            "failures": completed - successes,
            "success_rate": successes / completed if completed else None,
            "excluded_first_episodes": excluded,
            "cumulative_completed": total,
            "cumulative_successes": total_successes,
            "cumulative_failures": total - total_successes,
            "cumulative_success_rate": total_successes / total if total else None,
            "cumulative_excluded_first_episodes": total_excluded,
        }
        rate = f"{successes / completed:.1%}" if completed else "N/A"
        print(f"[TRAIN EPISODES] success={rate} ({successes}/{completed}), failures={completed-successes}, "
              f"window={payload['window_sim_s_per_env']:.3f} simulation seconds/env")
        serialized = json.dumps(payload, allow_nan=False)
        self._metrics_file.write(serialized + "\n")
        self._metrics_file.flush()
        print("TRAIN_EPISODES " + serialized)
        self._metrics_counts.zero_()
        self._metrics_steps = 0

    def close(self):
        if getattr(self, "_metrics_interval", 0) and self._metrics_steps:
            self._flush_episode_metrics(partial=True)
        if getattr(self, "_metrics_file", None) is not None:
            self._metrics_file.close()
            self._metrics_file = None
        super().close()
