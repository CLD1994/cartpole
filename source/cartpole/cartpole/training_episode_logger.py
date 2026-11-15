"""RSL-RL 5.5.1 回合统计适配：每次 PPO 日志只提交一份完整回合汇总。"""

from __future__ import annotations

import importlib.metadata
import json
from pathlib import Path

import torch
from rsl_rl.utils.logger import Logger


class EpisodeLogger(Logger):
    """只扩展统计，不改 rollout、奖励、重置、策略更新或 checkpoint 内容。"""

    def __init__(self, *, episode_env, **kwargs):
        super().__init__(**kwargs)
        self._episode_env = episode_env
        self._ready = torch.full(
            (self.num_envs,), not self.cfg.get("init_at_random_ep_len", True),
            dtype=torch.bool, device=episode_env.device,
        )
        self._counts = torch.zeros((3, self.num_envs), dtype=torch.int64, device=episode_env.device)
        self._window_steps = 0
        self._total_steps = 0
        self._totals = [0, 0, 0]
        self._last_row = None
        self._episodes_file = None
        self._checkpoints_file = None

    def init_logging_writer(self):
        super().init_logging_writer()
        if self.writer is not None:
            directory = Path(self.log_dir)
            self._episodes_file = (directory / "training-episodes.jsonl").open("x", encoding="utf-8")
            self._checkpoints_file = (directory / "training-checkpoints.jsonl").open("x", encoding="utf-8")

    def process_env_step(self, rewards, dones, extras, intrinsic_rewards=None):
        if self.writer is not None:
            terminated = self._episode_env.reset_terminated
            truncated = self._episode_env.reset_time_outs
            done = terminated | truncated
            complete = done & self._ready
            self._counts[0].add_(complete)
            self._counts[1].add_(complete & truncated & ~terminated)
            self._counts[2].add_(done & ~self._ready)
            self._ready.logical_or_(done)
            self._window_steps += 1
            self._total_steps += 1
        super().process_env_step(rewards, dones, extras, intrinsic_rewards)

    def log(self, it, start_it, *args, **kwargs):
        if self.writer is None:
            return super().log(it, start_it, *args, **kwargs)
        if self._window_steps != self.cfg["num_steps_per_env"]:
            raise RuntimeError("episode logger requires one log call per complete PPO rollout")
        # 一次 iteration 只在这里新增一次设备到主机传输，不按步/重置 .item()。
        completed, successes, excluded = self._counts.sum(dim=1).cpu().tolist()
        for i, count in enumerate((completed, successes, excluded)):
            self._totals[i] += count
        total, total_successes, total_excluded = self._totals
        rate = successes / completed if completed else None
        cumulative_rate = total_successes / total if total else None
        metrics = {
            "Metrics/completed_episodes": completed,
            "Metrics/successful_episodes": successes,
            "Metrics/failed_episodes": completed - successes,
            "Metrics/excluded_partial_episodes": excluded,
            "Metrics/success_rate_defined": int(completed > 0),
            "Metrics/cumulative_completed_episodes": total,
        }
        if rate is not None:
            metrics["Metrics/success_rate"] = rate
        if cumulative_rate is not None:
            metrics["Metrics/cumulative_success_rate"] = cumulative_rate
        # 这些key仅在iteration结尾出现一次；官方Logger的mean不再按步或reset批次重加权。
        self.ep_extras.append(metrics)
        super().log(it, start_it, *args, **kwargs)
        if rate is None:
            print("Metrics/success_rate: N/A (no complete episodes ended in this iteration)")
        row = {
            "schema_version": 2,
            "run_dir": str(Path(self.log_dir).resolve()),
            "rank": 0,
            "scope": "rank-0 training episodes ending in this rollout; not fixed-checkpoint evaluation",
            "iteration": int(it),
            "start_iteration": int(start_it),
            "window_start_step": self._total_steps - self._window_steps,
            "window_end_step": self._total_steps,
            "window_sim_s_per_env": self._window_steps * self._episode_env.step_dt,
            "completed": completed,
            "successes": successes,
            "failures": completed - successes,
            "success_rate": rate,
            "excluded_partial_episodes": excluded,
            "cumulative_completed": total,
            "cumulative_successes": total_successes,
            "cumulative_failures": total - total_successes,
            "cumulative_success_rate": cumulative_rate,
            "cumulative_excluded_partial_episodes": total_excluded,
        }
        self._episodes_file.write(json.dumps(row, allow_nan=False) + "\n")
        self._episodes_file.flush()
        self._last_row = row
        self._counts.zero_()
        self._window_steps = 0

    def save_model(self, path, it):
        super().save_model(path, it)
        if self.writer is not None:
            if self._last_row is None or self._last_row["iteration"] != it:
                raise RuntimeError("checkpoint must follow its iteration log")
            row = {
                "schema_version": 1,
                "checkpoint": str(Path(path).resolve()),
                "saved_after_update_iteration": int(it),
                "training_iteration_metrics": self._last_row,
                "scope": "training rollout before this policy update; not evaluation of saved policy",
            }
            self._checkpoints_file.write(json.dumps(row, allow_nan=False) + "\n")
            self._checkpoints_file.flush()

    def stop_logging_writer(self):
        super().stop_logging_writer()
        for file in (self._episodes_file, self._checkpoints_file):
            if file is not None:
                file.close()


def install_episode_logger(runner, env):
    """在官方runner构造后、writer初始化前接入版本限定的项目Logger。"""
    if importlib.metadata.version("rsl-rl-lib") != "5.5.1":
        raise RuntimeError("episode logger is verified against rsl-rl-lib 5.5.1; review adapter before upgrading")
    original = runner.logger
    if original.writer is not None:
        raise RuntimeError("episode logger must be installed before logging starts")
    if not hasattr(env.unwrapped, "termination_manager"):
        raise TypeError("episode logger requires the Isaac Lab manager-based termination buffers")
    replacement = EpisodeLogger(
        episode_env=env.unwrapped,
        log_dir=original.log_dir,
        cfg=original.cfg,
        env_cfg=original.env_cfg,
        num_envs=original.num_envs,
        is_distributed=runner.is_distributed,
        gpu_world_size=runner.gpu_world_size,
        gpu_global_rank=runner.gpu_global_rank,
        device=original.device,
    )
    replacement.git_status_repos = original.git_status_repos
    runner.logger = replacement
