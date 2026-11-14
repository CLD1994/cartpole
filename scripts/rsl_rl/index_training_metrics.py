"""将训练完整回合日志转换为 TensorBoard 和 checkpoint 窗口索引，不改变模型。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch
import yaml
from torch.utils.tensorboard import SummaryWriter


def index_metrics(run_dir: Path, start_iteration: int, resume_checkpoint: Path | None = None) -> Path:
    """start_iteration 必须对应本次 runner.learn 起点；新训练为0，恢复训练为载入迭代。"""
    if start_iteration < 0:
        raise ValueError("start_iteration must be nonnegative")
    run_dir = run_dir.resolve()
    # 只取原始标量，兼容配置文件中的 Python tuple 等 YAML 标签，不构造 Python 对象。
    with (run_dir / "params" / "agent.yaml").open(encoding="utf-8") as file:
        agent_cfg = yaml.load(file, Loader=yaml.BaseLoader)
    configured_checkpoint = agent_cfg.get("load_checkpoint", "model_.*.pt")
    resuming = configured_checkpoint != "model_.*.pt" or str(agent_cfg.get("resume", "false")).lower() == "true"
    if resuming:
        if resume_checkpoint is None:
            raise ValueError("resume requires --resume-checkpoint pointing to the actual loaded file")
        if not Path(configured_checkpoint).is_absolute() or Path(configured_checkpoint).resolve() != resume_checkpoint.resolve():
            raise ValueError("resume indexing supports only an exact absolute checkpoint path recorded in agent.yaml")
        resumed = torch.load(resume_checkpoint, map_location="cpu", weights_only=True)
        if int(resumed["iter"]) != start_iteration:
            raise ValueError("start_iteration does not match the loaded checkpoint metadata")
        del resumed
    elif start_iteration != 0 or resume_checkpoint is not None:
        raise ValueError("fresh training must use --start-iteration 0 without --resume-checkpoint")
    rollout = int(agent_cfg["num_steps_per_env"])
    if rollout <= 0:
        raise ValueError("num_steps_per_env must be positive")
    with (run_dir / "training-episodes.jsonl").open(encoding="utf-8") as file:
        windows = [json.loads(line) for line in file if line.strip()]
    previous_end = 0
    for window in windows:
        start, end = window["window_start_step"], window["window_end_step"]
        if start != previous_end or end <= start:
            raise ValueError("episode windows must be contiguous and nonempty")
        completed, successes, failures = (window[key] for key in ("completed", "successes", "failures"))
        if min(completed, successes, failures) < 0 or completed != successes + failures:
            raise ValueError("invalid completed/success/failure counts")
        rate = successes / completed if completed else None
        if window["success_rate"] != rate:
            raise ValueError("success_rate does not match its denominator")
        window["collection_iteration_first"] = start_iteration + start // rollout
        window["collection_iteration_last"] = start_iteration + (end - 1) // rollout
        window["num_steps_per_env"] = rollout
        window["scope"] = "completed training episodes; not fixed-checkpoint evaluation"
        previous_end = end

    checkpoints = []
    for path in run_dir.glob("model_*.pt"):
        # 仅允许安全权重加载，不退回任意 pickle 执行；文件必须来自本次训练。
        state = torch.load(path, map_location="cpu", weights_only=True)
        iteration = int(state["iter"])
        del state
        if iteration < start_iteration:
            raise ValueError(f"checkpoint precedes this training invocation: {path.name}")
        end_step = (iteration - start_iteration + 1) * rollout
        if end_step > previous_end:
            raise ValueError(f"episode log does not cover checkpoint {path.name}; wait for training to finish")
        eligible = [window for window in windows if window["window_end_step"] <= end_step]
        latest = eligible[-1] if eligible else None
        checkpoints.append({
            "checkpoint": path.name,
            "saved_after_update_iteration": iteration,
            "collection_end_step": end_step,
            "latest_complete_window_end_step": latest["window_end_step"] if latest else None,
            "window_age_control_steps": end_step - latest["window_end_step"] if latest else None,
            "latest_window_completed": latest["completed"] if latest else 0,
            "latest_window_successes": latest["successes"] if latest else 0,
            "latest_window_success_rate": latest["success_rate"] if latest else None,
            "scope": "training evidence available before save; not evaluation of saved policy",
        })
    checkpoints.sort(key=lambda item: item["saved_after_update_iteration"])

    # 新目录防止重跑时同一步重复写TensorBoard点；不改训练日志或checkpoint。
    destination = run_dir / "episode_metrics_index"
    destination.mkdir(exist_ok=False)
    with (destination / "windows.jsonl").open("w", encoding="utf-8") as file:
        for window in windows:
            file.write(json.dumps(window, allow_nan=False) + "\n")
    manifest = {
        "schema_version": 1,
        "run_dir": str(run_dir),
        "start_iteration": start_iteration,
        "resume_checkpoint": str(resume_checkpoint.resolve()) if resume_checkpoint else None,
        "num_steps_per_env": rollout,
        "tensorboard_step_axis": "environment control steps since this training invocation",
        "checkpoint_relation": "window outcomes may span policies and precede the saved post-update policy",
        "checkpoints": checkpoints,
    }
    (destination / "checkpoints.json").write_text(json.dumps(manifest, indent=2, allow_nan=False), encoding="utf-8")
    with SummaryWriter(log_dir=str(destination / "tensorboard")) as writer:
        for window in windows:
            end = window["window_end_step"]
            for key in ("completed", "successes", "failures", "excluded_first_episodes",
                        "cumulative_completed", "cumulative_successes", "cumulative_failures",
                        "collection_iteration_first", "collection_iteration_last"):
                writer.add_scalar(f"Metrics/{key}", window[key], end)
            writer.add_scalar("Metrics/success_rate_defined", int(window["completed"] > 0), end)
            if window["success_rate"] is not None:
                writer.add_scalar("Metrics/success_rate", window["success_rate"], end)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--start-iteration", required=True, type=int,
                        help="本次训练起始iteration；新训练填0，resume填实际载入checkpoint的iter")
    parser.add_argument("--resume-checkpoint", type=Path, help="resume时实际载入的checkpoint；必须与agent.yaml的绝对路径一致")
    args = parser.parse_args()
    print(index_metrics(args.run_dir, args.start_iteration, args.resume_checkpoint))


if __name__ == "__main__":
    main()
