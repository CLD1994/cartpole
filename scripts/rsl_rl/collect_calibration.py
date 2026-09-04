# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Collect PTQ calibration observations from a trained RSL-RL CartPole policy.

The script rolls out the trained policy in the same Isaac Lab task used for training,
collects the raw ``obs["policy"]`` vectors, and selects nested calibration sets using
Hybrid-v2 controlled-tail sampling:

- 75.00% representative samples from the robust core;
- 18.75% core-coverage farthest-point samples;
-  6.25% controlled-tail farthest-point samples.

For this project's CartPole task the ONNX input order is expected to be::

    [cart_position, pole_angle, cart_velocity, pole_angular_velocity]

Robust normalization/clipping is used only for sample selection. Every saved calibration
array contains the original float32 observations passed to the policy.
"""

from __future__ import annotations

# IMPORTANT: keep this bootstrap section standard-library-only. Isaac Sim / Kit can
# change the process cwd while importing/initializing, so user-supplied relative paths
# must be frozen before importing gymnasium, torch, RSL-RL, Isaac Lab, or local tasks.
import argparse
import contextlib
import importlib.metadata as metadata
import json
import os
import re
import sys
from pathlib import Path


SHELL_CWD = Path.cwd().resolve()


def _looks_like_local_checkpoint(value: str) -> bool:
    """Return True when an explicit checkpoint argument should be treated as local.

    URI-style values are left for Isaac Lab's ``retrieve_file_path``. Everything that
    looks like a filesystem path is frozen relative to the original shell cwd.
    """

    value = value.strip()
    if not value:
        return False

    # Nucleus / HTTP(S) / other URI-like sources should be handled later by Isaac Lab.
    if "://" in value:
        return False

    lower = value.lower()
    if lower.endswith((".pt", ".pth")):
        return True
    if value in (".", ".."):
        return True
    if value.startswith(("./", "../", ".\\", "..\\", "~")):
        return True
    if "/" in value or "\\" in value:
        return True

    # Bare local names are also local if they already exist relative to the invoking shell.
    return (SHELL_CWD / Path(value).expanduser()).exists()


def _resolve_against_shell_cwd(value: str | os.PathLike[str]) -> Path:
    """Resolve a user filesystem path against the cwd from which the script was invoked."""

    path = Path(value).expanduser()
    if not path.is_absolute():
        path = SHELL_CWD / path
    return path.resolve()


def _freeze_explicit_checkpoint_argv() -> tuple[str | None, Path | None]:
    """Freeze explicit local ``--checkpoint`` arguments before any Isaac-related import.

    Supports both ``--checkpoint VALUE`` and ``--checkpoint=VALUE``. The last occurrence
    wins, matching argparse's normal behavior. Local values are also rewritten in
    ``sys.argv`` so downstream parsers cannot reinterpret them relative to a changed cwd.
    """

    raw_checkpoint: str | None = None
    frozen_checkpoint: Path | None = None

    i = 1
    while i < len(sys.argv):
        token = sys.argv[i]
        if token == "--checkpoint":
            if i + 1 >= len(sys.argv):
                # Let argparse report the missing-value error later.
                break
            raw_checkpoint = sys.argv[i + 1]
            if _looks_like_local_checkpoint(raw_checkpoint):
                frozen_checkpoint = _resolve_against_shell_cwd(raw_checkpoint)
                sys.argv[i + 1] = str(frozen_checkpoint)
            else:
                frozen_checkpoint = None
            i += 2
            continue

        if token.startswith("--checkpoint="):
            raw_checkpoint = token.split("=", 1)[1]
            if _looks_like_local_checkpoint(raw_checkpoint):
                frozen_checkpoint = _resolve_against_shell_cwd(raw_checkpoint)
                sys.argv[i] = f"--checkpoint={frozen_checkpoint}"
            else:
                frozen_checkpoint = None
        i += 1

    return raw_checkpoint, frozen_checkpoint


EARLY_CHECKPOINT_ARG, EARLY_CHECKPOINT_PATH = _freeze_explicit_checkpoint_argv()

print(f"[BOOT] shell cwd: {SHELL_CWD}")
if EARLY_CHECKPOINT_PATH is not None:
    print(f"[BOOT] early checkpoint: {EARLY_CHECKPOINT_PATH}")
elif EARLY_CHECKPOINT_ARG is not None:
    print(f"[BOOT] checkpoint source left for Isaac Lab resolver: {EARLY_CHECKPOINT_ARG}")


# Heavy / Isaac-related imports must remain below the cwd/checkpoint bootstrap above.
import gymnasium as gym
import numpy as np
import torch
from packaging import version
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.seed import configure_seed
from isaaclab.utils.string import list_intersection, string_to_callable

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import (
    add_launcher_args,
    get_checkpoint_path,
    launch_simulation,
    setup_preset_cli,
)
from isaaclab_tasks.utils.hydra import hydra_task_config

# local imports -- keep the same registration/CLI behavior as play_rsl_rl.py
import cli_args  # isort: skip

import cartpole.tasks  # noqa: F401, E402

with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401


OBSERVATION_NAMES = (
    "cart_position_m",
    "pole_angle_rad",
    "cart_velocity_m_s",
    "pole_angular_velocity_rad_s",
)
EXPECTED_OBS_DIM = len(OBSERVATION_NAMES)

SELECTION_VERSION = "hybrid-v2-controlled-tail"
SELECTION_SEED_DEFAULT = 42
TAIL_START_PERCENTILE = 95.0
TAIL_END_PERCENTILE = 99.5

ROLE_REPRESENTATIVE = np.uint8(0)
ROLE_CORE_COVERAGE = np.uint8(1)
ROLE_CONTROLLED_TAIL = np.uint8(2)
ROLE_NAMES = {
    int(ROLE_REPRESENTATIVE): "representative_core",
    int(ROLE_CORE_COVERAGE): "core_coverage",
    int(ROLE_CONTROLLED_TAIL): "controlled_tail",
}

# Exact 16-sample cycle for 75% / 18.75% / 6.25%. Repeating this cycle makes
# the default 64/128/256/512 prefixes have exactly 48/12/4, 96/24/8, ... roles.
ROLE_CYCLE = np.asarray(
    [
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_CORE_COVERAGE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_CONTROLLED_TAIL,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_CORE_COVERAGE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_REPRESENTATIVE,
        ROLE_CORE_COVERAGE,
    ],
    dtype=np.uint8,
)


# -- argparse ----------------------------------------------------------------
parser = argparse.ArgumentParser(description="Collect PTQ calibration observations from an RSL-RL CartPole policy.")
parser.add_argument("--num_envs", type=int, default=32, help="Number of parallel environments. Default: 32.")
parser.add_argument("--task", type=str, default=None, help="Task name. Use the same task used for training.")
parser.add_argument(
    "--agent",
    type=str,
    default="rsl_rl_cfg_entry_point",
    help="RL agent configuration entry point.",
)
parser.add_argument("--seed", type=int, default=None, help="Environment/agent seed.")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use a published pretrained checkpoint instead of a local checkpoint.",
)
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument("--external_callback", default=None, help="External environment-registration callback.")

parser.add_argument(
    "--candidate_count",
    "--candidate-count",
    dest="candidate_count",
    type=int,
    default=4096,
    help="Number of rollout observations to collect before selection. Default: 4096.",
)
parser.add_argument(
    "--sample_stride",
    "--sample-stride",
    dest="sample_stride",
    type=int,
    default=6,
    help="Each environment contributes one sample every N policy steps, with a random per-env phase. Default: 6.",
)
parser.add_argument(
    "--warmup_steps",
    "--warmup-steps",
    dest="warmup_steps",
    type=int,
    default=30,
    help="Initial policy steps to run before candidate collection begins. Default: 30.",
)
parser.add_argument(
    "--calib_sizes",
    "--calib-sizes",
    dest="calib_sizes",
    type=int,
    nargs="+",
    default=[64, 128, 256, 512],
    help="Calibration set sizes to export. Default: 64 128 256 512.",
)
parser.add_argument(
    "--fps_clip",
    "--fps-clip",
    dest="fps_clip",
    type=float,
    default=3.0,
    help="Clip robust-scaled features to +/- this value for FPS distance only. Default: 3.0.",
)
parser.add_argument(
    "--selection_seed",
    "--selection-seed",
    dest="selection_seed",
    type=int,
    default=SELECTION_SEED_DEFAULT,
    help=f"Random seed for representative-core ordering. Default: {SELECTION_SEED_DEFAULT}.",
)
parser.add_argument(
    "--output_dir",
    "--output-dir",
    dest="output_dir",
    type=str,
    default=None,
    help="Output directory. Relative paths use the invoking shell cwd. Default: <checkpoint_dir>/calibration.",
)

cli_args.add_rsl_rl_args(parser)
add_launcher_args(parser)
args_cli, remaining_args = setup_preset_cli(parser)


# Call an external callback if requested, matching the existing play script behavior.
remaining_args_env_registration = None
if args_cli.external_callback:
    external_callback_function = string_to_callable(args_cli.external_callback, separator=".")
    remaining_args_env_registration = external_callback_function()

remaining_args = list_intersection(remaining_args, remaining_args_env_registration)
sys.argv = [sys.argv[0]] + remaining_args

installed_version = metadata.version("rsl-rl-lib")


def _validate_args() -> list[int]:
    if args_cli.num_envs <= 0:
        raise ValueError("--num_envs must be > 0")
    if args_cli.candidate_count <= 0:
        raise ValueError("--candidate_count must be > 0")
    if args_cli.sample_stride <= 0:
        raise ValueError("--sample_stride must be > 0")
    if args_cli.warmup_steps < 0:
        raise ValueError("--warmup_steps must be >= 0")
    if args_cli.fps_clip <= 0:
        raise ValueError("--fps_clip must be > 0")

    sizes = sorted(set(args_cli.calib_sizes))
    if not sizes or sizes[0] <= 0:
        raise ValueError("--calib_sizes must contain positive integers")
    if sizes[-1] > args_cli.candidate_count:
        raise ValueError(
            f"largest calibration size ({sizes[-1]}) exceeds candidate_count ({args_cli.candidate_count})"
        )
    return sizes


def _resolve_checkpoint_file(path_value: str | os.PathLike[str], *, source: str) -> Path:
    """Resolve a local checkpoint file, expanding run directories to the newest model_N file."""

    path = Path(path_value).expanduser()
    if not path.is_absolute():
        # Never let Kit's current cwd reinterpret a local checkpoint.
        path = SHELL_CWD / path
    path = path.resolve()

    if not path.exists():
        raise FileNotFoundError(f"{source} checkpoint does not exist: {path}")

    if path.is_dir():
        pattern = re.compile(r"^model_(\d+)\.(?:pt|pth)$", re.IGNORECASE)
        matches: list[tuple[int, Path]] = []
        for child in path.iterdir():
            if not child.is_file():
                continue
            match = pattern.fullmatch(child.name)
            if match is not None:
                matches.append((int(match.group(1)), child))

        if not matches:
            raise FileNotFoundError(
                f"Checkpoint directory contains no model_<N>.pt or model_<N>.pth files: {path}"
            )
        path = max(matches, key=lambda item: item[0])[1].resolve()

    if not path.is_file():
        raise FileNotFoundError(f"Resolved checkpoint is not a file: {path}")

    return path


def _resolve_resume_checkpoint(train_task_name: str, log_root_path: Path, agent_cfg: RslRlBaseRunnerCfg) -> Path:
    """Resolve the checkpoint without reinterpreting explicit local paths after Kit startup."""

    if args_cli.use_pretrained_checkpoint:
        resolved = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resolved:
            raise RuntimeError(f"No published pretrained checkpoint is available for task '{train_task_name}'.")
        return _resolve_checkpoint_file(resolved, source="published")

    if args_cli.checkpoint:
        if EARLY_CHECKPOINT_PATH is not None:
            # Explicit local path: no retrieve_file_path fallback is allowed.
            return _resolve_checkpoint_file(EARLY_CHECKPOINT_PATH, source="local")

        # URI / Nucleus / other Isaac asset source.
        checkpoint_source = args_cli.checkpoint
        resolved = retrieve_file_path(checkpoint_source)
        return _resolve_checkpoint_file(resolved, source="retrieved")

    resolved = get_checkpoint_path(str(log_root_path), agent_cfg.load_run, agent_cfg.load_checkpoint)
    return _resolve_checkpoint_file(resolved, source="configured")


def _resolve_output_dir(checkpoint_file: Path) -> Path:
    if args_cli.output_dir:
        output_dir = _resolve_against_shell_cwd(args_cli.output_dir)
    else:
        output_dir = checkpoint_file.parent / "calibration"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def _robust_selection_space(
    obs: np.ndarray, clip: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build robust-z, clipped FPS features, radius, center, and scale.

    The un-clipped robust-z space defines the 4-D radius. Only FPS distances use
    clipped features. Saved observations are never normalized or clipped.
    """

    center = np.median(obs, axis=0).astype(np.float32)
    q25, q75 = np.percentile(obs, [25.0, 75.0], axis=0)
    robust_scale = ((q75 - q25) / 1.349).astype(np.float32)
    std = obs.std(axis=0).astype(np.float32)

    scale = np.where(robust_scale > 1.0e-8, robust_scale, std)
    scale = np.where(scale > 1.0e-8, scale, np.float32(1.0)).astype(np.float32, copy=False)

    robust_z = ((obs - center) / scale).astype(np.float32, copy=False)
    features = np.clip(robust_z, -clip, clip).astype(np.float32, copy=False)
    radius = np.sqrt(np.einsum("ij,ij->i", robust_z, robust_z)).astype(np.float32, copy=False)
    return robust_z, features, radius, center, scale


def _build_role_schedule(count: int) -> np.ndarray:
    repeats, remainder = divmod(count, len(ROLE_CYCLE))
    if repeats:
        schedule = np.tile(ROLE_CYCLE, repeats)
        if remainder:
            schedule = np.concatenate((schedule, ROLE_CYCLE[:remainder]))
    else:
        schedule = ROLE_CYCLE[:remainder].copy()
    return schedule.astype(np.uint8, copy=False)


def _role_counts(role_schedule: np.ndarray, size: int) -> dict[str, int]:
    counts = np.bincount(role_schedule[:size].astype(np.int64), minlength=3)
    return {
        "representative": int(counts[int(ROLE_REPRESENTATIVE)]),
        "core_coverage": int(counts[int(ROLE_CORE_COVERAGE)]),
        "controlled_tail": int(counts[int(ROLE_CONTROLLED_TAIL)]),
    }


def _hybrid_v2_order(
    features: np.ndarray,
    core_mask: np.ndarray,
    tail_mask: np.ndarray,
    count: int,
    selection_seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Create one nested Hybrid-v2 selection order of ``count`` unique indices."""

    if count > len(features):
        raise ValueError("selection count cannot exceed number of candidate samples")

    role_schedule = _build_role_schedule(count)
    required = _role_counts(role_schedule, count)
    required_core = required["representative"] + required["core_coverage"]
    required_tail = required["controlled_tail"]

    core_indices = np.flatnonzero(core_mask)
    tail_indices = np.flatnonzero(tail_mask)
    if len(core_indices) < required_core:
        raise RuntimeError(
            f"Hybrid-v2 needs {required_core} core samples but only {len(core_indices)} are available. "
            "Increase candidate_count or reduce calibration size."
        )
    if len(tail_indices) < required_tail:
        raise RuntimeError(
            f"Hybrid-v2 needs {required_tail} controlled-tail samples but only {len(tail_indices)} are available. "
            "Increase candidate_count or reduce calibration size."
        )

    rng = np.random.default_rng(selection_seed)
    representative_order = rng.permutation(core_indices)
    representative_cursor = 0

    selected_indices = np.empty(count, dtype=np.int64)
    selected_mask = np.zeros(len(features), dtype=bool)
    min_distance_sq = np.full(len(features), np.inf, dtype=np.float32)

    def update_distances(index: int) -> None:
        delta = features - features[index]
        distance_sq = np.einsum("ij,ij->i", delta, delta)
        np.minimum(min_distance_sq, distance_sq, out=min_distance_sq)
        min_distance_sq[selected_mask] = -1.0

    def take_representative() -> int:
        nonlocal representative_cursor
        while representative_cursor < len(representative_order):
            index = int(representative_order[representative_cursor])
            representative_cursor += 1
            if not selected_mask[index]:
                return index
        raise RuntimeError("Representative-core permutation was exhausted unexpectedly")

    def take_farthest(pool_mask: np.ndarray, pool_name: str) -> int:
        eligible = pool_mask & ~selected_mask
        if not np.any(eligible):
            raise RuntimeError(f"No eligible samples remain in {pool_name} pool")

        if not np.any(selected_mask):
            # Defensive fallback. The normal role cycle starts with a representative sample.
            eligible_indices = np.flatnonzero(eligible)
            norms = np.einsum("ij,ij->i", features[eligible_indices], features[eligible_indices])
            return int(eligible_indices[int(np.argmin(norms))])

        scores = np.where(eligible, min_distance_sq, -np.inf)
        return int(np.argmax(scores))

    for position, role in enumerate(role_schedule):
        role_int = int(role)
        if role_int == int(ROLE_REPRESENTATIVE):
            index = take_representative()
        elif role_int == int(ROLE_CORE_COVERAGE):
            index = take_farthest(core_mask, "core")
        elif role_int == int(ROLE_CONTROLLED_TAIL):
            index = take_farthest(tail_mask, "controlled-tail")
        else:
            raise RuntimeError(f"Unknown selection role code: {role_int}")

        selected_indices[position] = index
        selected_mask[index] = True
        update_distances(index)

    if len(np.unique(selected_indices)) != count:
        raise RuntimeError("Internal error: Hybrid-v2 selection produced duplicate indices")

    return selected_indices, role_schedule


def _describe(obs: np.ndarray) -> dict[str, dict[str, float]]:
    stats: dict[str, dict[str, float]] = {}
    for i, name in enumerate(OBSERVATION_NAMES):
        col = obs[:, i]
        stats[name] = {
            "min": float(np.min(col)),
            "p01": float(np.percentile(col, 1.0)),
            "p05": float(np.percentile(col, 5.0)),
            "median": float(np.median(col)),
            "p95": float(np.percentile(col, 95.0)),
            "p99": float(np.percentile(col, 99.0)),
            "max": float(np.max(col)),
            "mean": float(np.mean(col)),
            "std": float(np.std(col)),
        }
    return stats


def _print_stats(title: str, obs: np.ndarray) -> None:
    print(f"\n[INFO] {title}: shape={obs.shape}")
    print("       name                         min          p01          p50          p99          max")
    for i, name in enumerate(OBSERVATION_NAMES):
        col = obs[:, i]
        print(
            f"       {name:<27}"
            f" {np.min(col):>11.5g}"
            f" {np.percentile(col, 1):>11.5g}"
            f" {np.median(col):>11.5g}"
            f" {np.percentile(col, 99):>11.5g}"
            f" {np.max(col):>11.5g}"
        )


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Roll out the policy, collect raw policy observations, and export Hybrid-v2 sets."""

    calib_sizes = _validate_args()

    if version.parse(installed_version) < version.parse("4.0.0"):
        raise RuntimeError(
            f"This collector targets the project's RSL-RL >= 4 API; installed version is {installed_version}."
        )

    with launch_simulation(env_cfg, args_cli):
        task_name = args_cli.task.split(":")[-1]
        train_task_name = task_name.replace("-Play", "")

        agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
        env_cfg.scene.num_envs = args_cli.num_envs
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)

        env_cfg.seed = agent_cfg.seed
        env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

        # Anchor the default training log tree to the original shell cwd as well.
        log_root_path = (SHELL_CWD / "logs" / "rsl_rl" / agent_cfg.experiment_name).resolve()
        print(f"[INFO] Loading experiment from directory: {log_root_path}")

        resume_file = _resolve_resume_checkpoint(train_task_name, log_root_path, agent_cfg)
        if not resume_file.is_file():
            raise FileNotFoundError(f"Checkpoint must resolve to a file before runner.load(): {resume_file}")

        checkpoint_dir = resume_file.parent
        output_dir = _resolve_output_dir(resume_file)

        env_cfg.log_dir = str(checkpoint_dir)
        env = gym.make(args_cli.task, cfg=env_cfg)

        if isinstance(env.unwrapped.cfg, DirectMARLEnvCfg):
            from isaaclab.envs import multi_agent_to_single_agent

            env = multi_agent_to_single_agent(env)

        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

        print(f"[INFO] Loading checkpoint FILE: {resume_file}")
        if agent_cfg.class_name == "OnPolicyRunner":
            runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        elif agent_cfg.class_name == "DistillationRunner":
            runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
        else:
            raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")

        if args_cli.deterministic:
            configure_seed(env_cfg.seed, True)
        runner.load(str(resume_file))
        policy = runner.get_inference_policy(device=env.unwrapped.device)

        obs = env.get_observations()
        policy_obs = obs["policy"]
        if policy_obs.ndim != 2 or policy_obs.shape[1] != EXPECTED_OBS_DIM:
            raise RuntimeError(
                "Unexpected policy observation shape: "
                f"{tuple(policy_obs.shape)}; expected [num_envs, {EXPECTED_OBS_DIM}] for "
                f"{OBSERVATION_NAMES}."
            )

        num_envs = int(policy_obs.shape[0])
        device = policy_obs.device
        sample_offsets = torch.randint(0, args_cli.sample_stride, (num_envs,), device=device)

        print(
            "[INFO] Collecting candidates: "
            f"num_envs={num_envs}, candidate_count={args_cli.candidate_count}, "
            f"warmup_steps={args_cli.warmup_steps}, sample_stride={args_cli.sample_stride}"
        )
        print(f"[INFO] Observation order: {OBSERVATION_NAMES}")

        candidates: list[np.ndarray] = []
        collected = 0
        step = 0

        try:
            while collected < args_cli.candidate_count:
                with torch.inference_mode():
                    policy_obs = obs["policy"]

                    if step >= args_cli.warmup_steps:
                        sample_mask = sample_offsets == (step % args_cli.sample_stride)
                        if torch.any(sample_mask):
                            batch = policy_obs[sample_mask].detach().cpu().numpy().astype(np.float32, copy=False)
                            remaining = args_cli.candidate_count - collected
                            if len(batch) > remaining:
                                batch = batch[:remaining]
                            candidates.append(batch.copy())
                            collected += len(batch)

                    actions = policy(obs)
                    obs, _, dones, _ = env.step(actions)
                    policy.reset(dones)

                step += 1
                if collected and (collected % 512 < num_envs):
                    print(f"[INFO] Candidate progress: {collected}/{args_cli.candidate_count}", end="\r")
        finally:
            env.close()

        print()
        candidate_obs = np.concatenate(candidates, axis=0).astype(np.float32, copy=False)
        if candidate_obs.shape != (args_cli.candidate_count, EXPECTED_OBS_DIM):
            raise RuntimeError(f"Internal error: collected candidate shape is {candidate_obs.shape}")
        if not np.isfinite(candidate_obs).all():
            bad = np.argwhere(~np.isfinite(candidate_obs))[:10]
            raise RuntimeError(f"Candidate observations contain NaN/Inf at indices {bad.tolist()}")

        np.save(output_dir / "candidates.npy", candidate_obs)
        _print_stats("candidate observations", candidate_obs)

        robust_z, selection_features, robust_radius, robust_center, robust_scale = _robust_selection_space(
            candidate_obs, args_cli.fps_clip
        )
        del robust_z  # radius/features are sufficient from here onward.

        radius_p95 = float(np.percentile(robust_radius, TAIL_START_PERCENTILE))
        radius_p995 = float(np.percentile(robust_radius, TAIL_END_PERCENTILE))
        core_mask = robust_radius <= radius_p95
        tail_mask = (robust_radius > radius_p95) & (robust_radius <= radius_p995)
        extreme_mask = robust_radius > radius_p995

        print(
            "[INFO] Robust-radius pools: "
            f"core={int(core_mask.sum())}, tail={int(tail_mask.sum())}, extreme={int(extreme_mask.sum())}, "
            f"P95={radius_p95:.6g}, P99.5={radius_p995:.6g}"
        )

        max_size = calib_sizes[-1]
        selection_indices, selection_roles = _hybrid_v2_order(
            selection_features,
            core_mask,
            tail_mask,
            max_size,
            args_cli.selection_seed,
        )

        # Selection invariants: core roles never come from tail/extreme; tail role stays in P95..P99.5.
        representative_indices = selection_indices[selection_roles == ROLE_REPRESENTATIVE]
        core_coverage_indices = selection_indices[selection_roles == ROLE_CORE_COVERAGE]
        controlled_tail_indices = selection_indices[selection_roles == ROLE_CONTROLLED_TAIL]
        if not np.all(core_mask[representative_indices]):
            raise RuntimeError("Internal error: representative role selected a non-core sample")
        if not np.all(core_mask[core_coverage_indices]):
            raise RuntimeError("Internal error: core-coverage role selected a non-core sample")
        if not np.all(tail_mask[controlled_tail_indices]):
            raise RuntimeError("Internal error: controlled-tail role selected a sample outside P95..P99.5")
        if np.any(extreme_mask[selection_indices]):
            raise RuntimeError("Internal error: Hybrid-v2 actively selected an extreme >P99.5 sample")

        np.save(output_dir / "robust_radius.npy", robust_radius.astype(np.float32, copy=False))
        np.save(output_dir / f"selection_indices_{max_size}.npy", selection_indices)
        np.save(output_dir / f"selection_roles_{max_size}.npy", selection_roles.astype(np.uint8, copy=False))

        exported: dict[str, str] = {}
        calibration_statistics: dict[str, dict[str, dict[str, float]]] = {}
        role_counts_by_size: dict[str, dict[str, int]] = {}
        for size in calib_sizes:
            prefix_indices = selection_indices[:size]
            selected_obs = candidate_obs[prefix_indices].astype(np.float32, copy=False)
            filename = f"calibration_{size}.npy"
            np.save(output_dir / filename, selected_obs)
            exported[str(size)] = filename
            calibration_statistics[str(size)] = _describe(selected_obs)
            role_counts_by_size[str(size)] = _role_counts(selection_roles, size)
            _print_stats(f"calibration {size}", selected_obs)
            print(f"[INFO] Role counts {size}: {role_counts_by_size[str(size)]}")

        metadata_payload = {
            "selection_version": SELECTION_VERSION,
            "task": args_cli.task,
            "checkpoint": str(resume_file),
            "rsl_rl_version": installed_version,
            "observation_order": list(OBSERVATION_NAMES),
            "onnx_expected_input": {"name": "obs", "shape": [1, EXPECTED_OBS_DIM], "dtype": "float32"},
            "num_envs": num_envs,
            "candidate_count": int(args_cli.candidate_count),
            "warmup_steps": int(args_cli.warmup_steps),
            "sample_stride": int(args_cli.sample_stride),
            "sampling": "random per-environment phase + fixed stride",
            "fps_clip": float(args_cli.fps_clip),
            "robust_center": robust_center.tolist(),
            "robust_scale": robust_scale.tolist(),
            "radius_p95": radius_p95,
            "radius_p995": radius_p995,
            "core_pool_count": int(core_mask.sum()),
            "tail_pool_count": int(tail_mask.sum()),
            "extreme_pool_count": int(extreme_mask.sum()),
            "selection": {
                "representative_ratio": 0.75,
                "core_coverage_ratio": 0.1875,
                "controlled_tail_ratio": 0.0625,
                "tail_start_percentile": TAIL_START_PERCENTILE,
                "tail_end_percentile": TAIL_END_PERCENTILE,
                "selection_seed": int(args_cli.selection_seed),
                "nested_prefixes": True,
            },
            "selection_role_encoding": ROLE_NAMES,
            "role_counts": role_counts_by_size,
            "calibration_sets": exported,
            "candidate_statistics": _describe(candidate_obs),
            "calibration_statistics": calibration_statistics,
        }
        with open(output_dir / "metadata.json", "w", encoding="utf-8") as f:
            json.dump(metadata_payload, f, indent=2, ensure_ascii=False)

        print(f"\n[INFO] Calibration data written to: {output_dir.resolve()}")
        print("[INFO] Saved files:")
        print("       candidates.npy")
        print("       robust_radius.npy")
        print(f"       selection_indices_{max_size}.npy")
        print(f"       selection_roles_{max_size}.npy")
        for size in calib_sizes:
            print(f"       calibration_{size}.npy")
        print("       metadata.json")


if __name__ == "__main__":
    main()
