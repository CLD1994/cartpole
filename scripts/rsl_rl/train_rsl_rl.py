# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""项目训练入口；生命周期来自 Isaac Lab v3.0.0，仅接入项目回合 Logger。"""

from __future__ import annotations

import argparse
import contextlib
import os
import time
from datetime import datetime

import torch.distributed as dist

from isaaclab.app import add_launcher_args, launch_simulation, report_activity
from isaaclab.envs import DirectMARLEnvCfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.seed import configure_seed
from isaaclab.utils.string import list_intersection

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path, resolve_task_config, setup_preset_cli

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    check_rsl_rl_version,
    create_rsl_rl_runner,
    handle_deprecated_rsl_rl_cfg,
)
from isaaclab_rl.utils.wandb import announce_new_run, is_wandb_checkpoint, resolve_wandb_checkpoint, resolve_wandb_entity
from isaaclab_rl.entrypoints.common import (
    CHECKPOINT_SELECTORS,
    add_common_train_args,
    apply_env_overrides,
    apply_video_recording,
    close_env,
    create_isaaclab_env,
    dump_train_configs,
    enable_cameras_for_video,
    pre_launch_video_config,
    resolve_checkpoint_selector,
    scoped_torch_backend_flags,
    set_hydra_args,
    show_run_summary,
    startup_screen,
    validate_distributed_device,
    wrap_sensor_capture,
    write_run_manifest,
)
from isaaclab_rl.entrypoints.backends import cli_args_rsl_rl as cli_args
from cartpole.training_episode_logger import install_episode_logger

# PLACEHOLDER: Extension template (do not remove this comment)
with contextlib.suppress(ImportError):
    import isaaclab_tasks_experimental  # noqa: F401


def _parse_args(argv: list[str]) -> argparse.Namespace:
    """Parse RSL-RL training arguments."""
    parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
    add_common_train_args(
        parser,
        agent_default="rsl_rl_cfg_entry_point",
        agent_help="Name of the RL agent configuration entry point.",
    )
    parser.add_argument(
        "--external_callback", default=None, help="Fully qualified path to an externally defined callback."
    )
    parser.add_argument(
        "--resume_optimizer_state",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Restore the policy optimizer; disable to keep a fresh optimizer and configured learning rate.",
    )
    parser.add_argument("--resume-learning-rate", type=float, default=None)
    parser.add_argument("--adaptive-lr-max", type=float, default=None)
    parser.add_argument("--training-candidates", action="store_true")
    parser.add_argument("--candidate-window", type=int, default=100)
    parser.add_argument("--candidate-min-episodes", type=int, default=256)
    parser.add_argument("--candidate-interval", type=int, default=1)
    parser.add_argument("--candidate-keep", type=int, default=3)
    cli_args.add_rsl_rl_args(parser)
    add_launcher_args(parser)
    remaining_args_env_registration = cli_args.register_external_tasks(argv)
    args_cli, remaining_args = setup_preset_cli(parser, argv)
    enable_cameras_for_video(args_cli)
    set_hydra_args(list_intersection(remaining_args, remaining_args_env_registration))
    return args_cli


def _resolve_checkpoint(args_cli: argparse.Namespace, agent_cfg: RslRlBaseRunnerCfg, log_root_path: str) -> str | None:
    """Resolve the checkpoint to resume from, or None when training starts from scratch."""
    if args_cli.checkpoint and is_wandb_checkpoint(args_cli.checkpoint):
        return resolve_wandb_checkpoint(args_cli.checkpoint)
    if args_cli.checkpoint in CHECKPOINT_SELECTORS:
        return resolve_checkpoint_selector(
            log_root_path,
            args_cli.checkpoint,
            library="rsl_rl",
            task=args_cli.task,
            checkpoint_pattern=r"model_.*\.pt",
            metadata={"agent": args_cli.agent},
        )
    if args_cli.checkpoint and os.path.isdir(args_cli.checkpoint):
        return get_checkpoint_path(
            os.path.dirname(args_cli.checkpoint), os.path.basename(args_cli.checkpoint), agent_cfg.load_checkpoint
        )
    if args_cli.checkpoint:
        return retrieve_file_path(args_cli.checkpoint)
    if agent_cfg.algorithm.class_name == "Distillation":
        raise ValueError("Distillation training requires --checkpoint.")
    return None


def run(argv: list[str]) -> None:
    """Train an RSL-RL agent while restoring the caller's Torch backend settings."""
    args_cli = _parse_args(argv)
    with scoped_torch_backend_flags(
        cuda_matmul_allow_tf32=True,
        cudnn_allow_tf32=True,
        cudnn_deterministic=False,
        cudnn_benchmark=True,  # Reuse autotuned convolution algorithms for fixed input shapes.
    ):
        _run(args_cli)


def _run(args_cli: argparse.Namespace) -> None:
    """Execute RSL-RL training with parsed arguments."""
    installed_version = check_rsl_rl_version()

    with startup_screen(args_cli, num_stages=3) as screen:
        env_cfg, agent_cfg = resolve_task_config(args_cli.task, args_cli.agent)
        pre_launch_video_config(env_cfg, args_cli)
        show_run_summary(screen, args_cli, env_cfg, library="rsl_rl", action="train")
        screen.stage("Launching simulation")
        with launch_simulation(env_cfg, args_cli), contextlib.ExitStack() as cleanup:
            agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
            agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
            apply_env_overrides(args_cli, env_cfg)
            validate_distributed_device(args_cli)
            if args_cli.max_iterations is not None:
                agent_cfg.max_iterations = args_cli.max_iterations
            rank = int(os.getenv("RANK", "0")) if args_cli.distributed else None
            if rank is not None:
                agent_cfg.device = env_cfg.sim.device
                agent_cfg.seed += rank
            env_cfg.seed = agent_cfg.seed

            log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
            print(f"[INFO] Logging experiment in directory: {log_root_path}")
            run_name = args_cli.run_timestamp or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            print(f"Exact experiment name requested from command line: {run_name}")
            if agent_cfg.run_name:
                run_name += f"_{agent_cfg.run_name}"
            log_dir = os.path.join(log_root_path, run_name)
            # rank 0 writes its settings and videos where a single-GPU run does; other ranks use rank_<rank>/
            rank_dir = log_dir if rank in (None, 0) else os.path.join(log_dir, f"rank_{rank}")
            # All ranks share the run folder, so its manifest is written once.
            if rank in (None, 0):
                write_run_manifest(log_dir, library="rsl_rl", task=args_cli.task, metadata={"agent": args_cli.agent})

            resume_path = _resolve_checkpoint(args_cli, agent_cfg, log_root_path)
            env_cfg.log_dir = rank_dir
            apply_video_recording(env_cfg, rank_dir, args_cli)

            screen.stage("Creating environment")
            env = create_isaaclab_env(
                args_cli.task,
                env_cfg,
                args_cli,
                convert_marl_to_single_agent=isinstance(env_cfg, DirectMARLEnvCfg),
            )
            cleanup.callback(lambda: close_env(env))
            env = wrap_sensor_capture(env, rank_dir, args_cli)

            screen.stage("Preparing agent")
            start_time = time.time()
            report_activity("Wrapping environment")
            env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
            report_activity(None)
            report_activity("Building policy")
            runner = create_rsl_rl_runner(env, agent_cfg, log_dir=log_dir)
            if args_cli.task.startswith("Hexchip-Cartpole"):
                install_episode_logger(runner, env)
            report_activity(None)

            # configure_seed must run after runner construction so torch determinism does not disturb its initialization
            if args_cli.deterministic:
                configure_seed(env_cfg.seed, torch_deterministic=True)

            runner.add_git_repo_to_log(__file__)
            if resume_path is not None:
                print(f"[INFO]: Loading model checkpoint from: {resume_path}")
                # map to this process's device; the checkpoint's tensors otherwise land on the GPU that saved them
                runner.load(
                    resume_path,
                    load_cfg=cli_args.checkpoint_load_cfg(
                        args_cli.resume_optimizer_state, runner_class=agent_cfg.class_name
                    ),
                    map_location=agent_cfg.device,
                )
            if args_cli.resume_learning_rate is not None or args_cli.adaptive_lr_max is not None:
                if (resume_path is None or not args_cli.resume_optimizer_state
                        or args_cli.resume_learning_rate is None or args_cli.adaptive_lr_max is None):
                    raise ValueError("LR experiment requires checkpoint, optimizer restore and both LR flags")
                from cartpole.training_lr_guard import install_lr_guard
                install_lr_guard(runner, initial_lr=args_cli.resume_learning_rate,
                                 max_lr=args_cli.adaptive_lr_max, log_dir=rank_dir)
            if args_cli.training_candidates:
                from cartpole.training_candidates import install_training_candidates
                install_training_candidates(runner, window=args_cli.candidate_window,
                    min_episodes=args_cli.candidate_min_episodes,
                    interval=args_cli.candidate_interval, keep=args_cli.candidate_keep)
            dump_train_configs(rank_dir, env_cfg, agent_cfg)

            if agent_cfg.logger == "wandb":
                announce_new_run(agent_cfg.wandb_project, resolve_wandb_entity())

            screen.close()
            with contextlib.suppress(KeyboardInterrupt):
                runner.learn(
                    num_learning_iterations=agent_cfg.max_iterations,
                    init_at_random_ep_len=agent_cfg.init_at_random_ep_len,
                )
                print(f"Training time: {round(time.time() - start_time, 2)} seconds")
                # the RL library creates the process group but never destroys it, which torch warns about at exit
                if dist.is_initialized():
                    dist.destroy_process_group()
