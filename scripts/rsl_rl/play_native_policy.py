#!/usr/bin/env python3
"""在主机 IsaacLab 中通过本机 HTTP 服务执行一个 CartPole 回合。

将本文件、``native_policy_client.py`` 和 ``native_policy_rollout.py`` 复制到
IsaacLab 项目的 ``scripts/rsl_rl/``，与 ``cli_args.py`` 相邻后运行。
"""

from __future__ import annotations

# 本区只使用标准库。Kit 启动时可能改变 cwd，因此先冻结用户给出的路径。
import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import importlib
import importlib.metadata as metadata
import json
import math
import os
from pathlib import Path
import shutil
import sys
import uuid
from typing import Any


SHELL_CWD = Path.cwd().resolve()
SCRIPT_PATH = Path(__file__).resolve()
ORIGINAL_ARGUMENTS = sys.argv[1:].copy()
PATH_ARGUMENTS = ("--bundle", "--checkpoint", "--output_dir", "--output-dir")


def _resolve_from_shell(value: str | os.PathLike[str]) -> Path:
    path = Path(value).expanduser()
    return (SHELL_CWD / path).resolve() if not path.is_absolute() else path.resolve()


def _freeze_path_arguments() -> dict[str, Path]:
    frozen: dict[str, Path] = {}
    for index, token in enumerate(sys.argv[1:], start=1):
        for option in PATH_ARGUMENTS:
            if token == option and index + 1 < len(sys.argv):
                path = _resolve_from_shell(sys.argv[index + 1])
                sys.argv[index + 1] = str(path)
                frozen[option] = path
            elif token.startswith(option + "="):
                path = _resolve_from_shell(token.split("=", 1)[1])
                sys.argv[index] = f"{option}={path}"
                frozen[option] = path
    return frozen


if __name__ == "__main__":
    FROZEN_PATHS = _freeze_path_arguments()

    # Isaac/torch imports must stay after path freezing and inside the executable path.
    import gymnasium as gym
    import numpy as np
    import torch
    from packaging import version
    from rsl_rl.runners import DistillationRunner, OnPolicyRunner

    from isaaclab.envs import DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg
    from isaaclab.utils.seed import configure_seed
    from isaaclab.utils.string import list_intersection, string_to_callable
    from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
    from isaaclab_tasks.utils import add_launcher_args, launch_simulation, setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config

    import isaaclab_tasks  # noqa: F401
    import cli_args  # isort: skip
    import cartpole.tasks  # noqa: F401, E402

    with contextlib.suppress(ImportError):
        import isaaclab_tasks_experimental  # noqa: F401

    try:
        from native_policy_client import PolicyClient, load_bundle, verify_bundle
        from native_policy_rollout import (
            EpisodeInterrupted,
            array_from_tensor,
            check_step_budget,
            classify_episode,
            compare_float_actions,
            episode_statistics,
            execute_control_step,
            policy_observation,
            validate_effective_seed,
        )
    except ImportError:
        from tools.native_policy_bridge.native_policy_client import PolicyClient, load_bundle, verify_bundle
        from tools.native_policy_bridge.native_policy_rollout import (
            classify_episode,
            compare_float_actions,
            episode_statistics,
            EpisodeInterrupted,
            array_from_tensor,
            check_step_budget,
            execute_control_step,
            policy_observation,
            validate_effective_seed,
        )

    parser = argparse.ArgumentParser(description="通过原生策略服务执行一个 CartPole 回合。")
    parser.add_argument("--url", required=True, help="完整的本机推理服务 URL。")
    parser.add_argument("--bundle", required=True, help="包含 manifest.json 的主机核验包目录。")
    parser.add_argument("--task", required=True, help="IsaacLab 任务名称。")
    parser.add_argument("--agent", default="rsl_rl_cfg_entry_point", help="RSL-RL agent 配置入口。")
    parser.add_argument("--seed", type=int, default=17013, help="环境和 agent 种子。")
    parser.add_argument("--mode", choices=("float", "real_quant"), required=True)
    parser.add_argument("--output_dir", "--output-dir", required=True, dest="output_dir")
    parser.add_argument("--external_callback", default=None)
    parser.add_argument("--disable_fabric", action="store_true", default=False)
    cli_args.add_rsl_rl_args(parser)
    add_launcher_args(parser)
    args_cli, remaining_args = setup_preset_cli(parser)
    if not args_cli.checkpoint:
        parser.error("--checkpoint must name one explicit local .pt or .pth file")
    callback_args = None
    if args_cli.external_callback:
        callback_args = string_to_callable(args_cli.external_callback, separator=".")()
    remaining_args = list_intersection(remaining_args, callback_args)
    sys.argv = [sys.argv[0], *remaining_args]
    RSL_RL_VERSION = metadata.version("rsl-rl-lib")


def _sha256(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def _file_identity(path: Path) -> dict[str, str | int]:
    resolved = path.resolve()
    return {"path": str(resolved), "sha256": _sha256(resolved), "size": resolved.stat().st_size}


def _json_value(value: Any, location: str = "root") -> Any:
    if value is None or isinstance(value, (str, bool, int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f"non-finite value at {location}")
        return value
    if isinstance(value, dict):
        return {str(key): _json_value(item, f"{location}.{key}") for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item, f"{location}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, Path):
        return str(value)
    if __name__ == "__main__" and isinstance(value, (np.generic, torch.Tensor, np.ndarray, torch.device)):
        if isinstance(value, torch.Tensor):
            value = value.detach().cpu().numpy()
        if isinstance(value, np.ndarray):
            return _json_value(value.tolist(), location)
        if isinstance(value, np.generic):
            return value.item()
        return str(value)
    if isinstance(value, slice):
        return _json_value([value.start, value.stop, value.step], location)
    if callable(value) and hasattr(value, "__module__") and hasattr(value, "__qualname__"):
        return {"callable": f"{value.__module__}:{value.__qualname__}"}
    raise TypeError(f"unsupported value at {location}: {type(value).__name__}")


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(_json_value(payload), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def _run_with_records(
    run_dir: Path,
    operation: Any,
    *,
    initial_stage: str,
    startup: dict[str, Any] | None,
    on_error: Any,
    write_json: Any,
    stderr: Any,
) -> Any:
    """统一保护 startup、operation、诊断和 failure 发布，同时保留原异常。"""

    failure_stage = initial_stage
    try:
        if startup is not None:
            try:
                write_json(run_dir / "startup.json", startup)
            except BaseException:
                failure_stage = "startup"
                raise
        return operation()
    except BaseException as error:
        failure = {
            "status": "incomplete",
            "stage": getattr(error, "rollout_step_stage", failure_stage),
            "error_type": type(error).__name__,
            "message": str(error),
            "finished_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        if startup is not None:
            failure["startup"] = startup
        if on_error is not None:
            try:
                failure.update(on_error(error))
            except BaseException as diagnostic_error:
                failure["diagnostic_error"] = {
                    "error_type": type(diagnostic_error).__name__,
                    "message": str(diagnostic_error),
                }
        try:
            write_json(run_dir / "failure.json", failure)
        except BaseException as failure_write_error:
            fallback = {
                "event": "failure_record_write_failed",
                "original_error": {
                    "error_type": type(error).__name__,
                    "message": str(error),
                    "stage": failure["stage"],
                },
                "failure_write_error": {
                    "error_type": type(failure_write_error).__name__,
                    "message": str(failure_write_error),
                },
                "failure": failure,
                "startup": startup,
            }
            try:
                stderr.write(json.dumps(_json_value(fallback), ensure_ascii=False, allow_nan=False) + "\n")
                stderr.flush()
            except BaseException:
                pass
        raise


def run_with_failure_record(
    run_dir: Path,
    operation: Any,
    *,
    initial_stage: str,
    on_error: Any = None,
    write_json: Any = None,
    stderr: Any = None,
) -> Any:
    """运行目录创建后的所有失败都发布可诊断的 ``failure.json``。"""

    return _run_with_records(
        run_dir,
        operation,
        initial_stage=initial_stage,
        startup=None,
        on_error=on_error,
        write_json=_write_json if write_json is None else write_json,
        stderr=sys.stderr if stderr is None else stderr,
    )


def run_with_startup_record(
    run_dir: Path,
    startup: dict[str, Any],
    operation: Any,
    *,
    initial_stage: str,
    on_error: Any = None,
    write_json: Any = None,
    stderr: Any = None,
) -> Any:
    """以同一失败边界发布最小启动事实并执行后续操作。"""

    return _run_with_records(
        run_dir,
        operation,
        initial_stage=initial_stage,
        startup=startup,
        on_error=on_error,
        write_json=_write_json if write_json is None else write_json,
        stderr=sys.stderr if stderr is None else stderr,
    )


def write_effective_env_config(run_dir: Path, env: Any) -> None:
    """从最终 wrapper 暴露的真实环境保存有效配置。"""

    _write_json(run_dir / "env_config.json", env.unwrapped.cfg.to_dict())


def _create_run_dir(parent: Path) -> Path:
    parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = parent / f"rollout-{stamp}-{uuid.uuid4().hex}"
    run_dir.mkdir()
    return run_dir


def _snapshot_scripts(run_dir: Path) -> dict[str, dict[str, str | int]]:
    snapshots = run_dir / "scripts"
    snapshots.mkdir()
    identities = {}
    for name in ("play_native_policy.py", "native_policy_client.py", "native_policy_rollout.py"):
        source = SCRIPT_PATH.parent / name
        if not source.is_file():
            raise FileNotFoundError(f"required host script is missing: {source}")
        target = snapshots / name
        shutil.copy2(source, target)
        identities[name] = _file_identity(target)
    return identities


def _resolve_checkpoint() -> Path:
    checkpoint = _resolve_from_shell(args_cli.checkpoint)
    if checkpoint.suffix.lower() not in (".pt", ".pth"):
        raise ValueError("--checkpoint must be an explicit .pt or .pth file")
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint is not a file: {checkpoint}")
    return checkpoint


def _rollout_symbol(name: str) -> Any:
    """测试导入时延迟加载纯 rollout；脚本路径已在 bootstrap 后直接加载。"""

    if name in globals():
        return globals()[name]
    try:
        module = importlib.import_module("native_policy_rollout")
    except ImportError:
        module = importlib.import_module("tools.native_policy_bridge.native_policy_rollout")
    return getattr(module, name)


def _scalar(value: Any, *, name: str, allow_bool: bool = False) -> float:
    flat = _rollout_symbol("array_from_tensor")(value, name=name, allow_bool=allow_bool).reshape(-1)
    if flat.size != 1:
        raise ValueError(f"{name} must contain one finite value")
    return float(flat[0])


def _policy_observation(observations: Any) -> torch.Tensor:
    return _rollout_symbol("policy_observation")(observations)


def _read_true_state(raw_env: Any) -> dict[str, float]:
    asset = raw_env.scene["robot"]
    cart_ids, _ = asset.find_joints("slider_to_cart")
    pole_ids, _ = asset.find_joints("cart_to_pole")
    if len(cart_ids) != 1 or len(pole_ids) != 1:
        raise RuntimeError("expected exactly one cart joint and one pole joint")
    positions = _rollout_symbol("array_from_tensor")(asset.data.joint_pos, name="joint positions")
    if positions.ndim != 2 or positions.shape[0] != 1:
        raise ValueError("joint positions must have shape [1, joints]")
    cart = _scalar(positions[0, cart_ids[0] : cart_ids[0] + 1], name="cart position")
    pole_raw = _scalar(positions[0, pole_ids[0] : pole_ids[0] + 1], name="pole angle")
    pole = math.atan2(math.sin(pole_raw), math.cos(pole_raw))
    return {"cart_position_m": cart, "pole_angle_rad": pole}


def _read_realized_initial_conditions(raw_env: Any) -> dict[str, Any]:
    """逐字段读取 reset 后的真实属性；任何失败都明确记录且使状态 incomplete。"""

    asset = raw_env.scene["robot"]
    readers = {
        "joint_state": lambda: {
            "positions": _rollout_symbol("array_from_tensor")(
                asset.data.joint_pos, name="initial joint positions"
            )[0].tolist(),
            "velocities": _rollout_symbol("array_from_tensor")(
                asset.data.joint_vel, name="initial joint velocities"
            )[0].tolist(),
            "joint_names": list(asset.joint_names),
        },
        "body_masses": lambda: {
            "body_names": list(asset.body_names),
            "values_kg": _rollout_symbol("array_from_tensor")(
                asset.data._body_mass.numpy(), name="body masses"
            )[0].tolist(),
        },
        "body_com": lambda: {
            "body_names": list(asset.body_names),
            "values": _rollout_symbol("array_from_tensor")(
                asset.data.body_com_pose_b, name="body COM poses"
            )[0].tolist(),
        },
        "body_inertia": lambda: {
            "body_names": list(asset.body_names),
            "values": _rollout_symbol("array_from_tensor")(
                asset.data.body_inertia, name="body inertia"
            )[0].tolist(),
        },
        "joint_damping": lambda: {
            "joint_names": list(asset.joint_names),
            "values": _rollout_symbol("array_from_tensor")(
                asset.root_view.get_dof_dampings().numpy(), name="joint damping"
            )[0].tolist(),
        },
        "motor_tau_s": lambda: _scalar(
            raw_env.action_manager.get_term("joint_effort").tau[0:1], name="motor tau"
        ),
        "motor_delay_steps": lambda: int(
            _scalar(raw_env.action_manager.get_term("joint_effort").delay_steps[0:1], name="motor delay")
        ),
    }
    values, errors = {}, {}
    for name, reader in readers.items():
        try:
            values[name] = reader()
        except Exception as error:
            errors[name] = {"error_type": type(error).__name__, "message": str(error)}
    return {"status": "complete" if not errors else "incomplete", "values": values, "errors": errors}


def _randomization_boundaries(raw_env: Any) -> dict[str, Any]:
    cfg = raw_env.cfg
    return {
        "responsibility": "IsaacLab reset event terms; values are read back after explicit reset",
        "reset_events": _json_value(cfg.events.to_dict()),
        "action_processing": _json_value(cfg.actions.to_dict()),
    }


def _termination_snapshot(raw_env: Any) -> dict[str, Any]:
    manager = raw_env.termination_manager
    terminated = bool(_scalar(manager.terminated[0], name="terminated", allow_bool=True))
    timeout = bool(_scalar(manager.time_outs[0], name="timeout", allow_bool=True))
    reasons: list[str] = []
    for name in manager.active_terms:
        if bool(_scalar(manager.get_term(name)[0], name=f"termination term {name}", allow_bool=True)):
            reasons.append(name)
    return {"terminated": terminated, "timeout": timeout, "reasons": reasons}


def _versions() -> dict[str, str]:
    result = {"python": sys.version, "rsl_rl": RSL_RL_VERSION}
    for distribution in ("isaaclab", "isaaclab-rl", "gymnasium", "torch"):
        with contextlib.suppress(metadata.PackageNotFoundError):
            result[distribution] = metadata.version(distribution)
    return result


def _append_trajectory(path: Path, record: dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(_json_value(record), ensure_ascii=False, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())


def _run_episode(env: Any, policy: Any, client: Any, mode: str, run_dir: Path) -> dict[str, Any]:
    reset_result = env.reset()
    observations = reset_result[0] if isinstance(reset_result, tuple) else reset_result
    obs_tensor = _policy_observation(observations)
    obs_array = obs_tensor.detach().cpu().numpy().astype(np.float32, copy=True)
    with torch.inference_mode():
        local_action = policy(observations).detach().cpu().numpy().astype(np.float32, copy=False)
    remote_float = client.infer(obs_array, "float")
    comparison = _rollout_symbol("compare_float_actions")(local_action, remote_float)
    _write_json(run_dir / "float_connection_check.json", comparison)
    if not comparison["within_tolerance"]:
        raise RuntimeError("host checkpoint policy does not match service float policy")

    raw_env = env.unwrapped
    initial = _read_realized_initial_conditions(raw_env)
    initial["randomization_boundaries"] = _randomization_boundaries(raw_env)
    _write_json(run_dir / "initial_conditions.json", initial)
    trajectory_path = run_dir / "trajectory.jsonl"
    records: list[dict[str, Any]] = []
    step_dt = float(raw_env.step_dt)
    step = 0
    while True:
        _rollout_symbol("check_step_budget")(
            step=step, max_episode_length=int(raw_env.max_episode_length)
        )
        obs_tensor = _policy_observation(observations)
        obs_array = obs_tensor.detach().cpu().numpy().astype(np.float32, copy=True)
        true_state = _read_true_state(raw_env)
        request = {"step": step, "mode": mode, "observation": obs_array.tolist()}
        started = datetime.now(timezone.utc)
        started_ns = __import__("time").perf_counter_ns()
        response: dict[str, Any] = {}

        def infer(observation: np.ndarray, selected_mode: str) -> np.ndarray:
            action = client.infer(observation, selected_mode)
            response["action"] = action
            response["latency_ms"] = (__import__("time").perf_counter_ns() - started_ns) / 1_000_000.0
            return action

        try:
            step_result = _rollout_symbol("execute_control_step")(
                env=env,
                infer=infer,
                observation=obs_array,
                mode=mode,
                action_factory=lambda action: torch.as_tensor(
                    action, dtype=torch.float32, device=raw_env.device
                ),
            )
        except BaseException as error:
            request["true_state_before_step"] = true_state
            if getattr(client, "last_request", None) is not None:
                request["client_request"] = client.last_request
            setattr(error, "rollout_request", request)
            setattr(
                error,
                "rollout_step_stage",
                error.stage if isinstance(error, _rollout_symbol("EpisodeInterrupted")) else "environment_step",
            )
            raise
        remote_action = response["action"]
        record = {
            **request,
            **true_state,
            "simulation_time_s": step * step_dt,
            "request_started_at_utc": started.isoformat(),
            "request_latency_ms": response["latency_ms"],
            "client_request": getattr(client, "last_request", None),
            "action": _scalar(remote_action, name="remote action"),
        }
        # execute_control_step 是每个有效响应唯一一次推进；此处先读终止信息，再处理新观测。
        try:
            termination = _termination_snapshot(raw_env)
        except BaseException as error:
            request["true_state_before_step"] = true_state
            request["client_request"] = getattr(client, "last_request", None)
            setattr(error, "rollout_request", request)
            setattr(error, "rollout_step_stage", "termination_readback")
            raise
        record["termination"] = termination
        _append_trajectory(trajectory_path, record)
        records.append(record)
        outcome = _rollout_symbol("classify_episode")(
            terminated=termination["terminated"], timeout=termination["timeout"]
        )
        if outcome != "running":
            return {
                "status": "complete",
                "outcome": outcome,
                "termination": termination,
                "statistics": _rollout_symbol("episode_statistics")(records, step_dt=step_dt),
                "initial_conditions_verification": initial["status"],
            }
        observations = step_result[0]
        step += 1


def main(env_cfg: Any, agent_cfg: Any) -> None:
    run_dir = _create_run_dir(_resolve_from_shell(args_cli.output_dir))
    print(f"[INFO] 本次单回合目录: {run_dir}", flush=True)
    stage = "startup"
    env = None
    client = None
    last_failure: tuple[BaseException, dict[str, Any]] | None = None
    startup = {
        "rollout_version": 1,
        "rollout_id": run_dir.name,
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "arguments": ORIGINAL_ARGUMENTS,
        "shell_cwd": str(SHELL_CWD),
        "url": args_cli.url,
        "bundle": str(_resolve_from_shell(args_cli.bundle)),
        "task": args_cli.task,
        "agent": args_cli.agent,
        "mode": args_cli.mode,
        "seed": {"requested": args_cli.seed, "effective": None},
        "checkpoint": None,
        "script": None,
        "script_snapshots": None,
        "service_health": None,
        "bundle_identity": None,
        "versions": None,
    }

    def execute() -> None:
        nonlocal agent_cfg, client, env, stage
        _rollout_symbol("validate_effective_seed")(requested=args_cli.seed, effective=args_cli.seed)
        stage = "checkpoint"
        checkpoint = _resolve_checkpoint()
        checkpoint_identity = _file_identity(checkpoint)
        startup["checkpoint"] = checkpoint_identity
        _write_json(run_dir / "startup.json", startup)
        stage = "script_snapshot"
        startup["script"] = _file_identity(SCRIPT_PATH)
        _write_json(run_dir / "startup.json", startup)
        script_snapshots = _snapshot_scripts(run_dir)
        startup.update(
            {
                "script_snapshots": script_snapshots,
                "versions": _versions(),
            }
        )
        _write_json(run_dir / "startup.json", startup)
        stage = "bundle_verification"
        bundle = load_bundle(_resolve_from_shell(args_cli.bundle))
        expected_checkpoint = bundle["identity"]["checkpoint_sha256"]
        if checkpoint_identity["sha256"] != expected_checkpoint:
            raise ValueError("checkpoint SHA-256 does not match bundle identity")
        client = PolicyClient(args_cli.url, bundle["identity"], timeout=10)
        startup["service_health"] = client.health
        startup["transport_handshake"] = getattr(client, "handshake_exchange", None)
        startup["bundle_identity"] = bundle["identity"]
        _write_json(run_dir / "startup.json", startup)
        verification = verify_bundle(client, bundle)
        _write_json(run_dir / "bundle_verification.json", verification)

        if version.parse(RSL_RL_VERSION) < version.parse("4.0.0"):
            raise RuntimeError(f"this rollout requires RSL-RL >= 4; installed: {RSL_RL_VERSION}")

        def simulate_and_record() -> None:
            nonlocal agent_cfg, env, stage
            agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
            env_cfg.scene.num_envs = 1
            agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, RSL_RL_VERSION)
            env_cfg.seed = agent_cfg.seed
            _rollout_symbol("validate_effective_seed")(requested=args_cli.seed, effective=env_cfg.seed)
            env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
            env_cfg.log_dir = str(checkpoint.parent)
            _write_json(run_dir / "agent_config.json", agent_cfg.to_dict())
            env = gym.make(args_cli.task, cfg=env_cfg)
            effective_seed = env.unwrapped.cfg.seed
            effective_seed = _rollout_symbol("validate_effective_seed")(
                requested=args_cli.seed, effective=effective_seed
            )
            startup["seed"]["effective"] = effective_seed
            _write_json(run_dir / "startup.json", startup)
            if isinstance(env.unwrapped.cfg, DirectMARLEnvCfg):
                from isaaclab.envs import multi_agent_to_single_agent

                env = multi_agent_to_single_agent(env)
            env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
            write_effective_env_config(run_dir, env)
            if agent_cfg.class_name == "OnPolicyRunner":
                runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            elif agent_cfg.class_name == "DistillationRunner":
                runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            else:
                raise ValueError(f"unsupported runner class: {agent_cfg.class_name}")
            if args_cli.deterministic:
                configure_seed(env_cfg.seed, True)
            stage = "model_load"
            runner.load(str(checkpoint))
            policy = runner.get_inference_policy(device=env.unwrapped.device)
            stage = "rollout"
            report = _run_episode(env, policy, client, args_cli.mode, run_dir)
            stage = "environment_close"
            closing, env = env, None
            closing.close()
            stage = "connection_close"
            client.close()
            stage = "source_verification"
            if _file_identity(checkpoint) != checkpoint_identity:
                raise RuntimeError("checkpoint changed during rollout")
            report.update(
                {
                    "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                    "checkpoint": checkpoint_identity,
                    "bundle_identity": bundle["identity"],
                    "service_health": client.health,
                    "mode": args_cli.mode,
                    "task": args_cli.task,
                    "seed": startup["seed"],
                }
            )
            stage = "report_write"
            report["completion_scope"] = "episode_and_environment"
            _write_json(run_dir / "report.json", report)
            print(f"[INFO] 单回合报告已保存: {run_dir}", flush=True)

        stage = "simulation"
        with launch_simulation(env_cfg, args_cli):
            # Kit 的关闭可能直接结束进程，成功和失败都必须在此边界内落盘。
            run_with_failure_record(
                run_dir, simulate_and_record, initial_stage=stage, on_error=failure_details
            )
            stage = "application_close"

    def failure_details(error: BaseException) -> dict[str, Any]:
        nonlocal env, last_failure
        if last_failure is not None and last_failure[0] is error:
            return last_failure[1]
        details: dict[str, Any] = {"stage": getattr(error, "rollout_step_stage", stage)}
        if last_failure is not None:
            previous_error, previous_details = last_failure
            details["stage"] = "application_close"
            details["preceding_failure"] = {
                "error_type": type(previous_error).__name__,
                "message": str(previous_error),
                **previous_details,
            }
        request = getattr(error, "rollout_request", None)
        if request is not None:
            details["request"] = request
        details["client_last_request"] = getattr(client, "last_request", None)
        details["client_verification_context"] = getattr(client, "verification_context", None)
        if env is not None:
            closing, env = env, None
            try:
                closing.close()
            except BaseException as cleanup_error:
                details["cleanup_error"] = {
                    "error_type": type(cleanup_error).__name__,
                    "message": str(cleanup_error),
                }
        details["transport_exchange"] = getattr(error, "transport_exchange", None)
        if client is not None:
            try:
                client.close()
            except BaseException as connection_cleanup_error:
                details["connection_cleanup_error"] = {
                    "error_type": type(connection_cleanup_error).__name__,
                    "message": str(connection_cleanup_error),
                }
        last_failure = (error, details)
        return details

    run_with_startup_record(
        run_dir, startup, execute, initial_stage=stage, on_error=failure_details
    )


if __name__ == "__main__":
    hydra_task_config(args_cli.task, args_cli.agent)(main)()
