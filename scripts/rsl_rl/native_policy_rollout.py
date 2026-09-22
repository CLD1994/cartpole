"""不依赖 IsaacLab 的单回合状态、控制步与统计契约。"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from typing import Any

import numpy as np


FLOAT_ACTION_ATOL = 1e-6
FLOAT_ACTION_RTOL = 1e-5


class EpisodeInterrupted(RuntimeError):
    """动作尚未交给环境时发生的接口中断。"""

    def __init__(self, stage: str, cause: BaseException):
        super().__init__(str(cause))
        self.stage = stage
        self.cause = cause


def array_from_tensor(value: Any, *, name: str, allow_bool: bool = False) -> np.ndarray:
    """把已知 NumPy、Torch 或具有 ``.torch`` 的 Isaac 数据包装转换为有限数组。"""

    if hasattr(value, "torch"):
        value = value.torch
    if isinstance(value, np.ndarray):
        result = value
    elif all(hasattr(value, method) for method in ("detach", "cpu", "numpy")):
        result = value.detach().cpu().numpy()
        if not isinstance(result, np.ndarray):
            raise TypeError(f"{name} tensor conversion did not return a NumPy array")
    else:
        raise TypeError(f"{name} must be a NumPy array, Torch tensor, or Isaac .torch wrapper")
    if not np.issubdtype(result.dtype, np.number) and not (allow_bool and np.issubdtype(result.dtype, np.bool_)):
        raise TypeError(f"{name} must contain numeric values")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains non-finite values")
    return result


def policy_observation(observations: Any) -> Any:
    """取得并验证键访问观测容器中的 ``policy`` float32[1,4]。"""

    try:
        policy = observations["policy"]
    except (KeyError, TypeError, IndexError) as error:
        raise ValueError("observations must contain a policy entry") from error
    values = array_from_tensor(policy, name="policy observation")
    if values.shape != (1, 4) or values.dtype != np.float32:
        raise ValueError("expected finite float32 policy observations [1, 4]")
    return policy


def check_step_budget(*, step: int, max_episode_length: int) -> None:
    """未收到真实终止标志时，不允许超过环境声明的回合步数。"""

    if type(step) is not int or type(max_episode_length) is not int or max_episode_length <= 0 or step < 0:
        raise ValueError("step and max_episode_length must be valid integers")
    if step >= max_episode_length:
        raise RuntimeError("termination protocol did not end the episode at max_episode_length")


def validate_effective_seed(*, requested: Any, effective: Any) -> int:
    """种子必须是匹配的非负整数；布尔值不属于合法种子。"""

    if type(requested) is not int or requested < 0:
        raise ValueError("requested seed must be a non-negative integer")
    if type(effective) is not int or effective != requested:
        raise ValueError(f"effective seed {effective!r} does not match requested seed {requested}")
    return effective


def classify_episode(*, terminated: bool, timeout: bool, interrupted: bool = False) -> str:
    """按照物理失败、时间上限和外部中断的优先级分类回合。"""

    if interrupted:
        return "incomplete"
    if terminated:
        return "failed"
    if timeout:
        return "survived"
    return "running"


def execute_control_step(
    *,
    env: Any,
    infer: Callable[[np.ndarray, str], np.ndarray],
    observation: np.ndarray,
    mode: str,
    action_factory: Callable[[np.ndarray], Any],
) -> Any:
    """取得有效 RPC 动作后才推进环境，并且只推进一次。"""

    try:
        action = infer(observation, mode)
    except Exception as error:
        raise EpisodeInterrupted("inference", error) from error
    return env.step(action_factory(action))


def compare_float_actions(
    local: np.ndarray,
    remote: np.ndarray,
    *,
    absolute_tolerance: float = FLOAT_ACTION_ATOL,
    relative_tolerance: float = FLOAT_ACTION_RTOL,
) -> dict[str, Any]:
    """比较主机 Torch 策略和服务 float 策略的 float32 输出。"""

    local_array = np.asarray(local)
    remote_array = np.asarray(remote)
    if local_array.dtype != np.float32 or remote_array.dtype != np.float32:
        raise ValueError("local and remote actions must both be float32")
    if local_array.shape != remote_array.shape or local_array.shape != (1, 1):
        raise ValueError("local and remote actions must both have shape [1, 1]")
    if not np.isfinite(local_array).all() or not np.isfinite(remote_array).all():
        raise ValueError("local and remote actions must be finite")
    errors = np.abs(local_array.astype(np.float64) - remote_array.astype(np.float64))
    return {
        "within_tolerance": bool(
            np.allclose(local_array, remote_array, atol=absolute_tolerance, rtol=relative_tolerance)
        ),
        "absolute_tolerance": float(absolute_tolerance),
        "relative_tolerance": float(relative_tolerance),
        "max_absolute_error": float(errors.max()),
        "local_action": local_array.tolist(),
        "remote_float_action": remote_array.tolist(),
    }


def episode_statistics(records: Sequence[Mapping[str, float]], *, step_dt: float) -> dict[str, Any]:
    """汇总已推进控制步的推进前真实状态与原始动作。"""

    if not records:
        raise ValueError("a complete episode must contain at least one control step")
    if not np.isfinite(step_dt) or step_dt <= 0:
        raise ValueError("step_dt must be a positive finite number")

    pole = np.asarray([row["pole_angle_rad"] for row in records], dtype=np.float64)
    cart = np.asarray([row["cart_position_m"] for row in records], dtype=np.float64)
    action = np.asarray([row["action"] for row in records], dtype=np.float64)
    if not (np.isfinite(pole).all() and np.isfinite(cart).all() and np.isfinite(action).all()):
        raise ValueError("trajectory statistics require finite values")
    delta = np.diff(action)
    return {
        "control_steps": len(records),
        "survival_time_s": float(len(records) * step_dt),
        "pole_angle_rms_rad": float(np.sqrt(np.mean(np.square(pole)))),
        "pole_angle_max_abs_rad": float(np.max(np.abs(pole))),
        "cart_position_min_m": float(cart.min()),
        "cart_position_max_m": float(cart.max()),
        "cart_position_range_m": float(cart.max() - cart.min()),
        "action_rms": float(np.sqrt(np.mean(np.square(action)))),
        "action_delta_rms": float(np.sqrt(np.mean(np.square(delta)))) if len(delta) else 0.0,
        "angle_samples": "pre_step",
    }
