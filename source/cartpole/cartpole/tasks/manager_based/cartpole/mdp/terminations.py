# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""使用仿真真实关节状态判定杆是否失衡。"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import SceneEntityCfg


def pole_angle_out_of_bounds(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    max_angle: float,
) -> torch.Tensor:
    """杆偏离直立零角度严格超过 max_angle 时终止；角度单位为弧度。

    直接读取真实关节位置，不使用带噪声的策略观测。整圈旋转视为同一
    方向；恰好位于边界不终止。非有限状态或无效阈值停止并报告错误。
    """
    if not math.isfinite(max_angle) or not 0.0 < max_angle < math.pi:
        raise ValueError("杆角度阈值必须是 0 到 π 之间的有限弧度值")
    positions = env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids]
    if not torch.isfinite(positions).all():
        raise ValueError("用于失衡判定的真实杆角度包含 NaN 或 Inf")
    angles = torch.atan2(torch.sin(positions), torch.cos(positions))
    return torch.any(torch.abs(angles) > max_angle, dim=1)
