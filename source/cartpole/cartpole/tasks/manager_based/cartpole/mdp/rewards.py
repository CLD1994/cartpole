# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv


def joint_pos_target_l2(
    env: ManagerBasedRLEnv,
    target: float,
    asset_cfg: SceneEntityCfg,
    wrap: bool = True,
) -> torch.Tensor:
    """Penalize joint position deviation from a target value.

    Args:
        env: The environment.
        target: Target joint position [rad or m, depending on joint type].
        asset_cfg: The asset (articulation) whose joints are penalized.
        wrap: Wrap joint positions to (-pi, pi] before taking the difference. Correct for
            revolute joints; must be ``False`` for prismatic joints, whose position is a
            length [m] and must not be angle-wrapped (beyond half a track length of ~3.14 m
            the wrap would fold far positions back near the target and reward deviation).
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos.torch[:, asset_cfg.joint_ids]
    # wrap the joint positions to (-pi, pi) for revolute joints
    if wrap:
        joint_pos = wrap_to_pi(joint_pos)
    # compute the reward
    return torch.sum(torch.square(joint_pos - target), dim=1)

class survival_success_rate(ManagerTermBase):
    """Tracks episode survival as the success metric.

    Returns zero reward (pure metric tracking). Flushes ``Metrics/success_rate``
    into ``extras["log"]`` on episode reset, where success = timed out without
    early termination.
    """

    def __init__(self, env: ManagerBasedRLEnv, cfg: RewardTermCfg):
        super().__init__(cfg, env)

    def reset(self, env_ids: torch.Tensor):
        manager = self._env.termination_manager
        survived = manager.time_outs[env_ids] & ~manager.terminated[env_ids]
        # 初始化或空重置没有可统计的回合，不发布空均值。
        if survived.numel() == 0:
            return
        self._env.extras.setdefault("log", {})["Metrics/success_rate"] = survived.float().mean().item()

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        return torch.zeros(env.num_envs, device=env.device)
