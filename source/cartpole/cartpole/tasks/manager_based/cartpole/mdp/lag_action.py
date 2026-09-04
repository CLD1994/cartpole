# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Joint effort action with a first-order motor response lag for sim2real transfer.

This module is deliberately separate from :mod:`lag_actions` (which holds only the config class):
importing it pulls in the full environment and physics stack, so it must not be imported before the
simulation app starts. It is resolved lazily at runtime through
:attr:`JointEffortLagActionCfg.class_type`.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.joint_actions import JointEffortAction

from .lag_actions import JointEffortLagActionCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class JointEffortLagAction(JointEffortAction):
    """Joint effort action with command transport delay, motor lag, and stiction deadzone.

    A voltage-driven DC gearmotor delivers force with a first-order lag; the speed-dependent
    back-EMF droop is modeled separately via joint damping (see the task's event terms).
    The real control loop also has transport latency (sensor streaming + serial link +
    inference) of one to a few policy steps, which destabilizes policies trained without
    it. This term therefore delays the effort command by ``delay_steps`` policy steps,
    then applies the first-order lag, then the stiction deadzone. ``tau`` and ``delay_steps``
    are kept as per-environment tensors so domain-randomization events can sample them
    per episode.
    """

    cfg: JointEffortLagActionCfg

    def __init__(self, cfg: JointEffortLagActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        # motor time constant per environment [s], randomized by an event term
        self.tau = torch.full((self.num_envs,), cfg.tau, device=self.device)
        # command transport delay per environment [policy steps], randomized by an event term
        self.max_delay_steps = cfg.max_delay_steps
        # clamp: an index beyond the history buffer would raise at gather time
        self.delay_steps = torch.full(
            (self.num_envs,), min(cfg.delay_steps, cfg.max_delay_steps), device=self.device, dtype=torch.long
        )
        # history of recent effort targets, newest first: [t, t-1, ..., t - max_delay_steps]
        self._action_history = torch.zeros(
            cfg.max_delay_steps + 1, self.num_envs, self.processed_actions.shape[1], device=self.device
        )
        # lagged effort target buffer (the force actually sent to the joint)
        self._lagged_effort = torch.zeros_like(self.processed_actions)

    def apply_actions(self):
        # push the newest command at the front of the history (transport delay happens
        # before the motor response)
        self._action_history = torch.cat([self.processed_actions.unsqueeze(0), self._action_history[:-1]], dim=0)
        # gather each environment's command from ``delay_steps`` steps ago
        env_ids = torch.arange(self.num_envs, device=self.device)
        delayed_actions = self._action_history[self.delay_steps, env_ids]
        # first-order low-pass on the delayed effort target
        k = (self._env.step_dt / (self._env.step_dt + self.tau)).unsqueeze(-1)
        self._lagged_effort += (delayed_actions - self._lagged_effort) * k
        # stiction deadzone: below the threshold the motor delivers no force; above it, subtract
        # the Coulomb friction that keeps opposing motion.
        effort = self._lagged_effort
        if self.cfg.deadzone > 0.0:
            effort = torch.where(
                effort.abs() < self.cfg.deadzone,
                torch.zeros_like(effort),
                effort - torch.sign(effort) * self.cfg.deadzone,
            )
        self._asset.set_joint_effort_target_index(target=effort, joint_ids=self._joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._lagged_effort[env_ids] = 0.0
        self._action_history[:, env_ids] = 0.0
