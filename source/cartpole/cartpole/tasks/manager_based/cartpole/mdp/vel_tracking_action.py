# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Joint velocity-tracking action for the v2 stepper-driven cartpole rig.

This module is deliberately separate from :mod:`vel_tracking_actions` (which holds only the
config class): importing it pulls in the full environment and physics stack, so it must not be
imported before the simulation app starts. It is resolved lazily at runtime through
:attr:`JointVelocityTrackingActionCfg.class_type`.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.joint_actions import JointEffortAction

from .vel_tracking_actions import JointVelocityTrackingActionCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class JointVelocityTrackingAction(JointEffortAction):
    """Velocity-tracking action implementing the v2 rig's stepper execution model.

    The rig's firmware receives a target cart velocity and tracks it with a bounded-acceleration
    ramp (see deploy/v2/protocol.md). This term reproduces that exact law so the policy trains
    against the behavior the firmware will execute:

    1. the raw action is delayed by ``delay_steps`` policy steps (transport latency),
    2. clipped to [-1, 1] and scaled by the per-env ``vel_max`` into a velocity target,
    3. tracked by an internal velocity reference that converges with first-order time constant
       ``tau`` and per-step increment bounded by ``acc_max * step_dt`` (the ramp),
    4. realized on the joint by a stiff effort servo ``clip(servo_gain * (v_ref - v_actual),
       +-effort_limit)``.

    ``vel_max``, ``tau``, ``acc_max`` and ``delay_steps`` are per-environment tensors so
    domain-randomization events can sample them per episode.
    """

    cfg: JointVelocityTrackingActionCfg

    def __init__(self, cfg: JointVelocityTrackingActionCfg, env: ManagerBasedEnv) -> None:
        super().__init__(cfg, env)
        # authority scale / tracking dynamics per environment, randomized by event terms
        self.vel_max = torch.full((self.num_envs,), cfg.vel_max, device=self.device)
        self.tau = torch.full((self.num_envs,), cfg.tau, device=self.device)
        self.acc_max = torch.full((self.num_envs,), cfg.acc_max, device=self.device)
        # command transport delay per environment [policy steps], randomized by an event term
        self.max_delay_steps = cfg.max_delay_steps
        # clamp: an index beyond the history buffer would raise at gather time
        self.delay_steps = torch.full(
            (self.num_envs,), min(cfg.delay_steps, cfg.max_delay_steps), device=self.device, dtype=torch.long
        )
        # history of recent RAW actions, newest first: [t, t-1, ..., t - max_delay_steps]
        self._action_history = torch.zeros(
            cfg.max_delay_steps + 1, self.num_envs, self.processed_actions.shape[1], device=self.device
        )
        # tracked velocity reference (the output of the ramp law; the servo realizes it)
        self._vel_ref = torch.zeros_like(self.processed_actions)

    def apply_actions(self):
        # push the newest raw command at the front of the history (transport delay happens
        # before the firmware ramp)
        self._action_history = torch.cat([self.processed_actions.unsqueeze(0), self._action_history[:-1]], dim=0)
        # gather each environment's command from ``delay_steps`` steps ago
        env_ids = torch.arange(self.num_envs, device=self.device)
        delayed_actions = self._action_history[self.delay_steps, env_ids]

        # velocity target: clip to [-1, 1] and scale by the per-env authority
        v_cmd = torch.clamp(delayed_actions, -1.0, 1.0) * self.vel_max.unsqueeze(-1)

        # ramp law: first-order convergence toward the target, per-step increment bounded by
        # acc_max * step_dt (dv = v * dt / (dt + tau), saturated at +-a_max * dt)
        step_dt = self._env.step_dt
        k = (step_dt / (step_dt + self.tau)).unsqueeze(-1)
        acc_cap = self.acc_max.unsqueeze(-1) * step_dt
        dv = torch.clamp((v_cmd - self._vel_ref) * k, -acc_cap, acc_cap)
        self._vel_ref += dv

        # stiff effort servo realizing the reference on the joint (safety-capped)
        v_actual = self._asset.data.joint_vel[:, self._joint_ids]
        effort = torch.clamp(
            self.cfg.servo_gain * (self._vel_ref - v_actual), -self.cfg.effort_limit, self.cfg.effort_limit
        )
        self._asset.set_joint_effort_target_index(target=effort, joint_ids=self._joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._vel_ref[env_ids] = 0.0
        self._action_history[:, env_ids] = 0.0
