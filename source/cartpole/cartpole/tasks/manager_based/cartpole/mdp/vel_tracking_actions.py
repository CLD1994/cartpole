# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a joint velocity-tracking action for the stepper-driven cartpole rig (v2).

The v2 rig drives the cart with a stepper motor: the policy command is a target cart velocity,
and the firmware tracks it with a ramp (bounded acceleration). The action term models exactly
that behavior per ``deploy/v2/sim2real-spec.md``:

    v_cmd = clip(action, -1, 1) * vel_max          (authority scale, randomized)
    dv    = clip((v_cmd - v) * dt / (dt + tau), +-acc_max * dt)   (ramp + first-order tracking)
    effort = clip(servo_gain * (v - v_actual), +-effort_limit)    (stiff velocity servo)

``vel_max``, ``tau`` and ``acc_max`` are per-environment tensors so a domain-randomization
event term can sample them per episode. The command transport delay (``delay_steps``) uses
the same attribute names as :class:`JointEffortLagActionCfg`, so the existing
``randomize_motor_delay`` event term works unchanged.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.actions_cfg import JointEffortActionCfg
from isaaclab.utils.configclass import configclass

if TYPE_CHECKING:
    from .vel_tracking_action import JointVelocityTrackingAction


@configclass
class JointVelocityTrackingActionCfg(JointEffortActionCfg):
    """Configuration for the velocity-tracking action of the v2 stepper cartpole rig.

    .. note::
        The action term lives in a separate module (:mod:`vel_tracking_action`) that is only
        imported at runtime (after the simulation app starts) via ``class_type``. Keeping this
        module free of that import lets it be imported during config resolution, before
        ``SimulationApp`` exists.
    """

    class_type: type[JointVelocityTrackingAction] | str = "{DIR}.vel_tracking_action:JointVelocityTrackingAction"

    vel_max: float = 0.9
    """Authority scale [m/s]: the clipped action in [-1, 1] maps to a velocity target in
    [-vel_max, vel_max]. Randomized per-environment by ``randomize_velocity_tracking``.
    Note: the base class ``scale`` is unused (kept at 1.0); ``vel_max`` plays its role as a
    per-environment tensor. The training upper bound must not exceed the firmware session
    clamp ``VEL_MAX`` (see deploy/v2/protocol.md)."""

    tau: float = 0.02
    """Tracking time constant [s]: how fast the tracked velocity converges toward the target
    within the acceleration bound. Randomized per-environment."""

    acc_max: float = 4.0
    """Acceleration limit [m/s^2]: the policy's true authority ceiling (the pole is steered by
    cart acceleration). Randomized per-environment. Training upper bound must not exceed the
    firmware session clamp ``ACC_MAX``."""

    delay_steps: int = 1
    """Command transport delay [policy steps]. The velocity command is held for this many steps
    before it reaches the tracking model. Models sensor streaming, serial link, and inference
    latency of the real loop. Randomized per-environment by ``randomize_motor_delay``."""

    max_delay_steps: int = 3
    """Capacity of the delay history buffer [policy steps]. Per-environment delays sampled by
    the event term are clamped to this value."""

    servo_gain: float = 40.0
    """Effort gain [N/(m/s)] realizing the tracked velocity on the joint: a first-order decay
    with time constant m_eff/servo_gain (~4 ms at m_eff ~ 0.17 kg), an order of magnitude
    faster than ``tau`` so the modeled dynamics dominate."""

    effort_limit: float = 3.0
    """Effort cap [N]. Safety net only (the acceleration limit is enforced by the tracking
    law itself); must comfortably exceed m_eff * acc_max (0.19 kg * 5 m/s^2 = 0.95 N) plus
    the pole gravity reaction (~0.9 N at 60 deg tilt)."""
