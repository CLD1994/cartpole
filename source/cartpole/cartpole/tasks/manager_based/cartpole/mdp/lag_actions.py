# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a joint effort action with a first-order motor response lag for sim2real transfer."""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.actions_cfg import JointEffortActionCfg
from isaaclab.utils.configclass import configclass

if TYPE_CHECKING:
    from .lag_action import JointEffortLagAction


@configclass
class JointEffortLagActionCfg(JointEffortActionCfg):
    """Configuration for a joint effort action with a first-order motor response lag.

    The effort command sent to the joint is low-pass filtered to model the response lag of a
    voltage-driven DC gearmotor without a current loop. The time constant ``tau`` [s] can be
    randomized per-environment by a domain-randomization event term.

    .. note::
        The action term lives in a separate module (:mod:`lag_action`) that is only imported at
        runtime (after the simulation app starts) via ``class_type``. Keeping this module free of
        that import lets it be imported during config resolution, before ``SimulationApp`` exists.
    """

    class_type: type[JointEffortLagAction] | str = "{DIR}.lag_action:JointEffortLagAction"

    tau: float = 0.05
    """Motor time constant [s]. The delivered force follows the desired force with a first-order
    response ``F += (F_desired - F) * dt / (dt + tau)``. Defaults to 0.05 (~3 steps at 60 Hz)."""

    delay_steps: int = 0
    """Command transport delay [policy steps]. The effort command is held for this many steps
    before it reaches the motor model, on top of the first-order lag. Models the sensor
    streaming, serial link, and inference latency of the real control loop. Randomized
    per-environment by a domain-randomization event term."""

    max_delay_steps: int = 0
    """Capacity of the delay history buffer [policy steps]. Per-environment delays sampled by
    the event term are clamped to this value. Set it to the upper bound of the randomized
    delay range."""

    deadzone: float = 0.0
    """Stiction deadzone [N]. ``|effort| < deadzone`` delivers zero force (motor can't overcome
    static friction); above it, the Coulomb friction ``deadzone`` is subtracted from the magnitude.
    Defaults to 0 (no deadzone)."""
