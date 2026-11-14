# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom domain-randomization event terms for sim2real transfer of the cartpole."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

_logger = logging.getLogger(__name__)
_warned_bodies: set[str] = set()
_logged_bodies: set[str] = set()

# One-time diagnostic for body-name matching, written to a dedicated file so it can be
# found without searching the (very large) console output.
_DIAG_FILE = "randomize_body_mass.log"


def _write_diag(message: str) -> None:
    """Append a one-line diagnostic to the dedicated log file."""
    with open(_DIAG_FILE, "a", encoding="utf-8") as f:
        f.write(message + "\n")


def randomize_body_mass(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    mass_range: dict[str, tuple[float, float]],
) -> None:
    """Randomize the mass of selected rigid bodies of an articulation.

    Body names are matched by regex; a key that matches no body is skipped and logged once.

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        asset_cfg: The asset (articulation) whose bodies are randomized.
        mass_range: Mapping from body name (regex) to (min, max) mass [kg].
    """
    asset = env.scene[asset_cfg.name]
    for body_key, (lo, hi) in mass_range.items():
        body_ids, body_names = asset.find_bodies(body_key)
        if not body_ids:
            if body_key not in _warned_bodies:
                _warned_bodies.add(body_key)
                _write_diag(f"key '{body_key}' matched NO bodies. Available bodies: {asset.body_names}")
                _logger.warning(
                    "randomize_body_mass: no bodies matched key '%s'. Available bodies: %s",
                    body_key,
                    asset.body_names,
                )
            continue
        if body_key not in _logged_bodies:
            _logged_bodies.add(body_key)
            _write_diag(f"key '{body_key}' matched bodies {body_names}")
            _logger.info("randomize_body_mass: key '%s' matched bodies %s", body_key, body_names)
        masses = torch.rand(len(env_ids), len(body_ids), device=env.device) * (hi - lo) + lo
        asset.set_masses_index(masses=masses, body_ids=body_ids, env_ids=env_ids)


def randomize_pole_properties(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    mass_range: tuple[float, float],
    length: float,
) -> None:
    """Set the pole's mass, COM and inertia to match a uniform rod of the given length.

    The USD pole geometry has a much higher COM offset and inertia than the real rod, which makes
    balancing far harder than on the real hardware. This term overrides the dynamics to a uniform
    rod of ``length`` meters pivoted at its base: COM at half length, transverse inertia
    ``m*L^2/12`` about the COM, negligible axial inertia. The pole body frame has the rod along z
    (confirmed by a readback diagnostic of the USD geometry).
    """
    asset = env.scene[asset_cfg.name]
    pole_ids, pole_names = asset.find_bodies("pole")
    if not pole_ids:
        _logger.warning("randomize_pole_properties: no bodies matched key 'pole'.")
        return
    num = len(env_ids)
    # sample mass [kg] per environment
    masses = torch.rand(num, 1, device=env.device) * (mass_range[1] - mass_range[0]) + mass_range[0]
    asset.set_masses_index(masses=masses, body_ids=pole_ids, env_ids=env_ids)
    # center of mass at half length along the rod axis (pole body frame)
    com = torch.zeros(num, 1, 7, device=env.device)
    com[:, :, 2] = length / 2.0  # z offset
    com[:, :, 6] = 1.0  # identity quaternion (x, y, z, w)
    asset.set_coms_index(coms=com, body_ids=pole_ids, env_ids=env_ids)
    # inertia about COM: transverse m*L^2/12, axial ~ 0 (thin rod along z)
    i_trans = masses * (length**2) / 12.0
    inertia = torch.zeros(num, 1, 9, device=env.device)
    inertia[:, :, 0] = i_trans  # I_xx
    inertia[:, :, 4] = i_trans  # I_yy
    inertia[:, :, 8] = i_trans * 0.01  # I_zz (thin rod; small but PhysX requires > 0)
    asset.set_inertias_index(inertias=inertia, body_ids=pole_ids, env_ids=env_ids)

    # one-time diagnostic: read back the pole properties to confirm the override took effect.
    if "_pole_props" not in _logged_bodies:
        _logged_bodies.add("_pole_props")
        e = int(env_ids[0])
        com_b = [round(float(x), 4) for x in asset.data.body_com_pose_b[e, pole_ids[0]]]
        inertia_b = [round(float(x), 4) for x in asset.data.body_inertia[e, pole_ids[0]]]
        mass = float(asset.data._body_mass.numpy()[e, pole_ids[0]])
        _write_diag(f"pole[{pole_names[0]}] AFTER set: mass={mass:.4f}kg com_body={com_b} inertia={inertia_b}")


def randomize_joint_damping(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    damping_range: dict[str, tuple[float, float]],
) -> None:
    """Randomize the damping of selected joints of an articulation.

    This models joint friction together with the back-EMF-induced speed droop of the real motor.

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        asset_cfg: The asset (articulation) whose joints are randomized.
        damping_range: Mapping from joint name (regex) to (min, max) damping.
    """
    asset = env.scene[asset_cfg.name]
    for joint_key, (lo, hi) in damping_range.items():
        joint_ids, _ = asset.find_joints(joint_key)
        damping = torch.rand(len(env_ids), len(joint_ids), device=env.device) * (hi - lo) + lo
        asset.write_joint_damping_to_sim_index(damping=damping, joint_ids=joint_ids, env_ids=env_ids)


def randomize_motor_lag(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    action_name: str,
    tau_range: tuple[float, float],
) -> None:
    """Randomize the motor time constant of the lag action term per environment.

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        action_name: Name of the action term in the action manager.
        tau_range: (min, max) motor time constant [s].
    """
    term = env.action_manager.get_term(action_name)
    lo, hi = tau_range
    term.tau[env_ids] = torch.rand(len(env_ids), device=env.device) * (hi - lo) + lo


def randomize_pole_axis_properties(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    asset_cfg: SceneEntityCfg,
    mass_range: tuple[float, float],
    length: float,
    com_offset_range: tuple[float, float],
    inertia_scale_range: tuple[float, float],
) -> None:
    """Set the pole's mass, COM and inertia for a uniform rod pivoted off-center (v2 rig).

    质量、长度和转轴到质心距离由设备档案传入。绕质心横向惯量按
    ``m*L^2/12`` 计算；平行轴项由刚体质心偏移处理，不重复加到这里。
    质量、质心与惯量比例按各自配置范围随机化。

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        asset_cfg: The asset (articulation) whose pole body is modified.
        mass_range: (min, max) pole mass [kg].
        length: Total rod length [m] (fixed).
        com_offset_range: (min, max) pivot-to-COM distance [m].
        inertia_scale_range: (min, max) multiplier applied to the uniform-rod inertia.
    """
    asset = env.scene[asset_cfg.name]
    pole_ids, pole_names = asset.find_bodies("pole")
    if not pole_ids:
        _logger.warning("randomize_pole_axis_properties: no bodies matched key 'pole'.")
        return
    num = len(env_ids)
    masses = torch.rand(num, 1, device=env.device) * (mass_range[1] - mass_range[0]) + mass_range[0]
    asset.set_masses_index(masses=masses, body_ids=pole_ids, env_ids=env_ids)
    # COM at the sampled offset along the rod axis (pole body frame: rod along z)
    com_lo, com_hi = com_offset_range
    com_offsets = torch.rand(num, 1, device=env.device) * (com_hi - com_lo) + com_lo
    com = torch.zeros(num, 1, 7, device=env.device)
    com[:, :, 2] = com_offsets  # z offset from the pivot
    com[:, :, 6] = 1.0  # identity quaternion (x, y, z, w)
    asset.set_coms_index(coms=com, body_ids=pole_ids, env_ids=env_ids)
    # inertia about COM: transverse m*L^2/12 (uniform rod), scaled by an independent factor
    s_lo, s_hi = inertia_scale_range
    scales = torch.rand(num, 1, device=env.device) * (s_hi - s_lo) + s_lo
    i_trans = masses * (length**2) / 12.0 * scales
    inertia = torch.zeros(num, 1, 9, device=env.device)
    inertia[:, :, 0] = i_trans  # I_xx
    inertia[:, :, 4] = i_trans  # I_yy
    inertia[:, :, 8] = i_trans * 0.01  # I_zz (thin rod; small but PhysX requires > 0)
    asset.set_inertias_index(inertias=inertia, body_ids=pole_ids, env_ids=env_ids)

    if "_pole_v2_props" not in _logged_bodies:
        _logged_bodies.add("_pole_v2_props")
        _write_diag(f"v2 pole[{pole_names[0]}]: mass={masses[0, 0]:.4f}kg com_offset={com_offsets[0, 0]:.4f}m")
        # read back the pole properties to confirm the override took effect (same as v1)
        e = int(env_ids[0])
        com_b = [round(float(x), 4) for x in asset.data.body_com_pose_b[e, pole_ids[0]]]
        inertia_b = [round(float(x), 4) for x in asset.data.body_inertia[e, pole_ids[0]]]
        mass = float(asset.data._body_mass.numpy()[e, pole_ids[0]])
        _write_diag(f"v2 pole[{pole_names[0]}] AFTER set: mass={mass:.4f}kg com_body={com_b} inertia={inertia_b}")


def randomize_velocity_tracking(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    action_name: str,
    vel_max_range: tuple[float, float],
    tau_range: tuple[float, float],
    acc_max_range: tuple[float, float],
) -> None:
    """Randomize the tracking parameters of the velocity-tracking action term per environment.

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        action_name: Name of the action term in the action manager.
        vel_max_range: (min, max) authority scale [m/s]. 该范围是建模参数，
            需要另行核验当前设备的实际执行能力。
        tau_range: (min, max) tracking time constant [s].
        acc_max_range: (min, max) acceleration limit [m/s^2]. Upper bound must not exceed the
            firmware session clamp ``ACC_MAX``.
    """
    term = env.action_manager.get_term(action_name)
    num = len(env_ids)
    lo, hi = vel_max_range
    term.vel_max[env_ids] = torch.rand(num, device=env.device) * (hi - lo) + lo
    lo, hi = tau_range
    term.tau[env_ids] = torch.rand(num, device=env.device) * (hi - lo) + lo
    lo, hi = acc_max_range
    term.acc_max[env_ids] = torch.rand(num, device=env.device) * (hi - lo) + lo


def randomize_motor_delay(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    action_name: str,
    delay_range: tuple[int, int],
) -> None:
    """Randomize the command transport delay of the lag action term per environment.

    Args:
        env: The environment.
        env_ids: Environment indices to randomize.
        action_name: Name of the action term in the action manager.
        delay_range: (min, max) delay [policy steps], both inclusive. Clamped to the
            action term's ``max_delay_steps`` buffer capacity.
    """
    term = env.action_manager.get_term(action_name)
    lo, hi = delay_range
    delays = torch.randint(lo, hi + 1, (len(env_ids),), device=env.device)
    term.delay_steps[env_ids] = torch.clamp(delays, max=term.max_delay_steps)
