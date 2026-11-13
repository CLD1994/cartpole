# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Cartpole environment config for the v2 rig (stepper motor, velocity-command actions).

本任务使用已记录的设备几何与固定500Hz策略基线。执行器动态仍是假设，
450–500Hz策略节拍波动及固件观测滤波尚未建模，不能直接部署到真机。
"""

import math

from cartpole.device_profile import load_profile, model_parameters

DEVICE = model_parameters(load_profile())

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass
from isaaclab.utils.noise import GaussianNoiseCfg

from .cartpole_env_cfg import CartpoleEnvCfg
from .cartpole_env_cfg_sim2real import gaussian_noise_from_list
from .mdp import (
    action_rate_l2,
    joint_pos_out_of_manual_limit,
    joint_pos_target_l2,
    reset_joints_by_offset,
)
from .mdp import dr_events as mdp_events
from .mdp import vel_tracking_actions as mdp_actions
from .mdp.terminations import pole_angle_out_of_bounds


@configclass
class RealCartpoleV2EnvCfg(CartpoleEnvCfg):
    """Cartpole config matched to the v2 stepper rig: velocity-command actions, 500 Hz control."""

    def __post_init__(self) -> None:
        super().__post_init__()

        self.episode_length_s = 10.0

        # 固定500Hz基线，物理积分1ms；不以动作延迟冒充策略频率波动。
        self.sim.dt = DEVICE["policy_dt"] / 2
        self.decimation = 2
        self.rewards.terminating.weight *= 0.005 / DEVICE["policy_dt"]
        # render_interval (set to decimation by the base class) stays 2

        # --- action: velocity command + bounded-acceleration tracking (see sim2real-spec.md).
        # The attribute name `joint_effort` is inherited from the base ActionsCfg slot; the term
        # itself is the v2 velocity-tracking action (a JointEffortActionCfg subclass, so the
        # config-class type check passes). ---
        self.actions.joint_effort = mdp_actions.JointVelocityTrackingActionCfg(
            asset_name="robot",
            joint_names=["slider_to_cart"],
            vel_max=DEVICE["velocity_m_s"],  # 配置目标，不是已验证全工况上限
            tau=0.02,  # nominal tracking time constant [s]; randomized 0.010-0.040
            acc_max=DEVICE["acceleration_m_s2"],
            delay_steps=3,
            max_delay_steps=8,
            servo_gain=40.0,  # N/(m/s)，未辨识的速度伺服增益
            effort_limit=8.0,  # 未辨识的仿真伺服力上限，不是电机测得推力
        )

        # --- observations: noise matched to the v2 sensors per sim2real-spec.md.
        # joint order for positions: [cart (m), pole (rad)]; for velocities: [m/s, rad/s].
        # x: sigma 0.05 mm (AS5600 9.77 um quantization + multiturn accumulation, relaxed);
        # theta: sigma 1 mrad (MT6901 3 urad resolution, relaxed to magnetic-encoder noise);
        # vx: sigma 5 mm/s, vth: sigma 10 mrad/s (firmware velocity-estimator targets). ---
        self.observations.policy.joint_pos_rel.noise = GaussianNoiseCfg(
            func=gaussian_noise_from_list, mean=0.0, std=[5.0e-5, 1.0e-3]
        )
        self.observations.policy.joint_vel_rel.noise = GaussianNoiseCfg(
            func=gaussian_noise_from_list, mean=0.0, std=[5.0e-3, 1.0e-2]
        )
        self.observations.policy.enable_corruption = True

        # --- reward: keep the cart near the track center (inherited shaping from v1) ---
        self.rewards.cart_pos = RewTerm(
            func=joint_pos_target_l2,
            weight=-0.5,
            # wrap=False: slider_to_cart is prismatic, its position [m] must not be angle-wrapped
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                "target": 0.0,
                "wrap": False,
            },
        )
        # --- reward: penalize velocity-command chattering (smooth commands on the stepper rig
        # avoid wasted reversals through zero speed and StallGuard transients) ---
        self.rewards.action_rate = RewTerm(
            func=action_rate_l2,
            weight=-0.05 * (0.005 / DEVICE["policy_dt"]) ** 2,
        )

        # 终止边界比软限位内缩130mm；仿真终止不等于物理制动。
        train_limit = DEVICE["soft_limit_m"] - 0.13
        self.terminations.cart_out_of_bounds = DoneTerm(
            func=joint_pos_out_of_manual_limit,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-train_limit, train_limit)},
        )
        self.terminations.pole_out_of_bounds = DoneTerm(
            func=pole_angle_out_of_bounds,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
                "max_angle": math.pi / 3,
            },
        )
        # 初始位置保留在轨道中部，避免从终止边界附近初始化。
        self.events.reset_cart_position = EventTerm(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                "position_range": (-0.15, 0.15),
                "velocity_range": (-0.3, 0.3),
            },
        )
        # 手扶起摆任务，初始角限制在±0.15rad；不宣称全角度可恢复。
        self.events.reset_pole_position = EventTerm(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
                "position_range": (-0.15, 0.15),
                "velocity_range": (-0.5, 0.5),
            },
        )

        # --- domain randomization (per episode, centered on the v2 hardware values) ---
        self.events.randomize_body_mass = EventTerm(
            func=mdp_events.randomize_body_mass,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "mass_range": {"base_link|cart": (DEVICE["cart_mass_kg"] * (1 - DEVICE["cart_mass_randomization_fraction"]), DEVICE["cart_mass_kg"] * (1 + DEVICE["cart_mass_randomization_fraction"]))},  # 0.065 kg +-10%
            },
        )
        # 端点均匀细杆；质量±10%、质心±3%、惯量±15%为初始随机化假设。
        self.events.randomize_pole_properties = EventTerm(
            func=mdp_events.randomize_pole_axis_properties,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "mass_range": (DEVICE["pole_mass_kg"] * 0.9, DEVICE["pole_mass_kg"] * 1.1),
                "length": DEVICE["pole_length_m"],
                "com_offset_range": (DEVICE["pole_com_m"] * 0.97, DEVICE["pole_com_m"] * 1.03),
                "inertia_scale_range": (0.85, 1.15),
            },
        )
        # the stepper rig's residual rail/belt friction: small, mostly folded into the tracking
        # tau; kept small rather than v1's large viscous-friction range (no DC-motor back-EMF)
        self.events.randomize_joint_damping = EventTerm(
            func=mdp_events.randomize_joint_damping,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "damping_range": {"slider_to_cart": (0.0, 2.0), "cart_to_pole": (0.0, 0.01)},
            },
        )
        # 参考跟踪动态范围；当前装置的速度、加速度与响应参数需要实测辨识。
        self.events.randomize_tracking = EventTerm(
            func=mdp_events.randomize_velocity_tracking,
            mode="reset",
            params={
                "action_name": "joint_effort",
                "vel_max_range": (DEVICE["velocity_m_s"], DEVICE["velocity_m_s"]),
                "tau_range": (0.010, 0.040),
                "acc_max_range": (8.0, DEVICE["acceleration_m_s2"]),
            },
        )
        # 命令传输延迟假设6–16ms；不是观测速度滤波，也不是策略频率波动。
        self.events.randomize_motor_delay = EventTerm(
            func=mdp_events.randomize_motor_delay,
            mode="reset",
            params={"action_name": "joint_effort", "delay_range": (3, 8)},
        )
