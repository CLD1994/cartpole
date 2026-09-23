# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Cartpole environment config for the v2 rig (stepper motor, velocity-command actions).

Reference hardware (42SHDC3025-24B stepper + TMC2209 @ 12 V driving a 2GT 20T pulley 1:1;
AS5600 cart encoder on the motor rear; MT6901 21-bit absolute pole encoder; ESP32-S3 motion
board; 65 g cart, 108 g uniform 0.30 m pole pivoted 20 mm from its short end; +-0.265 m track).
Behavior contract: ``deploy/v2/sim2real-spec.md``; link protocol: ``deploy/v2/protocol.md``.

Key differences vs ``cartpole_env_cfg_sim2real.py`` (the v1 DC-motor rig):
  - action semantics: velocity command + bounded-acceleration tracking (the stepper rig is a
    trajectory executor, not a force-balance system) — the force gain / deadzone / viscous
    damping model of v1 does not apply
  - control rate 200 Hz (physics 400 Hz, decimation 2); NPU inference measured at 0.38 ms max
  - tracking randomization upper bounds are pinned to the firmware session clamps
    (VEL_MAX = 1000 mm/s, ACC_MAX = 5000 mm/s^2): the training distribution must lie entirely
    inside what the rig can execute
"""

import math

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
    """Cartpole config matched to the v2 stepper rig: velocity-command actions, 200 Hz control."""

    def __post_init__(self) -> None:
        super().__post_init__()

        self.episode_length_s = 10.0

        # --- 200 Hz control: physics at 400 Hz with 2 substeps per policy step ---
        self.sim.dt = 1 / 400
        self.decimation = 2
        # render_interval (set to decimation by the base class) stays 2

        # --- action: velocity command + bounded-acceleration tracking (see sim2real-spec.md).
        # The attribute name `joint_effort` is inherited from the base ActionsCfg slot; the term
        # itself is the v2 velocity-tracking action (a JointEffortActionCfg subclass, so the
        # config-class type check passes). ---
        self.actions.joint_effort = mdp_actions.JointVelocityTrackingActionCfg(
            asset_name="robot",
            joint_names=["slider_to_cart"],
            vel_max=0.9,  # nominal authority scale [m/s]; randomized 0.8-1.0 (<= VEL_MAX 1.0)
            tau=0.02,  # nominal tracking time constant [s]; randomized 0.010-0.040
            acc_max=4.0,  # nominal acceleration limit [m/s^2]; randomized 3-5 (<= ACC_MAX 5)
            delay_steps=1,
            max_delay_steps=3,
            servo_gain=40.0,  # N/(m/s): ~4 ms servo time constant, well inside tau
            effort_limit=3.0,  # N: covers 2x m_eff*acc_max + pole gravity reaction
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
            weight=-0.05,
        )

        # --- termination + reset: track +-0.265 m usable, soft limit 0.260, train at +-0.25 m ---
        self.terminations.cart_out_of_bounds = DoneTerm(
            func=joint_pos_out_of_manual_limit,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-0.25, 0.25)},
        )
        self.terminations.pole_out_of_bounds = DoneTerm(
            func=pole_angle_out_of_bounds,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
                "max_angle": math.pi / 3,
            },
        )
        # cart reset: >= 100 mm from the +-0.25 m bounds; at 200 Hz the 1-3 step delay (max
        # 15 ms) coasts at most 4.5 mm at the 0.3 m/s reset velocity, so resets stay recoverable
        self.events.reset_cart_position = EventTerm(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                "position_range": (-0.15, 0.15),
                "velocity_range": (-0.3, 0.3),
            },
        )
        # pole reset: hand-lift start policy — with acc_max 5 m/s^2 the recoverable tilt is
        # roughly +-15-20 deg, so initial angles stay inside +-0.15 rad (±8.6 deg)
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
                "mass_range": {"base_link|cart": (0.0585, 0.0715)},  # 0.065 kg +-10%
            },
        )
        # pole: uniform 0.30 m rod pivoted 20 mm from the short end -> COM 0.130 m from the
        # pivot (NOT length/2). Mass 0.108 kg +-10%, COM offset +-3%, inertia +-15%
        self.events.randomize_pole_properties = EventTerm(
            func=mdp_events.randomize_pole_axis_properties,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "mass_range": (0.097, 0.119),
                "length": 0.30,
                "com_offset_range": (0.1261, 0.1339),
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
        # tracking dynamics (vel_max / tau / acc_max), bounds pinned to the firmware clamps
        self.events.randomize_tracking = EventTerm(
            func=mdp_events.randomize_velocity_tracking,
            mode="reset",
            params={
                "action_name": "joint_effort",
                "vel_max_range": (0.8, 1.0),
                "tau_range": (0.010, 0.040),
                "acc_max_range": (3.0, 5.0),
            },
        )
        # transport delay of the real loop (200 Hz stream + serial + inference): 1-3 policy
        # steps at 200 Hz = 5-15 ms; the policy must learn to act on stale state
        self.events.randomize_motor_delay = EventTerm(
            func=mdp_events.randomize_motor_delay,
            mode="reset",
            params={"action_name": "joint_effort", "delay_range": (1, 3)},
        )
