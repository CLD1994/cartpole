# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Cartpole environment config tuned for sim2real transfer to a real cartpole rig.

Reference hardware (MC520P10_12V gearmotor, 1:20, driving a 2GT-60T pulley; 52-PPR encoder on the
motor shaft; 150 g cart; 40 g / 30 cm pole; potentiometer pole-angle sensor). Dynamics numbers
below are MEASURED with deploy/calibrate_force.py (2026-08-18), not datasheet guesses:
  - action scale : 100 N -> 3.3 N  (PWM->force gain G = 0.0033 N/unit at full scale P1000)
  - deadzone     : 0.5 N           (breakaway measured 0.26-0.50 N depending on direction)
  - motor lag    : first-order, T = m/c ~ 17 ms (randomized 8-25 ms)
  - transport delay: 1-3 policy steps (sensor streaming + serial + inference), randomized
  - masses       : cart 0.15 kg, pole 0.04 kg, randomized +/- 30%
  - joint damping: cart 8-11 N*s/m measured (randomized 5-15), pole ~ 0
  - obs noise    : encoder quantization + finite-difference velocity noise
"""

import torch

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils.configclass import configclass
from isaaclab.utils.noise import GaussianNoiseCfg

from .cartpole_env_cfg import CartpoleEnvCfg
from .mdp import (
    action_rate_l2,
    joint_pos_out_of_manual_limit,
    joint_pos_target_l2,
    reset_joints_by_offset,
)
from .mdp import dr_events as mdp_events
from .mdp import lag_actions as mdp_actions


def gaussian_noise_from_list(data: torch.Tensor, cfg: GaussianNoiseCfg) -> torch.Tensor:
    """Add Gaussian noise with ``std`` given as a list.

    OmegaConf (used by the play/hydra path) cannot serialize a ``torch.Tensor``, so ``std`` is
    stored as a plain list and converted here at application time.
    """
    std = torch.as_tensor(cfg.std, device=data.device)
    return data + cfg.mean + std * torch.randn_like(data)


@configclass
class RealCartpoleEnvCfg(CartpoleEnvCfg):
    """Cartpole config with dynamics matched to the real rig and domain randomization."""

    def __post_init__(self) -> None:
        super().__post_init__()

        # the real rig is imprecise (sensor noise + stiction) and settles slowly, so give each
        # episode more time before the time-out reset — otherwise the policy is reset while still
        # oscillating toward a stable state and never gets rewarded for reaching it.
        self.episode_length_s = 10.0

        # --- action: effort, scaled to the real motor's force capability, with motor response lag ---
        self.actions.joint_effort = mdp_actions.JointEffortLagActionCfg(
            asset_name="robot",
            joint_names=["slider_to_cart"],
            scale=3.3,  # measured PWM->force gain 0.0033 N/unit at full scale (P1000)
            tau=0.015,  # measured T = m/c ~ 17 ms; overwritten per-episode by randomize_motor_lag
            # command transport delay [policy steps]: sensor streaming + serial link + inference
            # on the real loop sum to 1-3 steps at 60 Hz; randomized by randomize_motor_delay
            delay_steps=1,
            max_delay_steps=3,
            deadzone=0.5,  # measured breakaway: 0.26-0.50 N depending on direction and spot
        )

        # --- observations: encoder quantization + finite-difference velocity noise ---
        # joint order for positions: [cart (m), pole (rad)]; for velocities: [m/s, rad/s]
        # Measured ADC noise (R=37 counts, RAD_PER_COUNT=0.0002) → pos_std ≈ 1.85e-3 rad ≈ the base
        # 2e-3, so position noise stays at base. Vel is kept near base (not the 0.16 white-noise
        # bound) so the policy keeps velocity/damping feedback to fight the stiction limit cycle.
        self.observations.policy.joint_pos_rel.noise = GaussianNoiseCfg(
            func=gaussian_noise_from_list, mean=0.0, std=[2.0e-4, 2.0e-3]
        )
        self.observations.policy.joint_vel_rel.noise = GaussianNoiseCfg(
            func=gaussian_noise_from_list, mean=0.0, std=[2.0e-2, 1.0e-1]
        )
        self.observations.policy.enable_corruption = True

        # --- reward: keep the cart near the track center. The default reward has no cart-position
        # term, so under the DR dynamics the policy balances the pole but drifts to the ±3 m bounds ---
        self.rewards.cart_pos = RewTerm(
            func=joint_pos_target_l2,
            weight=-0.5,
            # wrap=False: slider_to_cart is prismatic, its position [m] must not be
            # angle-wrapped (harmless within the +-0.173 m track, wrong for longer rails)
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                "target": 0.0,
                "wrap": False,
            },
        )
        # --- reward: penalize action chattering. With the stiction deadzone the DR policy converges
        # to bang-bang; on the real rig (transport delay + motor lag) that becomes a sustained
        # limit cycle. Penalizing the command rate pushes the policy to smooth, low-gain commands.
        # action_rate_l2() takes no params: it uses the full (single-term) action vector ---
        self.rewards.action_rate = RewTerm(
            func=action_rate_l2,
            weight=-0.05,
        )

        # --- termination + reset: match the real track length (±0.173 m) instead of the default ±3 m ---
        self.terminations.cart_out_of_bounds = DoneTerm(
            func=joint_pos_out_of_manual_limit,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-0.173, 0.173)},
        )
        self.events.reset_cart_position = EventTerm(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                # start >= 73 mm from the +-0.173 m bounds: with the 1-3 step transport delay a
                # 0.25 m/s reset velocity coasts at most 12 mm before the command can act, so no
                # reset is unrecoverable (at 0.15 m / 0.5 m/s the worst resets were doomed)
                "position_range": (-0.10, 0.10),
                "velocity_range": (-0.25, 0.25),
            },
        )
        # the base task resets the pole to +-45 deg, which needs ~0.5-1 m of cart travel to
        # recover - impossible on the +-0.173 m real track. Those episodes are unsavable and
        # end as cart_out_of_bounds; reset to a recoverable band instead.
        self.events.reset_pole_position = EventTerm(
            func=reset_joints_by_offset,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
                "position_range": (-0.15, 0.15),
                "velocity_range": (-0.5, 0.5),
            },
        )

        # --- domain randomization (per episode, centered on the real hardware values) ---
        self.events.randomize_body_mass = EventTerm(
            func=mdp_events.randomize_body_mass,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "mass_range": {"base_link|cart": (0.13, 0.17)},
            },
        )
        # the USD pole has ~280x the real rod's inertia and a 3x higher COM, which made balancing
        # far harder than reality; override its mass + COM + inertia to a uniform 0.30 m rod
        self.events.randomize_pole_properties = EventTerm(
            func=mdp_events.randomize_pole_properties,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "mass_range": (0.034, 0.046),
                "length": 0.30,
            },
        )
        self.events.randomize_joint_damping = EventTerm(
            func=mdp_events.randomize_joint_damping,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                # measured 8-11 N*s/m (steady-state slope method); randomized wider for the sticky zones
                "damping_range": {"slider_to_cart": (5.0, 15.0), "cart_to_pole": (0.0, 0.01)},
            },
        )
        self.events.randomize_motor_lag = EventTerm(
            func=mdp_events.randomize_motor_lag,
            mode="reset",
            params={"action_name": "joint_effort", "tau_range": (0.008, 0.025)},  # measured T ~ 17 ms
        )
        # transport delay of the real loop (5 ms sensor window + serial + 60 Hz inference):
        # 1-3 policy steps at 60 Hz; the policy must learn to act on stale state
        self.events.randomize_motor_delay = EventTerm(
            func=mdp_events.randomize_motor_delay,
            mode="reset",
            params={"action_name": "joint_effort", "delay_range": (1, 3)},
        )
