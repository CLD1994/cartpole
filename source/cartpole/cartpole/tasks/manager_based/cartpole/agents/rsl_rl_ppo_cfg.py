# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils.configclass import configclass
from cartpole.device_profile import load_profile, model_parameters

V2_DT = model_parameters(load_profile())["policy_dt"]

from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg, RslRlSymmetryCfg

from ..mdp import symmetry


@configclass
class CartpolePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 16
    max_iterations = 150
    save_interval = 50
    experiment_name = "cartpole"
    actor = RslRlMLPModelCfg(
        hidden_dims=[32, 32],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = RslRlMLPModelCfg(
        hidden_dims=[32, 32],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class CartpolePPORunnerWithSymmetryCfg(CartpolePPORunnerCfg):
    """Configuration for the PPO agent with symmetry augmentation."""

    # all the other settings are inherited from the parent class
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        symmetry_cfg=RslRlSymmetryCfg(
            use_data_augmentation=True, data_augmentation_func=symmetry.compute_symmetric_states
        ),
    )


@configclass
class RealCartpolePPORunnerCfg(CartpolePPORunnerCfg):
    """PPO configuration for the sim2real (domain-randomized) cartpole task."""

    # domain randomization makes optimization harder -> more iterations
    max_iterations = 1000
    save_interval = 50
    experiment_name = "cartpole_real"


@configclass
class RealCartpoleV2PPORunnerCfg(RealCartpolePPORunnerCfg):
    """PPO configuration for the v2 rig (stepper, velocity-command actions, 500 Hz)."""

    # 120 × 2ms = 240ms，保持原48 × 5ms的rollout时长。
    num_steps_per_env = round(0.240 / V2_DT)
    experiment_name = "cartpole_real_v2"

    def __post_init__(self):
        # 保持单位物理时间的折扣与GAE衰减，不把周期缩短变成长时间视野。
        self.algorithm = self.algorithm.copy()
        self.algorithm.gamma = 0.99 ** (V2_DT / 0.005)
        self.algorithm.lam = 0.95 ** (V2_DT / 0.005)
