"""Optional balance recipe with 16 ms velocity observations; original task unchanged."""
from isaaclab.utils.configclass import configclass
from .cartpole_env_cfg_sim2real_v2 import RealCartpoleV2EnvCfg
from .mdp.velocity_window_observation import JointVelocityWindow
from isaaclab.managers import RewardTermCfg as RewTerm, SceneEntityCfg
from .mdp import joint_boundary_l2


@configclass
class RealCartpoleV2Window16EnvCfg(RealCartpoleV2EnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # 新配方允许轨道内部任意位置平衡；旧 V2 仍保留回中惩罚。
        self.rewards.cart_pos = None
        self.rewards.cart_vel.weight = -0.1
        train_limit = self.terminations.cart_out_of_bounds.params["bounds"][1]
        self.rewards.cart_boundary = RewTerm(
            func=joint_boundary_l2, weight=-0.5,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
                    "free_radius": 0.7 * train_limit, "limit": train_limit},
        )
        self.observations.policy.joint_vel_rel.func = JointVelocityWindow
        self.observations.policy.joint_vel_rel.params = {'window_s': 0.016}


@configclass
class RealCartpoleV2Window16Tau0EnvCfg(RealCartpoleV2Window16EnvCfg):
    """窗口观测对照：仅移除参考速度的一阶滤波，保留限加速度等全部参数。"""
    def __post_init__(self):
        super().__post_init__()
        self.actions.joint_effort.tau = 0.0
        # 重置事件必须同步固定为零，否则每个回合会重新抽样 10–40 ms。
        self.events.randomize_tracking.params['tau_range'] = (0.0, 0.0)


@configclass
class RealCartpoleV2Window16Tau0Acc9Hz400EnvCfg(RealCartpoleV2Window16Tau0EnvCfg):
    """400 Hz 策略、9 m/s² 参考加速度上限；保留原 PPO 和奖励参数。"""

    def __post_init__(self):
        super().__post_init__()
        # PhysX 每个 0.5 ms 子步更新参考速度；5 个子步执行一次策略。
        self.sim.dt = 0.0005
        self.decimation = 5
        self.sim.render_interval = self.decimation
        self.actions.joint_effort.acc_max = 9.0
        # reset 随机化也必须遵守新上限，不能重新抽到 9–10 m/s²。
        self.events.randomize_tracking.params['acc_max_range'] = (8.0, 9.0)
