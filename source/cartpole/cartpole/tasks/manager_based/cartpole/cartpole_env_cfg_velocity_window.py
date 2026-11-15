"""Single-factor 16 ms velocity-observation adaptation; original task unchanged."""
from isaaclab.utils.configclass import configclass
from .cartpole_env_cfg_sim2real_v2 import RealCartpoleV2EnvCfg
from .mdp.velocity_window_observation import JointVelocityWindow


@configclass
class RealCartpoleV2Window16EnvCfg(RealCartpoleV2EnvCfg):
    def __post_init__(self):
        super().__post_init__()
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
