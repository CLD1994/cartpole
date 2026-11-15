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
