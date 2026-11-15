"""双动作参考速度斜坡；速度与加速度共用同一动作延迟。"""
import torch

from .vel_tracking_action import JointVelocityTrackingAction


class JointVelocityAccelerationAction(JointVelocityTrackingAction):
    @property
    def action_dim(self):
        return 2

    def __init__(self, cfg, env):
        if not 0.0 < cfg.acceleration_min <= cfg.acceleration_max <= 9.0:
            raise ValueError("双动作加速度范围必须在 (0, 9] m/s² 内")
        if cfg.scale != 1.0 or cfg.offset != 0.0 or cfg.clip is not None:
            raise ValueError("双动作使用固定归一化映射，不支持额外 scale/offset/clip")
        super().__init__(cfg, env)
        if self._num_joints != 1:
            raise ValueError("双动作只支持一个小车关节")
        # 原始动作/延迟历史是二维；实际关节参考速度始终是一维。
        self._vel_ref = torch.zeros((self.num_envs, 1), device=self.device)

    def apply_actions(self):
        pair = self._delayed_actions.clamp(-1.0, 1.0)
        v_cmd = pair[:, :1] * self.vel_max.unsqueeze(-1)
        requested_acc = self.cfg.acceleration_min + (pair[:, 1:2] + 1.0) * 0.5 * (
            self.cfg.acceleration_max - self.cfg.acceleration_min
        )
        # 保留基线的逐回合能力随机化；请求不能超过执行器能力。
        acc = torch.minimum(requested_acc, self.acc_max.unsqueeze(-1))
        dt = self._env.physics_dt
        k = (dt / (dt + self.tau)).unsqueeze(-1)
        self._vel_ref += torch.clamp((v_cmd - self._vel_ref) * k, -acc * dt, acc * dt)
        v_actual = self._asset.data.joint_vel[:, self._joint_ids]
        effort = torch.clamp(
            self.cfg.servo_gain * (self._vel_ref - v_actual),
            -self.cfg.effort_limit, self.cfg.effort_limit,
        )
        self._asset.set_joint_effort_target_index(target=effort, joint_ids=self._joint_ids)


def velocity_action_rate_l2(env):
    """保持单动作基线奖励：只惩罚速度动作变化，不新增加速度动作惩罚。"""
    return (env.action_manager.action[:, 0] - env.action_manager.prev_action[:, 0]).square()
