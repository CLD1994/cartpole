"""Control-tick endpoint velocity estimate shared by training and evaluation."""
import math

import torch
from isaaclab.managers import ManagerTermBase


class JointVelocityWindow(ManagerTermBase):
    """Ideal position difference over 16 ms, not an extra pure 16 ms delay.

    Repeated observation computations in one environment step do not add samples.
    Reset slots warm up with instantaneous velocity for the first full window.
    非整数控制周期窗口使用两个历史位置线性插值，不是额外纯延迟。
    Noise is applied afterwards by ObservationManager, exactly as in the baseline.
    """

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene['robot']
        if list(self.robot.joint_names) != ['slider_to_cart', 'cart_to_pole']:
            raise ValueError('velocity window requires cart,pole joint ordering')
        self.span = float(cfg.params['window_s'])
        if not math.isfinite(self.span) or self.span < env.step_dt:
            raise ValueError('velocity window must be finite and span at least one control step')
        ratio = self.span / env.step_dt
        nearest = round(ratio)
        if math.isclose(ratio, nearest, abs_tol=1e-9):
            ratio = float(nearest)
        self.whole_steps = math.floor(ratio)
        self.fraction = ratio - self.whole_steps
        self.steps = math.ceil(ratio)
        q = self.robot.data.joint_pos.torch
        self.history = q.clone().unsqueeze(0).repeat(self.steps + 1, 1, 1)
        self.age = torch.zeros(q.shape[0], dtype=torch.long, device=q.device)
        self.pending_reset = torch.ones(q.shape[0], dtype=torch.bool, device=q.device)
        self.head = 0
        self.last_tick = None

    def reset(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        self.pending_reset[ids] = True

    def __call__(self, env, window_s=0.016):
        q = self.robot.data.joint_pos.torch
        tick = env.common_step_counter
        if tick != self.last_tick:
            self.head = (self.head + 1) % (self.steps + 1)
            self.history[self.head] = q
            self.age += 1
            self.last_tick = tick
        # Runs also after an automatic reset in the same tick as a recorder call.
        self.history[:, self.pending_reset] = q[self.pending_reset]
        self.age[self.pending_reset] = 0
        self.pending_reset.zero_()
        recent = (self.head - self.whole_steps) % (self.steps + 1)
        past = self.history[recent]
        if self.fraction:
            older = (recent - 1) % (self.steps + 1)
            past = past * (1.0 - self.fraction) + self.history[older] * self.fraction
        estimate = (self.history[self.head] - past) / self.span
        velocity = torch.where((self.age >= self.steps).unsqueeze(1),
                               estimate, self.robot.data.joint_vel.torch)
        return velocity - self.robot.data.default_joint_vel.torch
