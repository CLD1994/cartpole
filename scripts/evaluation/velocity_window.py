"""Evaluation-only endpoint velocity approximation; never changes physics/rewards."""
import math


class VelocityWindow:
    def __init__(self, env, window_s):
        import torch
        self.torch, self.env = torch, env
        self.robot = env.scene['robot']
        if list(self.robot.joint_names) != ['slider_to_cart', 'cart_to_pole']:
            raise ValueError('velocity-window requires cart,pole joint/observation ordering')
        self.dt = float(env.step_dt)
        self.steps = round(window_s / self.dt)
        if self.steps < 1 or not math.isclose(self.steps*self.dt, window_s, abs_tol=1e-9):
            raise ValueError('window must be an integer number of control steps')
        self.span = self.steps*self.dt
        q = self.robot.data.joint_pos.torch
        self.history = q.clone().unsqueeze(0).repeat(self.steps+1, 1, 1)
        self.age = torch.zeros(q.shape[0], dtype=torch.long, device=q.device)
        self.head = 0

    def reset(self, mask=None):
        q = self.robot.data.joint_pos.torch
        if mask is None:
            self.history[:] = q
            self.age.zero_()
            self.head = 0
        else:
            self.history[:, mask] = q[mask]
            self.age[mask] = 0

    def advance(self, dones):
        self.head = (self.head+1) % (self.steps+1)
        self.history[self.head] = self.robot.data.joint_pos.torch
        self.age += 1
        # Auto-reset slots must never form differences across episode boundaries.
        self.reset(dones.bool())

    def apply(self, obs):
        # Preserve the exact observation/noise already produced by Isaac Lab.
        # Change only velocity channels by estimated minus noiseless true velocity.
        values = obs['policy']
        if values.shape != self.robot.data.joint_pos.torch.shape[:-1]+(4,):
            raise ValueError('velocity-window requires four policy observations')
        oldest = (self.head+1) % (self.steps+1)
        estimate = (self.history[self.head]-self.history[oldest])/self.span
        true_velocity = self.robot.data.joint_vel.torch
        correction = self.torch.where((self.age >= self.steps).unsqueeze(1),
                                      estimate-true_velocity, self.torch.zeros_like(estimate))
        result = obs.clone()
        policy = values.clone()
        policy[:, 2:4] += correction
        result['policy'] = policy
        return result
