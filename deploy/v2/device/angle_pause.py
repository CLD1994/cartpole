"""摆角滞回暂停；不发送命令，不改变设备会话状态。"""
import math

class AnglePause:
    def __init__(self, enter_deg=60., resume_deg=58.):
        if not (math.isfinite(enter_deg) and math.isfinite(resume_deg)
                and 0 <= resume_deg < enter_deg <= 180):
            raise ValueError('invalid_angle_pause_thresholds')
        self.enter = math.radians(enter_deg)
        self.resume = math.radians(resume_deg)
        self.paused = False
        self.resumed_since_command = False

    def check(self, observation, now, budget_at, hard_limit, observation_limit):
        # 既有活动超时不能因本次大摆角而被豁免。
        if not self.paused and now-budget_at >= hard_limit:
            raise RuntimeError('command_gap_deadline')
        if observation is None:
            if self.paused:
                raise RuntimeError('angle_pause_observation_deadline')
            return None  # 非暂停的缺测由原运行器处理。
        angle = abs(observation.si()[1])
        if not math.isfinite(angle):
            raise RuntimeError('invalid_angle')
        if (self.paused or angle >= self.enter) and now-observation.received_at >= observation_limit:
            raise RuntimeError('angle_pause_observation_deadline')
        if self.paused and angle <= self.resume:
            self.paused = False
            self.resumed_since_command = True
            return 'resumed'
        if not self.paused and angle >= self.enter:
            self.paused = True
            return 'paused'
        return None
