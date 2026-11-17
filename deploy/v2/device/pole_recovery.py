"""摆杆短暂无效时等待新测量；设备故障与失联仍结束会话。"""
from session import Phase
from state_frame import parse_state

class PoleRecoverySessionMixin:
    allow_pole_recovery = True
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pole_waiting = False
        self.pole_wait_at = None
        self.last_state_at = -float('inf')
        self.pole_recovery_stats = {'pauses': 0, 'recoveries': 0, 'invalid_frames': 0,
                                    'maximum_wait_s': 0., 'intentional_completion_gaps': 0}

    def _receive_state(self, line, now):
        frame = parse_state(line)
        self.last_state_at = self._receipt_time(now)
        if self.allow_pole_recovery and self.phase == Phase.ACTIVE and frame.cart is not None and frame.pole is None:
            if not self.pole_waiting:
                self.pole_waiting = True
                self.pole_pause_requested = False
                self.pole_wait_at = self.last_state_at
                self.pole_recovery_stats['pauses'] += 1
            self.pole_recovery_stats['invalid_frames'] += 1
            self._valid = False
            self.observation = None
            return
        super()._receive_state(line, now)

    def _active_measurement_ready(self, now):
        if self.pole_waiting:
            # 无效帧证明通信仍在继续，但不是可供模型推理的测量。
            return self._fresh(self.last_state_at, now, self.limits.state_timeout_s)
        return super()._active_measurement_ready(now)

    def finish_pole_recovery(self, observation_max_age):
        now = self._now()
        if (not self.pole_waiting or self.phase != Phase.ACTIVE or self.failure
                or not self._measurement_ready(now)
                or now-self.observation.received_at >= observation_max_age
                or self.observation.received_at <= self.pole_wait_at):
            return False
        self.pole_recovery_stats['maximum_wait_s'] = max(
            self.pole_recovery_stats['maximum_wait_s'], now-self.pole_wait_at)
        self.pole_recovery_stats['recoveries'] += 1
        self.pole_waiting = False
        return True

    def command(self, action):
        if self.pole_waiting and self.phase == Phase.ACTIVE:
            raise RuntimeError('pole_recovery_pending')
        return super().command(action)
