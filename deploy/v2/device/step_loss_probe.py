"""临时现场事件探针；只依据收到的固件事件，不自动推断机械停转。"""
from collections import deque
from session import Phase


class EventWaitTimeout(RuntimeError):
    pass


class StepLossProbe:
    def __init__(self):
        self.expected_run_id = None
        self.wait_started_at = None
        self.events = deque(maxlen=128)
        self.highest_event_id = 0
        self.highest_device_us = 0
        self.trigger = None
        self.first_probe_write_clock_s = None
        self.first_probe_completion_clock_s = None
        self.first_zero_write_clock_s = None
        self.first_zero_completion_clock_s = None
        self.discarded_model_candidates = 0

    def receive(self, line, now, active):
        words = line.split()
        if (len(words) != 5 or words[:2] != ['EVENT', 'STEP_LOSS']
                or any(not word or any(c not in '0123456789' for c in word)
                       for word in words[2:])):
            raise ValueError('malformed_step_loss_event')
        event, run, device_us = map(int, words[2:])
        if not (0 < event <= 2**32-1 and 0 < run <= 2**32-1 and 0 <= device_us <= 2**63-1):
            raise ValueError('malformed_step_loss_event')
        fresh = event > self.highest_event_id and device_us >= self.highest_device_us
        row = {'event_id': event, 'run_id': run, 'device_us': device_us,
               'host_received_clock_s': now, 'received_while_active': active,
               'disposition': 'duplicate_or_older'}
        if self.expected_run_id is None or run != self.expected_run_id:
            row['disposition'] = 'unverified_or_other_run'
            self.events.append(row)
            return
        if active:
            self.check_wait(now)
        if fresh:
            self.highest_event_id = event
            self.highest_device_us = device_us
            row['disposition'] = 'startup_baseline' if not active else 'after_first_trigger'
            if active and self.trigger is None:
                row['disposition'] = 'trigger'
                self.trigger = dict(row)
        self.events.append(row)

    def check_wait(self, now):
        if (self.trigger is None and self.wait_started_at is not None
                and now >= self.wait_started_at + 120.):
            raise EventWaitTimeout('event_wait_timeout')

    def phase(self, now):
        if self.trigger is None:
            return 'model'
        if self.first_zero_completion_clock_s is not None:
            return 'done' if now >= self.first_zero_completion_clock_s + 2. else 'zero'
        if self.first_probe_write_clock_s is not None and now >= self.first_probe_write_clock_s + 5.:
            return 'zero'
        return 'probe'

    def before_write(self, now):
        phase = self.phase(now)
        if phase == 'model':
            raise RuntimeError('probe_not_triggered')
        if phase == 'probe':
            if self.first_probe_write_clock_s is None:
                self.first_probe_write_clock_s = now
            return 10
        if self.first_zero_write_clock_s is None:
            self.first_zero_write_clock_s = now
        return 0

    def completed(self, velocity, now):
        if velocity == 10 and self.first_probe_completion_clock_s is None:
            self.first_probe_completion_clock_s = now
        if velocity == 0 and self.first_zero_completion_clock_s is None:
            self.first_zero_completion_clock_s = now

    def report(self):
        return {key: (list(value) if isinstance(value, deque) else value)
                for key, value in vars(self).items()}


class StepLossEventMixin:
    """只在显式事件探针会话启用；其余消息使用原状态机及失败停车。"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.step_loss_probe = StepLossProbe()
        self.run_query_pending = False
        self.run_reply = None

    def receive(self, line):
        if line.split()[:1] == ['RUN']:
            words = line.split()
            if (len(words) != 2 or not words[1].isascii() or not words[1].isdigit()
                    or not 0 < int(words[1]) <= 2**32-1):
                raise RuntimeError('malformed_run_reply')
            if self.run_query_pending:
                self.run_reply = int(words[1])
                self.step_loss_probe.expected_run_id = self.run_reply
                self.run_query_pending = False
            return
        if line.split()[:1] != ['EVENT']:
            return super().receive(line)
        self.tick()
        now = self._now()
        try:
            self.step_loss_probe.receive(line, now, self.phase == Phase.ACTIVE)
        except ValueError:
            self._fail('malformed_step_loss_event', now)
            raise RuntimeError('malformed_step_loss_event')
