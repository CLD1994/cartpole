"""可选只读事件记录；不改变模型动作，不推断机械停转。"""
from collections import deque
from queue import Queue, Full
from threading import Thread
from session import Phase


class StepLossProbe:
    def __init__(self):
        self.monitor_only = False
        self.pre_event_samples = deque(maxlen=400)
        self.event_window = []
        self.post_event_samples = 0
        self.expected_run_id = None
        self.wait_started_at = None
        self.events = deque(maxlen=128)
        self.highest_event_id = 0
        self.highest_device_us = 0
        self.trigger = None

    def enable_monitor(self):
        self.monitor_only = True
        self._notice_queue = Queue(maxsize=1)
        def output_once():
            message = self._notice_queue.get()
            try:
                print(message, flush=True)
            except Exception:
                pass  # console delivery is optional; report keeps authoritative event
        Thread(target=output_once, daemon=True, name='step-loss-notice').start()

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
        if fresh:
            self.highest_event_id = event
            self.highest_device_us = device_us
            row['disposition'] = 'startup_baseline' if not active else 'after_first_trigger'
            if active and self.trigger is None:
                row['disposition'] = 'trigger'
                self.trigger = dict(row)
                if self.monitor_only:
                    self.event_window = list(self.pre_event_samples)
                    if hasattr(self, '_notice_queue'):
                        try:
                            self._notice_queue.put_nowait(f'STEP_LOSS event={event} run={run} device_us={device_us} host_s={now:.6f}：仅记录；是否继续控制取决于模式和时序门控')
                        except Full:
                            pass
        self.events.append(row)

    def record_sample(self, sample):
        if not self.monitor_only:
            return
        row = dict(sample)
        if self.trigger is None:
            self.pre_event_samples.append(row)
        elif self.post_event_samples < 2000:
            self.event_window.append(row)
            self.post_event_samples += 1

    def report(self):
        return {key: (list(value) if isinstance(value, deque) else value)
                for key, value in vars(self).items() if not key.startswith('_')}


class StepLossEventMixin:
    """只在显式事件记录会话启用；其余消息使用原状态机及失败停车。"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.step_loss_probe = StepLossProbe()
        self.run_query_pending = False
        self.run_query_sent_at = float('-inf')
        self.run_reply = None

    def receive(self, line):
        words = line.split()
        if words[:2] == ['OK', 'RUN']:
            if (len(words) != 3 or not words[2].isascii() or not words[2].isdigit()
                    or not 0 < int(words[2]) <= 2**32-1):
                self._fail('malformed_run_reply', self._now())
                raise RuntimeError('malformed_run_reply')
            if self.run_query_pending and self._receipt_time(self._now()) >= self.run_query_sent_at:
                self.run_reply = int(words[2])
                self.step_loss_probe.expected_run_id = self.run_reply
                self.run_query_pending = False
            return
        if line.split()[:1] != ['EVENT']:
            return super().receive(line)
        self.tick()
        now = self._now()
        try:
            self.step_loss_probe.receive(line, self._receipt_time(now), self.phase == Phase.ACTIVE)
        except ValueError:
            self._fail('malformed_step_loss_event', now)
            raise RuntimeError('malformed_step_loss_event')

