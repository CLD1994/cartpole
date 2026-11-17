"""有界串口接收；不保证设备采样新鲜度，不自动恢复控制。"""
from dataclasses import dataclass
import math
from typing import Protocol


class ByteSource(Protocol):
    @property
    def in_waiting(self) -> int: ...
    def read(self, size: int) -> bytes: ...


@dataclass(frozen=True)
class ReceiveLimits:
    max_pending: int = 1024
    max_line: int = 256
    max_poll_s: float = .02
    max_read_s: float = .01
    partial_timeout_s: float = .02

    def __post_init__(self):
        if (type(self.max_pending) is not int or type(self.max_line) is not int
                or not 0 < self.max_line <= self.max_pending <= 1048576
                or any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in
                       (self.max_poll_s, self.max_read_s, self.partial_timeout_s))):
            raise ValueError('invalid_receive_limits')


class Receiver:
    """source必须独占且read非阻塞；clock须与session共享单调时间轴。

    失败后可继续poll推进有界停机；Session的致命异常（如时钟无效）传播。"""
    def __init__(self, source: ByteSource, session, clock, limits=ReceiveLimits(), *, timeline=None, slow_window=None, hard_window_s=.1):
        self.source, self.session, self.clock, self.limits = source, session, clock, limits
        self.timeline = timeline
        self.slow_window = slow_window
        self.hard_window_s = hard_window_s
        if not math.isfinite(hard_window_s) or hard_window_s <= 0:
            raise ValueError("invalid_hard_window")
        self.error = None
        self._desync = False
        self._pending = b''
        self._partial_since = None
        self._last_poll = None

    def _fail(self, reason):
        self.error = self.error or reason
        self._desync = True
        self._pending = b''
        self._partial_since = None
        self.session.abort()
        return False

    def _window_ok(self, reason, elapsed, soft):
        if not math.isfinite(elapsed) or elapsed < 0:
            return False
        if elapsed <= soft:
            return True
        if self.slow_window is None or elapsed >= self.hard_window_s:
            return False
        self.slow_window(reason, elapsed)
        return True

    def poll(self, *, previous_poll_limit_s=None):
        """至多一次读操作，返回本批是否有效；error锁存接收异常，致命会话异常传播。"""
        self.session.tick()
        now = self.clock()
        elapsed = None if self._last_poll is None else now-self._last_poll
        poll_limit = self.limits.max_poll_s if previous_poll_limit_s is None else previous_poll_limit_s
        self._last_poll = now
        if not math.isfinite(now) or (elapsed is not None and
                not self._window_ok('poll_deadline', elapsed, poll_limit)):
            return self._fail('poll_deadline')
        if self._partial_since is not None and now-self._partial_since > self.limits.partial_timeout_s:
            return self._fail('partial_timeout')
        try:
            if self.timeline:
                self.timeline.mark('receiver_count_call')
            count = self.source.in_waiting
            if self.timeline:
                self.timeline.mark('receiver_count_return', io={'count':count})
            if type(count) is not int or not 0 <= count <= self.limits.max_pending:
                return self._fail('backlog')
            if count:
                if self.timeline:
                    self.timeline.mark('receiver_read_call', io={'count':count})
                data = self.source.read(count)
                if self.timeline:
                    self.timeline.mark('receiver_read_return', io={'read_bytes':len(data) if isinstance(data,bytes) else None})
            else:
                data = b''
                if self.timeline:
                    self.timeline.mark('receiver_read_skipped', io={'count':count})
            if self.timeline:
                self.timeline.mark('receiver_remaining_call')
            remaining = self.source.in_waiting
        except (OSError, ValueError):
            return self._fail('read_error')
        end = self.clock()
        if self.timeline:
            self.timeline.mark('receiver_remaining_return', io={'remaining':remaining, 'read_window_s':end-now})
        self.session.tick()
        if not math.isfinite(end) or not self._window_ok("read_deadline", end-now, self.limits.max_read_s):
            return self._fail('read_deadline')
        if self._partial_since is not None and end-self._partial_since > self.limits.partial_timeout_s:
            return self._fail('partial_timeout')
        if type(remaining) is not int or not 0 <= remaining <= self.limits.max_pending:
            return self._fail('backlog')
        if not isinstance(data, bytes) or len(data) > count or (count and not data):
            return self._fail('read_contract')
        combined = self._pending + data
        parts = combined.split(b'\n')
        if any(len(p) > self.limits.max_line for p in parts):
            return self._fail('line_overflow')
        if any(c > 126 or (c < 32 and c not in (10, 13)) for c in combined):
            return self._fail('invalid_ascii')
        if self._desync:
            if len(parts) == 1:
                return True
            parts = parts[1:]
            self._desync = False
        old_partial = bool(self._pending)
        self._pending = parts[-1]
        if not self._pending:
            self._partial_since = None
        elif not old_partial or len(parts) > 1:
            self._partial_since = now
        for line in parts[:-1]:
            self.session.receive(line.decode('ascii').rstrip('\r'))
        return True
