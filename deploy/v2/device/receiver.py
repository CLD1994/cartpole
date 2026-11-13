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
                or not 0 < self.max_line <= self.max_pending
                or any(not math.isfinite(v) or v <= 0 for v in
                       (self.max_poll_s, self.max_read_s, self.partial_timeout_s))):
            raise ValueError('invalid_receive_limits')


class Receiver:
    """source必须独占且read非阻塞；clock须与session共享单调时间轴。

    失败后可继续poll推进有界停机；Session的致命异常（如时钟无效）传播。"""
    def __init__(self, source: ByteSource, session, clock, limits=ReceiveLimits()):
        self.source, self.session, self.clock, self.limits = source, session, clock, limits
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

    def poll(self):
        """至多一次读操作，返回本批是否有效；error锁存接收异常，致命会话异常传播。"""
        self.session.tick()
        now = self.clock()
        if (not math.isfinite(now) or (self._last_poll is not None and
                not 0 <= now - self._last_poll <= self.limits.max_poll_s)):
            self._last_poll = now
            return self._fail('poll_deadline')
        self._last_poll = now
        if self._partial_since is not None and now-self._partial_since > self.limits.partial_timeout_s:
            return self._fail('partial_timeout')
        try:
            count = self.source.in_waiting
            if type(count) is not int or not 0 <= count <= self.limits.max_pending:
                return self._fail('backlog')
            data = self.source.read(count) if count else b''
            remaining = self.source.in_waiting
        except (OSError, ValueError):
            return self._fail('read_error')
        end = self.clock()
        self.session.tick()
        if not math.isfinite(end) or not 0 <= end-now <= self.limits.max_read_s:
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
