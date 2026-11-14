"""设备会话的公开状态机；I/O 和单调时钟由调用方提供。不会自动使能。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Callable, Protocol


class Transport(Protocol):
    def write(self, data: bytes) -> int: ...


class Phase(Enum):
    OBSERVING = 'observing'
    ACTIVE = 'active'
    STOPPING = 'stopping'
    FINISHED = 'finished'


@dataclass(frozen=True)
class Limits:
    control_hz: float
    speed_mm_s: int
    acceleration_mm_s2: int
    margin_um: int
    watchdog_ms: int = 100
    watchdog_decel: int = 5000
    state_timeout_s: float = .1
    validity_timeout_s: float = .3
    preflight_timeout_s: float = 1.
    stop_timeout_s: float = 1.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('会话限制必须是有限正数')

    @property
    def config(self):
        return {'VEL_MAX': self.speed_mm_s, 'ACC_MAX': self.acceleration_mm_s2,
                'WDT_MS': self.watchdog_ms, 'WDT_DEC': self.watchdog_decel,
                'X_MARGIN_UM': self.margin_um}


@dataclass(frozen=True)
class Observation:
    x_um: int
    theta_urad: int
    velocity_mm_s: int
    angular_velocity_mrad_s: int
    received_at: float

    def si(self):
        """固件坐标系；不隐式翻转角度、不冒充策略的坐标系。"""
        return (self.x_um / 1e6, self.theta_urad / 1e6,
                self.velocity_mm_s / 1e3, self.angular_velocity_mrad_s / 1e3)


class Session:
    """一次显式控制会话。失败锁存，只能新建会话并重新预检后再启动。

    receive 接受完整 ASCII 行，tick 必须被调用以检查静默超时和停机截止。
    发送协作者必须具有有界写超时；这里不能中断阻塞的外部 I/O。
    """
    def __init__(self, transport: Transport, clock: Callable[[], float], limits: Limits):
        self.transport, self.clock, self.limits = transport, clock, limits
        self.phase = Phase.OBSERVING
        self.failure: str | None = None
        self.disabled_confirmed = False
        self.observation: Observation | None = None
        self.mode: tuple[str, str] | None = None
        self.values: dict[str, tuple[int, float]] = {}
        self._mode_at = self._valid_at = -math.inf
        self._valid = False
        self._last_time = -math.inf
        self._stop_at = self._last_stop_tx = -math.inf
        self._disable_sent = False

    def _now(self):
        now = self.clock()
        if not math.isfinite(now) or now < self._last_time:
            self.observation = None
            self.failure = self.failure or 'clock_invalid'
            was_active = self.phase in (Phase.ACTIVE, Phase.STOPPING)
            self.phase = Phase.FINISHED
            if was_active:
                self._write('VEL 0')
            raise RuntimeError('clock_invalid')
        self._last_time = now
        return now

    def _write(self, line):
        data = (line + '\n').encode('ascii')
        try:
            if self.transport.write(data) != len(data):
                raise OSError('short_write')
            return True
        except (OSError, TimeoutError):
            self.failure = self.failure or 'transport_write'
            self.observation = None
            self.phase = Phase.FINISHED
            return False

    def observe(self):
        """清除预检缓存并发只读查询；不CONFIG、不确认位置、不ENABLE。"""
        if self.phase != Phase.OBSERVING or self.failure:
            raise RuntimeError('session_not_observing')
        self._now()
        self.observation = None
        self.mode = None
        self.values.clear()
        self._valid = False
        self._mode_at = self._valid_at = -math.inf
        self.refresh()

    def refresh(self):
        """刷新模式、标定及配置，保留遥测；活动期间应每0.5秒调用。"""
        if self.phase not in (Phase.OBSERVING, Phase.ACTIVE) or self.failure:
            raise RuntimeError('session_not_refreshable')
        self._now()
        for line in ('QUERY', 'CALIB?', 'CONFIG?'):
            if not self._write(line):
                break

    def _fresh(self, timestamp, now, maximum):
        return 0 <= now - timestamp <= maximum

    def _measurement_ready(self, now):
        return (self._valid and self._fresh(self._valid_at, now, self.limits.validity_timeout_s)
                and self.observation is not None
                and self._fresh(self.observation.received_at, now, self.limits.state_timeout_s))

    def preflight_reason(self):
        now = self._now()
        if self.failure:
            return self.failure
        reason = self._metadata_reason(now)
        if reason:
            return reason
        if not self._measurement_ready(now):
            return 'measurement_unavailable'
        if abs(self.observation.x_um) >= self.track_limit_um:
            return 'position_outside_limits'
        return None

    @property
    def expected_readback(self):
        return {'CAL_STATE': 2, 'CAL_SAVED': 1, 'CAL_STORAGE': 0, **self.limits.config}

    @property
    def readback_names(self):
        return self.expected_readback.keys() | {'CAL_LENGTH_UM'}

    def _metadata_reason(self, now):
        if self.mode != ('READY', 'NONE') or not self._fresh(self._mode_at, now, self.limits.preflight_timeout_s):
            return 'mode_not_ready'
        expected = self.expected_readback
        for key, value in expected.items():
            entry = self.values.get(key)
            if entry is None or entry[0] != value or not self._fresh(entry[1], now, self.limits.preflight_timeout_s):
                return 'readback_' + key
        length = self.values.get('CAL_LENGTH_UM')
        if length is None or not 100000 <= length[0] <= 1000000 or not self._fresh(length[1], now, self.limits.preflight_timeout_s):
            return 'readback_CAL_LENGTH_UM'
        return None

    @property
    def track_limit_um(self):
        return self.values['CAL_LENGTH_UM'][0] // 2 - self.limits.margin_um

    def start(self):
        """显式开启本机发送权限；固件必须已由操作者使能。本调用不发送运动。"""
        if self.phase != Phase.OBSERVING:
            raise RuntimeError('session_not_observing')
        reason = self.preflight_reason()
        if reason:
            raise RuntimeError(reason)
        self.phase = Phase.ACTIVE

    def _fail(self, reason, now):
        if self.phase in (Phase.ACTIVE, Phase.STOPPING):
            self.failure = self.failure or reason
        self.observation = None
        self._valid = False
        if self.phase == Phase.ACTIVE:
            self._begin_stop(now)

    def _begin_stop(self, now):
        self.phase = Phase.STOPPING
        self._stop_at = now
        self._last_stop_tx = now
        self._disable_sent = False
        self.observation = None
        self._write('VEL 0')

    def stop(self):
        """开始有界零速/禁用流程；不把调用成功当成机械已停。"""
        now = self._now()
        if self.phase == Phase.ACTIVE:
            self._begin_stop(now)
        elif self.phase == Phase.STOPPING:
            self.tick()

    def abort(self):
        """外部推理/接收适配器失败时锁存停止，不需要伪造协议行。"""
        now = self._now()
        if self.phase == Phase.OBSERVING:
            self.failure = self.failure or 'application_abort'
            self.observation = None
            self.phase = Phase.FINISHED
        elif self.phase in (Phase.ACTIVE, Phase.STOPPING):
            self._fail('application_abort', now)

    def receive(self, line: str):
        # Check old deadlines before newly received bytes can renew their timestamps.
        self.tick()
        if self.phase == Phase.FINISHED:
            return
        now = self._now()
        words = line.strip().split()
        if not words:
            return
        tag = words[0]
        try:
            if tag == 'VEL_VALID':
                if len(words) != 3 or any(w not in ('0', '1') for w in words[1:]):
                    raise ValueError()
                valid = words[1:] == ['1', '1']
                if not valid or not self._valid:
                    self.observation = None
                self._valid, self._valid_at = valid, now
                if not valid:
                    self._fail('measurement_invalid', now)
            elif tag == 'STATE':
                if len(words) != 5:
                    raise ValueError()
                values = tuple(int(w) for w in words[1:])
                if any(not -(2**31) <= n < 2**31 for n in values):
                    raise ValueError()
                if self._valid and self._fresh(self._valid_at, now, self.limits.validity_timeout_s):
                    self.observation = Observation(*values, now)
            elif tag == 'MODE':
                if len(words) != 3 or words[1] not in ('DISABLED', 'READY', 'FAULT'):
                    raise ValueError()
                self.mode, self._mode_at = (words[1], words[2]), now
                if self.phase == Phase.STOPPING and self._disable_sent and self.mode == ('DISABLED', 'NONE'):
                    self.disabled_confirmed = True
                    self.phase = Phase.FINISHED
                elif self.phase == Phase.ACTIVE and self.mode != ('READY', 'NONE'):
                    self._fail('firmware_mode', now)
                elif self.phase == Phase.STOPPING and self.mode not in (('READY', 'NONE'), ('DISABLED', 'NONE')):
                    self._fail('firmware_mode', now)
            elif tag == 'OK':
                known = len(words) >= 2 and words[1] in self.readback_names
                if known:
                    try:
                        if len(words) != 3:
                            raise ValueError()
                        int(words[2])
                    except ValueError:
                        self.values.pop(words[1], None)
                        raise
                if len(words) == 3:
                    value = int(words[2])
                    previous = self.values.get(words[1])
                    self.values[words[1]] = (value, now)
                    if self.phase == Phase.ACTIVE:
                        expected = self.expected_readback
                        if (words[1] in expected and value != expected[words[1]]) or (words[1] == 'CAL_LENGTH_UM' and previous and value != previous[0]):
                            self._fail('readback_changed', now)
                elif len(words) != 2:
                    raise ValueError()
            elif tag == 'ERR':
                if self.phase == Phase.STOPPING and words == ['ERR', 'BUSY']:
                    self._disable_sent = False
                elif self.phase in (Phase.ACTIVE, Phase.STOPPING):
                    self._fail('firmware_error', now)
                # BUSY permits a later retry without extending the stopping deadline.
        except ValueError:
            if tag in ('STATE', 'VEL_VALID', 'MODE', 'OK'):
                self._valid = False
                self._fail('malformed_' + tag, now)

    def tick(self):
        now = self._now()
        if self.phase == Phase.OBSERVING and not self._measurement_ready(now):
            self.observation = None
        if self.phase == Phase.ACTIVE and self._metadata_reason(now):
            self._fail('metadata_unavailable', now)
        if self.phase == Phase.ACTIVE and not self._measurement_ready(now):
            self._fail('measurement_timeout', now)
        if self.phase == Phase.STOPPING:
            if now - self._stop_at >= self.limits.stop_timeout_s:
                self.failure = self.failure or 'stop_unconfirmed'
                self.phase = Phase.FINISHED
                self.observation = None
            elif now - self._last_stop_tx >= .03:
                self._last_stop_tx = now
                if not self._write('VEL 0'):
                    return
                if self._measurement_ready(now) and self.observation.received_at > self._stop_at and self.observation.velocity_mm_s == 0:
                    if self._write('DISABLE'):
                        self._disable_sent = True

    def command(self, action: float):
        """[-1,1]动作映射为目标速度；每次发送前重新检查有效性。"""
        self.tick()
        if self.phase != Phase.ACTIVE:
            raise RuntimeError(self.failure or 'session_not_active')
        try:
            valid_action = (isinstance(action, Real) and not isinstance(action, bool)
                            and math.isfinite(action) and -1 <= action <= 1)
        except (TypeError, ValueError, OverflowError):
            valid_action = False
        if not valid_action:
            self._fail('invalid_action', self._now())
            raise ValueError('invalid_action')
        # 整数 mm/s，恰好半单位时远离零，与 C 的 lround 一致。
        fraction, whole = math.modf(action * self.limits.speed_mm_s)
        velocity = int(whole)
        if abs(fraction) >= 0.5:
            velocity += 1 if fraction > 0 else -1
        if abs(self.observation.x_um) >= self.track_limit_um and self.observation.x_um * velocity > 0:
            velocity = 0
        if not self._write(f'VEL {velocity}'):
            raise RuntimeError(self.failure)
        return velocity
