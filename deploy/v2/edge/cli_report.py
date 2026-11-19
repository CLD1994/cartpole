"""端侧 CLI 的只读报告接口，不参与设备控制。

parse_state_window_summary 接受 state --secs 的完整四行 stdout 报告：窗口汇总、
未知行计数、小车范围或不可用、摆杆范围或不可用，顺序固定且各一次。
允许 LF/CRLF 和最后一个换行；拒绝附加行、重复行、缺项与非法数值。
parse_state_output 接受该报告及同一 stdout 中的 MODE/STEP_LOSS 事件。
时间和频率是 CLI 已舍入的观测值，不代表设备采样时间或精确频率。
"""
import math
import re
from typing import NamedTuple

ERROR_INVALID_STATE_WINDOW_REPORT = 'invalid_state_window_report'


class StateWindowReport(NamedTuple):
    elapsed_s: float
    received_state_count: int
    observed_hz: float
    unknown_line_count: int
    cart_range_um: tuple[int, int] | None
    pole_range_urad: tuple[int, int] | None


def parse_state_window_summary(output: str) -> StateWindowReport:
    """校验完整报告并返回带单位字段；非法报告抛出 ValueError。"""
    def invalid():
        raise ValueError(ERROR_INVALID_STATE_WINDOW_REPORT)

    if not isinstance(output, str):
        invalid()
    normalized = output.replace('\r\n', '\n')
    if normalized.endswith('\n'):
        normalized = normalized[:-1]
    lines = normalized.split('\n')
    if len(lines) != 4:
        invalid()
    summary = re.fullmatch(
        r'\[edge\] window ([0-9]+\.[0-9]) s: ([0-9]+) STATE lines -> ([0-9]+\.[0-9]) Hz \(observed\)', lines[0])
    unknown = re.fullmatch(r'\[edge\] unknown lines: ([0-9]+) \(require 0\)', lines[1])
    if summary is None or unknown is None:
        invalid()
    elapsed, count, hz = float(summary[1]), int(summary[2]), float(summary[3])
    if not math.isfinite(elapsed) or not math.isfinite(hz):
        invalid()

    def measurement_range(line, lane, symbol, unit):
        if line == f'[edge] {lane} samples unavailable':
            return None
        match = re.fullmatch(r'\[edge\] sampled ' + symbol + r' range \[(-?[0-9]+), (-?[0-9]+)\] ' + unit, line)
        if match is None:
            invalid()
        low, high = int(match[1]), int(match[2])
        if low > high:
            invalid()
        return low, high

    return StateWindowReport(elapsed, count, hz, int(unknown[1]),
        measurement_range(lines[2], 'cart', 'x', 'um'),
        measurement_range(lines[3], 'pole', 'theta', 'urad'))


class ModeEvent(NamedTuple):
    mode: str
    reason: str


class StepLossEvent(NamedTuple):
    event_id: int
    run_id: int
    device_time_us: int


class StateOutput(NamedTuple):
    report: StateWindowReport
    events: tuple[ModeEvent | StepLossEvent, ...]


def parse_state_output(output: str) -> StateOutput:
    """解析 state --secs 完整 stdout；仅允许四行报告及已定义的异步事件。

    MODE 只接受当前设备模式/原因；STEP_LOSS 三字段为非负 int64。
    保留事件接收顺序；任何其他行均拒绝，不静默丢弃未知输出。
    """
    if not isinstance(output, str):
        raise ValueError(ERROR_INVALID_STATE_WINDOW_REPORT)
    normalized = output.replace('\r\n', '\n')
    if normalized.endswith('\n'):
        normalized = normalized[:-1]
    records, events = [], []
    for line in normalized.split('\n'):
        mode = re.fullmatch(r'\[edge\] MODE (DISABLED|READY|FAULT) (NONE|STEP_LOSS|AS5600|MT6835|POWER)', line)
        loss = re.fullmatch(r'\[edge\] STEP_LOSS event=([0-9]{1,19}) run=([0-9]{1,19}) device_us=([0-9]{1,19})', line)
        if mode:
            events.append(ModeEvent(mode[1], mode[2]))
        elif loss:
            values = tuple(int(value) for value in loss.groups())
            if any(value > 2**63 - 1 for value in values):
                raise ValueError(ERROR_INVALID_STATE_WINDOW_REPORT)
            events.append(StepLossEvent(*values))
        else:
            records.append(line)
    return StateOutput(parse_state_window_summary('\n'.join(records)), tuple(events))
