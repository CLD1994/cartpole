"""端侧 CLI 的只读报告接口，不参与设备控制。

报告依次包含有效/总帧数、未知行、小车范围、摆杆范围和目标频率结论。
允许 LF/CRLF 和最后一个换行；拒绝附加行、重复行、缺项与非法数值。
parse_state_output 还接受同一 stdout 中的 MODE/STEP_LOSS 事件。
时间和频率是 CLI 已舍入的观测值，不代表设备采样时间或精确频率。
"""
import math
import re
from typing import NamedTuple

ERROR_INVALID_STATE_WINDOW_REPORT = 'invalid_state_window_report'


class StateWindowReport(NamedTuple):
    elapsed_s: float
    received_state_count: int
    valid_state_count: int
    observed_hz: float
    unknown_line_count: int
    cart_range_um: tuple[int, int] | None
    pole_range_urad: tuple[int, int] | None
    expected_hz: float | None
    rate_ok: bool | None


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
    if len(lines) != 5:
        invalid()
    fd = r'[0-9]+\.[0-9]'
    summary = re.fullmatch(
        rf'时长 ({fd}) s，有效 STATE ([0-9]+) 行 → ({fd}) Hz（接收总数 ([0-9]+) 行）', lines[0])
    unknown = re.fullmatch(r'UNKNOWN 行：([0-9]+)（要求 0）', lines[1])
    if summary is None or unknown is None:
        invalid()
    elapsed, count, hz, total = float(summary[1]), int(summary[2]), float(summary[3]), int(summary[4])
    if not math.isfinite(elapsed) or not math.isfinite(hz) or count > total:
        invalid()

    def measurement_range(line, unavailable, pattern):
        if line == unavailable:
            return None
        match = re.fullmatch(pattern, line)
        if match is None:
            invalid()
        low, high = int(match[1]), int(match[2])
        if low > high or not all(math.isfinite(float(n)) for n in match.groups()[2:]):
            invalid()
        return low, high

    fixed2 = r'-?[0-9]+\.[0-9]{2}'
    cart = measurement_range(lines[2], '小车测量不可用',
                             rf'x_um ∈ \[(-?[0-9]+), (-?[0-9]+)\]（漂移 [0-9]+ µm；x ≈ ({fixed2}) mm）')
    pole = measurement_range(lines[3], '摆杆测量不可用',
                             rf'theta_urad ∈ \[(-?[0-9]+), (-?[0-9]+)\]（θ ∈ \[({fixed2})°, ({fixed2})°\]）')
    expected, rate_ok = None, None
    if lines[4] != '未指定目标频率，仅检查收到有效 STATE 且 UNKNOWN 为 0；不证明期间没有无效帧。':
        target = re.fullmatch(r'目标 ([0-9]+(?:\.[0-9]+)?(?:e[+-][0-9]+)?) Hz（容差 ±1 Hz）：(通过|未通过)', lines[4])
        if target is None:
            invalid()
        expected = float(target[1])
        if not math.isfinite(expected) or expected <= 0:
            invalid()
        rate_ok = target[2] == '通过'
    return StateWindowReport(elapsed, total, count, hz, int(unknown[1]), cart, pole, expected, rate_ok)


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
    """解析 state --secs 完整 stdout；仅允许报告及已定义的异步事件。

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
        mode = re.fullmatch(r'\[MODE\] (DISABLED|READY|FAULT) (NONE|STEP_LOSS|AS5600|MT6835|POWER)', line)
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
