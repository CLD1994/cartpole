"""正式 STATE：测量与两路有效位；不提供设备时间或样本身份。"""
from dataclasses import dataclass
import re

@dataclass(frozen=True)
class Lane:
    position: int
    velocity: int

@dataclass(frozen=True)
class StateFrame:
    cart: Lane | None
    pole: Lane | None

def parse_state(line: str) -> StateFrame:
    number = r'(?:0|-?[1-9][0-9]*)'
    if re.fullmatch(r'STATE [0-3] ' + ' '.join([number] * 4), line) is None:
        raise ValueError('state_format')
    flags, x, th, vx, w = map(int, line.split(' ')[1:])
    if any(not -2147483648 <= v <= 2147483647 for v in (x, th, vx, w)):
        raise ValueError('state_range')
    lanes = []
    for bit, pos, vel in ((1, x, vx), (2, th, w)):
        if not flags & bit:
            if pos != 0 or vel != 0:
                raise ValueError('state_invalid_lane')
            lanes.append(None)
        else:
            if bit == 2 and abs(pos) > 3141592:
                raise ValueError('state_angle')
            lanes.append(Lane(pos, vel))
    return StateFrame(*lanes)
