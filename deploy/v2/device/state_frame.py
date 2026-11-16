"""统一 STATE 文本帧；无效路返回 None，不把编码占位解释为测量。"""
from dataclasses import dataclass
import re

@dataclass(frozen=True)
class Lane:
    seq: int
    age_us: int
    position: int
    velocity: int

@dataclass(frozen=True)
class StateFrame:
    boot: int
    tick: int
    cart: Lane | None
    pole: Lane | None

_WIDTHS = (8, 8, 1, 4, 4, 4, 4, 8, 8, 8, 8)

def parse_state(line: str) -> StateFrame:
    pattern = "STATE " + " ".join(r"[0-9a-fA-F]{%d}" % n for n in _WIDTHS)
    if re.fullmatch(pattern, line) is None:
        raise ValueError("state_format")
    words = line.split(" ")
    if len(words) != 12 or words[0] != 'STATE':
        raise ValueError('state_format')
    if any(re.fullmatch(r'[0-9a-fA-F]{%d}' % width, word) is None
           for word, width in zip(words[1:], _WIDTHS)):
        raise ValueError('state_hex')
    boot, tick, flags, cs, ps, ca, pa, x, th, vx, w = (int(v, 16) for v in words[1:])
    if flags > 3:
        raise ValueError('state_flags')
    def signed(v):
        return v if v < 0x80000000 else v - 0x100000000
    lanes = []
    for bit, seq, age, pos, vel in ((1, cs, ca, x, vx), (2, ps, pa, th, w)):
        if not flags & bit:
            if (seq, age, pos, vel) != (0, 0xffff, 0, 0):
                raise ValueError('state_invalid_lane')
            lanes.append(None)
        else:
            if age > 20000:
                raise ValueError('state_age')
            if bit == 2 and abs(signed(pos)) > 3141592:
                raise ValueError('state_angle')
            lanes.append(Lane(seq, age, signed(pos), signed(vel)))
    return StateFrame(boot, tick, *lanes)
