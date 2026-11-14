#!/usr/bin/env python3
"""只读设备会话检查器，不执行策略、不改变配置或电机使能。"""
import argparse
import json
from pathlib import Path
import time

from session import Limits, Session
from receiver import Receiver

PROFILE = Path(__file__).resolve().parents[3] / 'source/cartpole/cartpole/data/device-profile.json'


def limits_from_profile(profile):
    c = profile['configured']
    return Limits(profile['training']['control_hz'], c['target_speed_mm_s'],
                  c['acceleration_mm_s2'], c['soft_limit_margin_um'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--seconds', type=float, default=2.)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds 必须在 (0,60]')
    import serial
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    limits = limits_from_profile(profile)
    with serial.Serial(args.port, profile['measured']['protocol_baud'], timeout=0, write_timeout=.1) as port:
        port.reset_input_buffer()
        session = Session(port, time.monotonic, limits)
        session.observe()
        deadline = time.monotonic() + args.seconds
        receiver = Receiver(port, session, time.monotonic)
        next_query = time.monotonic() + .5
        while time.monotonic() < deadline:
            if not receiver.poll():
                break
            time.sleep(.001)
            if time.monotonic() >= next_query:
                session.refresh()
                next_query = time.monotonic() + .5
            session.tick()
        reason = session.preflight_reason()
        print(json.dumps({'mode': session.mode, 'preflight_reason': reason,
                          'state': vars(session.observation) if session.observation else None,
                          'readback': {k: v[0] for k, v in session.values.items()},
                          'target_control_hz': limits.control_hz,
                          'receive_error': receiver.error, 'session_failure': session.failure, 'motion_sent': False}, ensure_ascii=False, indent=2))
    return int(receiver.error is not None or session.failure is not None or reason is not None)


if __name__ == '__main__':
    raise SystemExit(main())
