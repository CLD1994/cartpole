#!/usr/bin/env python3
"""只读设备会话检查器，不执行策略、不改变配置或电机使能。"""
import argparse
from pathlib import Path
import time

from session import Session
from receiver import Receiver

import model_manifest
from experiment_setup import limits_from_manifest, PROTOCOL_BAUD
from cli_output import print_readonly_summary, readonly_entrypoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--model', required=True, type=Path, help='包含 model.json 和 policy.onnx 的导出目录')
    parser.add_argument('--json', action='store_true', help='输出机器可读 JSON')
    parser.add_argument('--seconds', type=float, default=2.)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error('--seconds 必须在 (0,60]')
    manifest = model_manifest.load_manifest(args.model / 'model.json', args.model / 'policy.onnx')
    limits = limits_from_manifest(manifest)
    import serial
    with serial.Serial(args.port, PROTOCOL_BAUD, timeout=0, write_timeout=.1) as port:
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
        report = {'mode': session.mode, 'preflight_reason': reason,
                          'state': vars(session.observation) if session.observation else None,
                          'readback': {k: v[0] for k, v in session.values.items()},
                          'target_control_hz': limits.control_hz,
                          'receive_error': receiver.error, 'session_failure': session.failure, 'motion_sent': False}
        report['status'] = 'complete' if not (receiver.error or session.failure or reason) else 'error'
        print_readonly_summary(report, json_output=args.json)
    return int(receiver.error is not None or session.failure is not None or reason is not None)


if __name__ == '__main__':
    raise SystemExit(readonly_entrypoint(main))
