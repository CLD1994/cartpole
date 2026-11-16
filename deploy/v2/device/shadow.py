#!/usr/bin/env python3
"""只读策略影子运行：查询设备并推理，绝不发送运动或配置命令。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

# CPython 3.12 Windows monotonic 使用粗粒度 GetTickCount64。所有期限与
# 接收时间戳必须共享 perf_counter，不能混用不同起点的时钟。
CLOCK = time.perf_counter

import model_manifest
from experiment_setup import limits_from_manifest, PROTOCOL_BAUD, report_path
from cli_output import print_readonly_summary, readonly_entrypoint
from receiver import Receiver
from session import Session


class QueryOnlyTransport:
    """在串口写入边界阻止所有非查询命令。"""
    def __init__(self, port):
        self.port = port

    def write(self, data):
        if data not in (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n'):
            raise RuntimeError('shadow_write_rejected')
        return self.port.write(data)


class Policy:
    """四维单批次观测；仅支持单动作。"""
    def __init__(self, path, action_dim=1):
        if action_dim != 1:
            raise ValueError("unsupported_action_dimension")
        self.action_dim = action_dim
        import numpy as np
        import onnxruntime as ort
        self.np = np
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.runtime = ort.InferenceSession(str(path), sess_options=options,
                                            providers=['CPUExecutionProvider'])
        inputs, outputs = self.runtime.get_inputs(), self.runtime.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('需要单输入、单输出的无记忆策略')
        if inputs[0].type != 'tensor(float)' or len(inputs[0].shape) != 2 or inputs[0].shape[1] != 4:
            raise ValueError('输入必须为 float32 [batch,4]，顺序 [x,theta,xdot,thetadot]')
        if isinstance(inputs[0].shape[0], int) and inputs[0].shape[0] != 1:
            raise ValueError('输入批次必须允许1')
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.forward((0., 0., 0., 0.))

    def forward(self, observation):
        x = self.np.asarray(observation, dtype=self.np.float32).reshape(1, 4)
        if not self.np.isfinite(x).all():
            raise ValueError('nonfinite_observation')
        y = self.runtime.run([self.output_name], {self.input_name: x})[0]
        shapes = ((1,), (1, 1))
        if y.shape not in shapes or not self.np.isfinite(y).all():
            raise ValueError('invalid_policy_output')
        return float(y.reshape(-1)[0])


def summary(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'count': len(values), 'mean': sum(values) / len(values),
            'p95': ordered[math.ceil(.95 * len(values)) - 1], 'max': ordered[-1]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--model', type=Path, required=True, help='模型导出目录')
    p.add_argument('--json', action='store_true', help='输出机器可读 JSON')
    p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--angle-sign', choices=('both', 'positive', 'negative'), default='both',
                   help='默认分别记录两种角量映射，不自动判断哪种正确')
    p.add_argument('--output', type=Path, help='新建JSON报告，拒绝覆盖已有文件')
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        p.error('--seconds 必须在(0,60]')
    model = args.model / 'policy.onnx'
    manifest = model_manifest.load_manifest(args.model / 'model.json', model)
    limits = limits_from_manifest(manifest)
    args.output = args.output or report_path(args.model, 'shadow')
    policy = Policy(model)
    # 模型初始化/预热在打开串口前完成，避免初始化延迟占据接收周期。
    for _ in range(20):
        policy.forward((0., 0., 0., 0.))
    signs = (1, -1) if args.angle_sign == 'both' else ((1,) if args.angle_sign == 'positive' else (-1,))
    report = {'mode': 'shadow', 'status': 'running', 'motion_sent': False,
              'scope': '只读观测和CPU推理；不是闭环、坐标方向确认或设备采样延迟验证',
              'onnx': str(model), 'model_manifest': manifest,
              'clock': {'selected': 'perf_counter',
                        'perf_counter': vars(time.get_clock_info('perf_counter')),
                        'monotonic': vars(time.get_clock_info('monotonic'))},
              'sleep_strategy': 'time.sleep up to 200us; no global timer or priority changes',
              'angle_signs': signs, 'target_control_hz': limits.control_hz, 'samples': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # 在打开设备前确认输出路径可写，且不覆盖此前结果。
    with args.output.open('x', encoding='utf-8') as output:
        session = receiver = None
        intervals, lateness, inference, sleeps = [], [], [], []
        last_sample = last_tick = None
        try:
            import serial
            port = serial.Serial(port=None, baudrate=PROTOCOL_BAUD,
                                 timeout=0, write_timeout=.01)
            port.dtr = False
            port.rts = False
            port.port = args.port
            with port:
                port.reset_input_buffer()
                session = Session(QueryOnlyTransport(port), CLOCK, limits)
                session.observe()
                receiver = Receiver(port, session, CLOCK)
                start = CLOCK()
                deadline, next_query, next_tick = start + args.seconds, start + .5, start
                period = 1 / limits.control_hz
                while CLOCK() < deadline:
                    if not receiver.poll() or session.failure:
                        raise RuntimeError(receiver.error or session.failure)
                    now = CLOCK()
                    if now >= next_query:
                        session.refresh()
                        next_query = CLOCK() + .5
                    if now < next_tick:
                        # Python 3.11+ Windows 使用高精度等待计时器；实际唤醒仍可能迟到。
                        requested = min(.0002, next_tick - now)
                        sleep_start = CLOCK()
                        time.sleep(requested)
                        sleeps.append(CLOCK() - sleep_start)
                        continue
                    lateness.append(max(0., now - next_tick))
                    if last_tick is not None:
                        intervals.append(now - last_tick)
                    last_tick = now
                    # 不追赶错过的拍点，防止同一旧观测被集中重复推理。
                    next_tick = now + period
                    obs = session.observation
                    if obs is None or obs.received_at == last_sample:
                        continue
                    last_sample = obs.received_at
                    state = obs.si()
                    row = {'elapsed_s': now-start, 'firmware_state_si': state,
                           'observation_effective_age_s': now-obs.received_at, 'predictions': []}
                    for sign in signs:
                        begin = CLOCK()
                        action = policy.forward((state[0], sign*state[1], state[2], sign*state[3]))
                        elapsed = CLOCK()-begin
                        inference.append(elapsed)
                        row['predictions'].append({'angle_sign': sign, 'raw_action': action,
                            'clipped_action': max(-1., min(1., action)), 'inference_s': elapsed})
                    report['samples'].append(row)
                report['wall_s'] = CLOCK()-start
                report['status'] = 'complete' if report['samples'] else 'no_valid_observations'
        except KeyboardInterrupt:
            report['status'] = 'interrupted'
        except Exception as exc:
            report['status'] = 'error'
            report['error'] = f'{type(exc).__name__}: {exc}'
        finally:
            report['loop_interval_s'] = summary(intervals)
            report['schedule_lateness_s'] = summary(lateness)
            report['inference_s'] = summary(inference)
            report['sleep_wall_s'] = summary(sleeps)
            if session is not None:
                report['preflight_reason'] = session.preflight_reason()
                report['firmware_mode'] = session.mode
                report['readback'] = {k: v[0] for k, v in session.values.items()}
                report['session_failure'] = session.failure
            if receiver is not None:
                report['receive_error'] = receiver.error
            json.dump(report, output, ensure_ascii=False, indent=2)
            output.write('\n')
    print_readonly_summary(report, args.output, json_output=args.json)
    return 0 if report['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(readonly_entrypoint(main))

