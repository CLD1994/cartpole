#!/usr/bin/env python3
"""禁用设备上的决策节拍诊断；只查询和推理，绝不发送VEL或使能。"""
import argparse
import json
import math
from pathlib import Path
import time

from shadow import CLOCK, Policy, QueryOnlyTransport, summary
import model_manifest
from experiment_setup import limits_from_manifest, PROTOCOL_BAUD, report_path, ANGLE_SIGN
from cli_output import print_readonly_summary, readonly_entrypoint
from receiver import Receiver
from session import Session


class TimingFailure(RuntimeError):
    def __init__(self, reason):
        self.trigger_clock = CLOCK()
        super().__init__(reason)


class DiagnosticSession(Session):
    """观察态保持只读；计时阶段锁存控制所需的故障与测量失效。"""
    timing_armed = False

    def _fail(self, reason, now):
        if self.timing_armed:
            self.failure = self.failure or reason
        super()._fail(reason, now)

    def receive(self, line):
        super().receive(line)
        if self.timing_armed:
            words = line.split()
            if words and words[0] == 'ERR':
                self._fail('firmware_error', self._now())
            if self.mode != ('DISABLED', 'NONE'):
                self._fail('device_not_disabled', self._now())

    def tick(self):
        super().tick()
        if self.timing_armed and not self._measurement_ready(self._now()):
            self._fail('measurement_timeout', self._now())


class ObservedSource:
    def __init__(self, port):
        self.port = port
        self.last_count = 0
        self.last_read = None
        self.partial = False

    @property
    def in_waiting(self):
        self.last_count = self.port.in_waiting
        return self.last_count

    def read(self, count):
        start = CLOCK()
        data = self.port.read(count)
        self.last_read = {'start_s': start, 'duration_s': CLOCK()-start,
                          'requested_bytes': count, 'returned_bytes': len(data)}
        if data:
            self.partial = not data.endswith(b'\n')
        return data


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--model', required=True, type=Path, help='模型导出目录')
    p.add_argument('--json', action='store_true', help='输出机器可读 JSON')
    p.add_argument('--output', type=Path)
    p.add_argument('--seconds', type=float, default=.5)
    p.add_argument('--scheduler', choices=('legacy', 'wait-fresh'), default='legacy')
    a = p.parse_args()
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= .5:
        p.error('--seconds 必须在(0,0.5]；诊断复用运动原型的半秒窗口')
    model = a.model / 'policy.onnx'
    manifest = model_manifest.load_manifest(a.model / 'model.json', model)
    limits = limits_from_manifest(manifest)
    period = 1 / limits.control_hz
    a.output = a.output or report_path(a.model, 'timing')
    policy = Policy(model)
    for _ in range(20):
        policy.forward((0., 0., 0., 0.))
    report = {'diagnostic_revision': 3, 'model_manifest': manifest, 'control_period_s': period, 'status': 'starting', 'motion_sent': False, 'scheduler': a.scheduler,
              'scope': '禁用设备只读调度诊断；虚拟决策不是串口动作，不验证闭环或机械响应',
              'differences_from_motion': ['无VEL写入耗时/回执及真实动力学负载',
                  '角量固定按历史映射取反，仅比较调度，不读取或验收运动坐标合同',
                  '不执行运动状态角度/位置/制动门控，允许下垂静态诊断',
                  '只复现测量/故障及时序门控，不复现ACTIVE配置一致性检查',
                  '新增内存记录会影响计时；不同运行不能视为同一遥测重放'],
              'events': [], 'virtual_decisions': [], 'clock': vars(time.get_clock_info('perf_counter'))}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as output:
        session = receiver = source = None
        start = next_tick = last_decision = None
        stage = 'open'
        last_sample = None
        poll_s = wait_s = inference_s = None
        try:
            import serial
            port = serial.Serial(port=None, baudrate=PROTOCOL_BAUD, timeout=0, write_timeout=.002)
            port.dtr = port.rts = False
            port.port = a.port
            with port:
                source = ObservedSource(port)
                session = DiagnosticSession(QueryOnlyTransport(port), CLOCK, limits)
                receiver = Receiver(source, session, CLOCK)
                port.reset_input_buffer()
                deadline = CLOCK()+.5
                while CLOCK() < deadline:
                    byte = port.read(1)
                    if byte == b'\n':
                        break
                    if not byte:
                        time.sleep(.0002)
                else:
                    raise TimingFailure('alignment_timeout')
                session.observe()
                deadline = CLOCK()+2
                stage = 'require_disabled_and_query_replies'
                required_replies = set(session.readback_names) | {
                    'POLE_ZERO_URAD', 'POLE_INV', 'X_INV', 'STEP_LOSS_UM'}
                while CLOCK() < deadline:
                    if not receiver.poll() or session.failure:
                        raise TimingFailure(receiver.error or session.failure)
                    if session.mode is not None and session.mode != ('DISABLED', 'NONE'):
                        raise TimingFailure('device_not_disabled')
                    if (session.mode == ('DISABLED', 'NONE') and session.observation is not None
                            and required_replies <= session.values.keys()
                            and not source.partial and source.in_waiting == 0):
                        # 仅要求本次查询完整收齐，不要求READY或配置等于控制目标。
                        break
                    time.sleep(.0002)
                else:
                    raise TimingFailure('disabled_startup_not_complete')
                report['startup_readback'] = {k: v[0] for k, v in session.values.items()}
                session.timing_armed = True
                start = next_tick = last_decision = CLOCK()
                deadline = start+a.seconds
                while CLOCK() < deadline:
                    stage = 'poll'
                    if source.in_waiting > 128:
                        raise TimingFailure('active_backlog')
                    begin = CLOCK()
                    if not receiver.poll() or session.failure:
                        raise TimingFailure(receiver.error or session.failure)
                    poll_s = CLOCK()-begin
                    if session.mode != ('DISABLED', 'NONE'):
                        raise TimingFailure('device_not_disabled')
                    now = CLOCK()
                    stage = 'gap_check'
                    if now-last_decision > .004:
                        raise TimingFailure('command_gap_deadline')
                    if now < next_tick:
                        stage = 'wait_tick'
                        begin = CLOCK()
                        time.sleep(min(.0002, next_tick-now))
                        wait_s = CLOCK()-begin
                        continue
                    stage = 'tick'
                    if now-next_tick > period:
                        raise TimingFailure('control_deadline')
                    obs = session.observation
                    fresh = obs is not None and obs.received_at != last_sample
                    report['events'].append({'elapsed_s': now-start, 'new_observation': fresh,
                        'observation_effective_age_s': now-obs.received_at if obs else None,
                        'virtual_gap_s': now-last_decision, 'tick_lateness_s': now-next_tick,
                        'poll_s': poll_s, 'previous_wait_s': wait_s, 'pending_bytes_cached': source.last_count})
                    if obs is None or now-obs.received_at > .004:
                        raise TimingFailure('observation_late')
                    if a.scheduler == 'legacy':
                        next_tick = now+period
                    if not fresh:
                        stage = 'wait_fresh'
                        if a.scheduler == 'wait-fresh':
                            begin = CLOCK()
                            time.sleep(.0002)
                            wait_s = CLOCK()-begin
                        continue
                    if a.scheduler == 'wait-fresh':
                        next_tick = now+period
                    last_sample = obs.received_at
                    stage = 'inference'
                    begin = CLOCK()
                    x, th, vx, w = obs.si()
                    raw = policy.forward((x, ANGLE_SIGN*th, vx, ANGLE_SIGN*w))
                    inference_s = CLOCK()-begin
                    if CLOCK()-obs.received_at > .004 or CLOCK()-now > period:
                        raise TimingFailure('inference_deadline')
                    stage = 'predecision_poll'
                    # 固定原始截止，不因分片/新STATE到达延长候选动作的预算。
                    receive_deadline = min(last_decision+.004, obs.received_at+.004, deadline)
                    drain = {'deadline_clock_s': receive_deadline, 'polls': 0,
                             'wait_s': 0., 'started_clock_s': CLOCK()}
                    report['events'][-1]['predecision_receive'] = drain
                    while True:
                        if CLOCK() >= receive_deadline:
                            raise TimingFailure('predecision_receive_deadline')
                        if source.in_waiting > 128:
                            raise TimingFailure('precommand_backlog')
                        begin = CLOCK()
                        if not receiver.poll() or session.failure:
                            raise TimingFailure(receiver.error or session.failure)
                        poll_s = CLOCK()-begin
                        drain['polls'] += 1
                        drain['last_poll_s'] = poll_s
                        if session.mode != ('DISABLED', 'NONE'):
                            raise TimingFailure('device_not_disabled')
                        latest = session.observation
                        if latest is None or not session._measurement_ready(CLOCK()):
                            raise TimingFailure('predecision_measurement_invalid')
                        # Receiver保留分片；新到完整有效STATE不替换原推理输入/截止。
                        pending = source.in_waiting
                        drain['pending_bytes'] = pending
                        drain['partial'] = source.partial
                        drain['elapsed_s'] = CLOCK()-drain['started_clock_s']
                        if CLOCK() >= receive_deadline:
                            raise TimingFailure('predecision_receive_deadline')
                        if pending > 128:
                            raise TimingFailure('precommand_backlog')
                        if not source.partial and pending == 0:
                            break
                        if pending == 0:
                            begin = CLOCK()
                            time.sleep(min(.0001, max(0., receive_deadline-begin)))
                            drain['wait_s'] += CLOCK()-begin
                    stage = 'virtual_decision'
                    completed = CLOCK()
                    if completed >= receive_deadline:
                        raise TimingFailure('prewrite_fixed_deadline')
                    if completed >= deadline:
                        break
                    report['virtual_decisions'].append({'elapsed_s': completed-start,
                        'gap_s': completed-last_decision, 'inference_s': inference_s, 'raw_action': raw})
                    last_decision = completed
                report['status'] = 'complete'
        except (Exception, KeyboardInterrupt) as exc:
            now = getattr(exc, 'trigger_clock', CLOCK())
            obs = session.observation if session else None
            report['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'error'
            report['error'] = f'{type(exc).__name__}: {exc}'
            # 异常后仅使用缓存，不再读取串口/打印，也绝不发停机或动作命令。
            report['failure_snapshot'] = {'stage': stage, 'clock_s': now,
                'clock_capture': 'at_failure' if hasattr(exc, 'trigger_clock') else 'after_cleanup_for_unexpected_exception',
                'elapsed_s': now-start if start is not None else None,
                'last_virtual_decision_clock_s': last_decision,
                'actual_virtual_gap_s': now-last_decision if last_decision is not None else None,
                'next_tick_clock_s': next_tick, 'tick_lateness_s': now-next_tick if next_tick is not None else None,
                'observation_freshness_reference_s': obs.received_at if obs else None,
                'observation_is_new': obs.received_at != last_sample if obs else False,
                'observation_effective_age_s': now-obs.received_at if obs else None,
                'poll_s': poll_s, 'last_wait_s': wait_s, 'last_inference_s': inference_s,
                'pending_bytes_cached': source.last_count if source else None,
                'partial': source.partial if source else None, 'last_read': source.last_read if source else None}
        finally:
            report['decision_gap_s'] = summary([r['gap_s'] for r in report['virtual_decisions']])
            report['duplicate_tick_count'] = sum(not r['new_observation'] for r in report['events'])
            report['last_observed_firmware_mode'] = session.mode if session else None
            report['receive_error'] = receiver.error if receiver else None
            json.dump(report, output, ensure_ascii=False, indent=2)
            output.write('\n')
    print_readonly_summary(report, a.output, json_output=a.json)
    return 0 if report['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(readonly_entrypoint(main))

