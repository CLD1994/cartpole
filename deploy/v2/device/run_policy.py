#!/usr/bin/env python3
"""限时策略联调入口：无自动使能/配置；需要明确模型合同及现场启动请求。"""
import argparse
import json
import math
from pathlib import Path
import time

from shadow import CLOCK, PROFILE, Policy, summary
from check import limits_from_profile
from model_integrity import verify_model_files
from receiver import Receiver
from session import Session, Phase


class ControlTransport:
    """只允许查询、会话速度命令和禁用，拒绝ENABLE/配置/定位。"""
    def __init__(self, port):
        self.port = port
        self.partial = False

    @property
    def in_waiting(self):
        return self.port.in_waiting

    def read(self, count):
        data = self.port.read(count)
        if data:
            self.partial = not data.endswith(b"\n")
        return data

    def write(self, data):
        if data not in (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n', b'DISABLE\n'):
            try:
                line = data.decode('ascii')
                value = int(line[4:-1])
                valid = line == f'VEL {value}\n' and abs(value) <= 1250
            except (ValueError, UnicodeError):
                valid = False
            if not valid:
                raise ValueError('control_write_rejected')
        return self.port.write(data)


class BoundSession(Session):
    """把本次台位坐标参数纳入既有新鲜读回及运行中变化检查。"""
    def __init__(self, transport, clock, limits, coordinates):
        self.coordinates = coordinates
        super().__init__(transport, clock, limits)

    @property
    def expected_readback(self):
        return {**super().expected_readback, **self.coordinates}


def braking_reason(session, velocity_mm_s):
    # 仅条件性理想模型：150ms反应预算(含100ms看门狗)+配置减速度+50mm余量。
    # 真实制动能力未测量，不能把此计算称作安全停车证明。
    obs = session.observation
    if obs is None:
        return 'measurement_unavailable'
    speed = max(abs(obs.velocity_mm_s), abs(velocity_mm_s)) / 1000.
    decel = min(session.limits.watchdog_decel, session.limits.acceleration_mm_s2) / 1000.
    required = speed * .15 + speed * speed / (2 * decel) + .05
    available = (session.track_limit_um - abs(obs.x_um)) / 1e6
    return 'braking_envelope' if required >= available else None


def load_contract(path, model):
    c = json.loads(path.read_text(encoding='utf-8-sig'))
    expected = {'schema_version': 1, 'observation_order': ['x', 'theta', 'xdot', 'thetadot'],
                'observation_units': ['m', 'rad', 'm/s', 'rad/s'],
                'action_type': 'cart_target_velocity', 'action_scale_m_s': 1.25,
                'training_control_hz': 500, 'x_sign': 1, 'action_sign': 1}
    if any(c.get(k) != v for k, v in expected.items()):
        raise ValueError('model_contract_mismatch')
    if type(c.get('angle_sign')) is not int or c['angle_sign'] not in (-1, 1):
        raise ValueError('angle_sign_required')
    verify_model_files(model, c)
    coordinates = c.get('firmware_coordinates')
    if (not isinstance(coordinates, dict) or set(coordinates) != {'POLE_ZERO_URAD', 'POLE_INV', 'X_INV'}
            or any(type(v) is not int for v in coordinates.values())
            or coordinates['POLE_INV'] not in (0, 1) or coordinates['X_INV'] not in (0, 1)
            or not -3141592 <= coordinates['POLE_ZERO_URAD'] <= 3141592):
        raise ValueError('firmware_coordinates_required')
    # 声明仅用于避免误启动，不是物理验证证据；来源需由操作者审阅。
    if any(c.get(k) is not True for k in ('export_contract_reviewed', 'coordinate_evidence_reviewed',
                                         'timing_mismatch_acknowledged', 'onsite_stop_ready')):
        raise ValueError('contract_review_incomplete')
    if not isinstance(c.get('evidence_note'), str) or not c['evidence_note'].strip():
        raise ValueError('evidence_note_required')
    return c


def state_guard(obs, starting=False):
    x, th, vx, w = obs.si()
    if not all(math.isfinite(v) for v in (x, th, vx, w)):
        return 'nonfinite_state'
    # 仅近直立平衡试验，不允许下垂起摆；固定保守区域不等于安全保证。
    if abs(x) > .2 or abs(th) > (math.radians(10) if starting else math.radians(20)):
        return 'trial_position_or_angle_limit'
    if starting and (abs(vx) > .05 or abs(w) > .5):
        return 'initial_velocity_limit'
    return None


def guard_snapshot(obs, starting=False):
    return {'state_si': list(obs.si()) if obs else None,
            'host_received_at_s': obs.received_at if obs else None,
            'checked_at_s': CLOCK(), 'starting': starting,
            'limits': {'max_abs_x_m': .2,
                       'max_abs_theta_rad': math.radians(10 if starting else 20),
                       'max_abs_vx_m_s': .05 if starting else None,
                       'max_abs_omega_rad_s': .5 if starting else None},
            'reason': state_guard(obs, starting) if obs else 'measurement_unavailable'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--onnx', required=True, type=Path)
    p.add_argument('--contract', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--seconds', type=float, default=.5)
    p.add_argument('--start-confirmed', action='store_true', help='现场明确请求本次运动；绝不自动使能')
    a = p.parse_args()
    if not a.start_confirmed:
        p.error('需要现场明确 --start-confirmed；只读诊断请用 shadow.py')
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= .5:
        p.error('--seconds 必须在(0,0.5]' )
    contract = load_contract(a.contract, a.onnx)
    profile = json.loads(PROFILE.read_text(encoding='utf-8'))
    limits = limits_from_profile(profile)
    if limits.control_hz != 500 or limits.speed_mm_s != 1250:
        raise ValueError('profile_contract_mismatch')
    policy = Policy(a.onnx)
    for _ in range(20):
        policy.forward((0., 0., 0., 0.))
    # 外部权重初始化前后均核验；在运行期间不可改动导出文件。
    verified_files = verify_model_files(a.onnx, contract)
    report = {'verified_model_files': verified_files, 'status': 'starting', 'scope': '限时真机联调；不是硬实时或真机验收',
              'contract': contract, 'clock': vars(time.get_clock_info('perf_counter')),
              'requested_seconds': a.seconds, 'samples': [], 'motion_command_sent': False, 'motion_command_attempted': False}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as output:
        session = receiver = None
        intervals = []
        import serial
        port = serial.Serial(port=None, baudrate=profile['measured']['protocol_baud'], timeout=0, write_timeout=.002)
        port.dtr = port.rts = False
        port.port = a.port
        try:
            with port:
                try:
                    port.reset_input_buffer()
                    transport = ControlTransport(port)
                    session = BoundSession(transport, CLOCK, limits, contract['firmware_coordinates'])
                    receiver = Receiver(transport, session, CLOCK)
                    # 与固件tools一致：先对齐持续遥测行，再发请求，避免丢查询回执。
                    align_end = CLOCK()+.5
                    while CLOCK() < align_end:
                        byte = port.read(1)
                        if byte == b'\n':
                            break
                        if not byte:
                            time.sleep(.0002)
                    else:
                        raise RuntimeError('initial_alignment_timeout')
                    session.observe()
                    end = CLOCK() + 2.
                    # 收集启动读回；不在循环内等待操作者使能，不发送运动。
                    while CLOCK() < end:
                        if not receiver.poll() or session.failure:
                            raise RuntimeError(receiver.error or session.failure)
                        if session.preflight_reason() is None:
                            break
                        time.sleep(.0002)
                    reason = session.preflight_reason()
                    if reason:
                        raise RuntimeError('preflight:' + reason)
                    if session.values['CAL_LENGTH_UM'][0] != profile['measured']['calibrated_travel_um']:
                        raise RuntimeError('profile_travel_mismatch')
                    report['initial_state_check'] = guard_snapshot(session.observation, starting=True)
                    reason = report['initial_state_check']['reason']
                    if reason:
                        raise RuntimeError(reason)
                    if braking_reason(session, 0):
                        raise RuntimeError('initial_braking_envelope')
                    if transport.partial or transport.in_waiting:
                        raise RuntimeError('preflight_receive_incomplete')
                    session.start()
                    start = next_tick = CLOCK()
                    end = start + a.seconds
                    last_tick = last_sample = None
                    last_command_at = start
                    while CLOCK() < end:
                        # 首次原型运行最多0.5秒；运行中不插入元数据查询。
                        if transport.in_waiting > 128:
                            raise RuntimeError('active_backlog')
                        if not receiver.poll() or session.phase != Phase.ACTIVE:
                            raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                        now = CLOCK()
                        if now-last_command_at > .004:
                            raise RuntimeError('command_gap_deadline')
                        if now < next_tick:
                            time.sleep(min(.0002, next_tick-now))
                            continue
                        if now-next_tick > .002:
                            raise RuntimeError('control_deadline')
                        if last_tick is not None:
                            intervals.append(now-last_tick)
                        last_tick = now
                        next_tick = now + .002
                        obs = session.observation
                        if obs is None or now-obs.received_at > .004:
                            raise RuntimeError('observation_late')
                        if obs.received_at == last_sample:
                            continue
                        reason = state_guard(obs)
                        if reason:
                            report['failed_state_check'] = guard_snapshot(obs)
                            raise RuntimeError(reason)
                        last_sample = obs.received_at
                        x, th, vx, w = obs.si()
                        begin = CLOCK()
                        raw = policy.forward((x, contract['angle_sign']*th, vx, contract['angle_sign']*w))
                        inference_s = CLOCK()-begin
                        if CLOCK()-obs.received_at > .004 or CLOCK()-now > .002:
                            raise RuntimeError('inference_deadline')
                        # 新状态可能在推理期间到达；先处理故障/有效性变化，再允许会话发命令。
                        if transport.in_waiting > 128:
                            raise RuntimeError('precommand_backlog')
                        if not receiver.poll() or session.phase != Phase.ACTIVE:
                            raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                        if transport.partial or transport.in_waiting:
                            # 未解析完的尾帧可能是FAULT；不延续上一非零目标等待排空。
                            raise RuntimeError('precommand_receive_incomplete')
                        latest = session.observation
                        if latest is None or state_guard(latest):
                            report['failed_state_check'] = guard_snapshot(latest)
                            raise RuntimeError('latest_state_limit')
                        if CLOCK()-obs.received_at > .004 or CLOCK() >= end:
                            raise RuntimeError('command_deadline')
                        action = max(-1., min(1., raw))
                        # 向上取整绝对目标速度，覆盖Session整数化后的大小。
                        planned_speed = math.ceil(abs(action) * limits.speed_mm_s)
                        if braking_reason(session, planned_speed):
                            raise RuntimeError('braking_envelope')
                        if CLOCK()-last_command_at > .004:
                            raise RuntimeError('prewrite_command_gap_deadline')
                        report['motion_command_attempted'] = True
                        velocity = session.command(action)
                        completed_at = CLOCK()
                        command_gap = completed_at-last_command_at
                        last_command_at = completed_at
                        report['motion_command_sent'] = True
                        report['samples'].append({'elapsed_s': CLOCK()-start, 'state_si': obs.si(),
                                                  'raw_action': raw, 'velocity_mm_s': velocity,
                                                  'inference_s': inference_s,
                                                  'command_gap_s': command_gap})
                        if command_gap > .004:
                            raise RuntimeError('write_completion_deadline')
                    report['status'] = 'trial_ended'
                except KeyboardInterrupt:
                    report['status'] = 'interrupted'
                except Exception as exc:
                    report['status'] = 'error'
                    report['error'] = f'{type(exc).__name__}: {exc}'
                finally:
                    if session is not None and (session.phase in (Phase.ACTIVE, Phase.STOPPING)
                                                or session.mode == ('READY', 'NONE')):
                        report['safe_stop_requested'] = True
                        session.stop()
                        # 使用独立外部截止，异常/第二次Ctrl+C不能触发自动恢复。
                        stop_end = CLOCK()+limits.stop_timeout_s+.1
                        try:
                            while session.phase != Phase.FINISHED and CLOCK() < stop_end:
                                receiver.poll()
                                session.tick()
                                time.sleep(.0002)
                        except BaseException as exc:
                            report['stop_error'] = type(exc).__name__
                    if (session is not None and (report['motion_command_attempted']
                            or report.get('safe_stop_requested')) and not session.disabled_confirmed):
                        # 写超时/短写可能已有字节到达；FINISHED不等于已停车。
                        report['stop_unconfirmed'] = True
                        try:
                            report['last_zero_write_complete'] = transport.write(b'VEL 0\n') == 6
                        except Exception as exc:
                            report['last_zero_write_error'] = type(exc).__name__
                        report['fallback_note'] = '无法确认停车；固件看门狗仅在配置/固件/供电正常时可能兜底，需现场独立停机'
        except Exception as exc:
            report['status'] = 'error'
            report['outer_error'] = f'{type(exc).__name__}: {exc}'
        finally:
            report['loop_interval_s'] = summary(intervals)
            report['disabled_confirmed'] = bool(session and session.disabled_confirmed)
            report['last_observed_firmware_mode'] = session.mode if session else None
            report['stop_unconfirmed'] = bool((report['motion_command_attempted']
                or report.get('safe_stop_requested')) and not report['disabled_confirmed'])
            report['session_failure'] = session.failure if session else None
            report['receive_error'] = receiver.error if receiver else None
            report['status'] = ('complete' if report['status'] == 'trial_ended' and report['samples'] and report['disabled_confirmed']
                                and not report['session_failure'] and not report['receive_error'] else report['status'])
            json.dump(report, output, ensure_ascii=False, indent=2)
            output.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'samples'}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
