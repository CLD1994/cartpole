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


class ReadbackCycle:
    """单组在途的只读刷新；跨控制周期收齐回执，绝不伪造TTL。"""
    groups = (
        ('QUERY', ()),
        ('CALIB?', ('CAL_STATE', 'CAL_LENGTH_UM', 'CAL_SAVED', 'CAL_STORAGE')),
        ('CONFIG?', ('VEL_MAX', 'ACC_MAX', 'WDT_MS', 'WDT_DEC', 'X_MARGIN_UM',
                     'POLE_ZERO_URAD', 'POLE_INV', 'X_INV', 'STEP_LOSS_UM')),
    )

    def __init__(self, transport, history, start):
        self.transport, self.history = transport, history
        self.next_at, self.index, self.pending = start+.15, 0, None

    def update(self, session, now):
        if self.pending is None:
            return
        row = self.pending
        sent = row['sent_clock_s']
        if now-sent >= .1:
            raise RuntimeError('readback_response_deadline')
        command, keys = self.groups[row['group_index']]
        if command == 'QUERY':
            complete = session._mode_at >= sent
        else:
            complete = all(k in session.values and session.values[k][1] >= sent for k in keys)
        if complete:
            row.update(status='complete', completed_clock_s=now, response_s=now-sent)
            self.pending = None

    def send_if_due(self, now, end):
        if self.pending is not None or now < self.next_at or end-now < .02:
            return
        command, keys = self.groups[self.index]
        row = {'command': command, 'group_index': self.index, 'expected_keys': list(keys),
               'sent_clock_s': now, 'status': 'attempted'}
        self.pending = row
        self.history.append(row)
        data = (command+'\n').encode('ascii')
        if self.transport.write(data) != len(data):
            raise RuntimeError('readback_short_write')
        row.update(status='waiting', write_completed_clock_s=CLOCK())
        self.next_at = now+.15
        self.index = (self.index+1) % len(self.groups)


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
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= 5.:
        p.error('--seconds 必须在(0,5]' )
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
    report = {'loop_interval_scope': 'new-observation decision ticks, not serial command completion', 'runner_revision': 3, 'readback_refresh': [], 'scheduler': 'wait-fresh', 'duplicate_tick_count': 0, 'verified_model_files': verified_files, 'status': 'starting', 'scope': '限时真机联调；不是硬实时或真机验收',
              'contract': contract, 'clock': vars(time.get_clock_info('perf_counter')),
              'requested_seconds': a.seconds, 'samples': [], 'motion_command_sent': False, 'motion_command_attempted': False}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as output:
        session = receiver = None
        intervals = []
        stage = 'startup'
        start = last_command_at = next_tick = None
        drain = refresh = None
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
                    if a.seconds > .5:
                        refresh = ReadbackCycle(transport, report['readback_refresh'], start)
                    while CLOCK() < end:
                        # 长窗口单组轮转刷新；4ms命令截止与1秒元数据TTL不变。
                        stage = 'poll'
                        if transport.in_waiting > 128:
                            raise RuntimeError('active_backlog')
                        if not receiver.poll() or session.phase != Phase.ACTIVE:
                            raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                        now = CLOCK()
                        if refresh:
                            refresh.update(session, now)
                        stage = 'gap_check'
                        if now-last_command_at > .004:
                            raise RuntimeError('command_gap_deadline')
                        if now < next_tick:
                            stage = 'wait_tick'
                            time.sleep(min(.0002, next_tick-now))
                            continue
                        stage = 'tick'
                        if now-next_tick > .002:
                            raise RuntimeError('control_deadline')
                        obs = session.observation
                        if obs is None or now-obs.received_at > .004:
                            raise RuntimeError('observation_late')
                        if obs.received_at == last_sample:
                            report['duplicate_tick_count'] += 1
                            stage = 'wait_fresh'
                            time.sleep(min(.0002, max(0., min(last_command_at+.004, obs.received_at+.004, end)-CLOCK())))
                            continue
                        if last_tick is not None:
                            intervals.append(now-last_tick)
                        last_tick = now
                        # 仅新观测消耗一个调度点；重复观测等待期间不推进截止。
                        next_tick = now + .002
                        stage = 'state_guard'
                        reason = state_guard(obs)
                        if reason:
                            report['failed_state_check'] = guard_snapshot(obs)
                            raise RuntimeError(reason)
                        last_sample = obs.received_at
                        x, th, vx, w = obs.si()
                        stage = 'inference'
                        begin = CLOCK()
                        raw = policy.forward((x, contract['angle_sign']*th, vx, contract['angle_sign']*w))
                        inference_s = CLOCK()-begin
                        if CLOCK()-obs.received_at > .004 or CLOCK()-now > .002:
                            raise RuntimeError('inference_deadline')
                        # 分片/新字节可在poll末尾到达；仅在固定原始截止内补收。
                        stage = 'precommand_receive'
                        receive_deadline = min(last_command_at+.004, obs.received_at+.004, end)
                        drain = {'deadline_clock_s': receive_deadline, 'polls': 0,
                                 'wait_s': 0., 'started_clock_s': CLOCK()}
                        while True:
                            if CLOCK() >= receive_deadline:
                                raise RuntimeError('precommand_receive_deadline')
                            if transport.in_waiting > 128:
                                raise RuntimeError('precommand_backlog')
                            if not receiver.poll() or session.phase != Phase.ACTIVE:
                                raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                            drain['polls'] += 1
                            if refresh:
                                refresh.update(session, CLOCK())
                            latest = session.observation
                            if latest is None or state_guard(latest):
                                report['failed_state_check'] = guard_snapshot(latest)
                                raise RuntimeError('latest_state_limit')
                            pending = transport.in_waiting
                            drain.update(pending_bytes=pending, partial=transport.partial,
                                         elapsed_s=CLOCK()-drain['started_clock_s'])
                            if CLOCK() >= receive_deadline:
                                raise RuntimeError('precommand_receive_deadline')
                            if pending > 128:
                                raise RuntimeError('precommand_backlog')
                            drain['readback_in_flight'] = bool(refresh and refresh.pending)
                            if not transport.partial and pending == 0:
                                break
                            if pending == 0:
                                begin = CLOCK()
                                time.sleep(min(.0001, max(0., receive_deadline-begin)))
                                drain['wait_s'] += CLOCK()-begin
                        action = max(-1., min(1., raw))
                        # 向上取整绝对目标速度，覆盖Session整数化后的大小。
                        planned_speed = math.ceil(abs(action) * limits.speed_mm_s)
                        stage = 'braking_guard'
                        if braking_reason(session, planned_speed):
                            raise RuntimeError('braking_envelope')
                        stage = 'prewrite'
                        if CLOCK() >= receive_deadline:
                            raise RuntimeError('prewrite_fixed_deadline')
                        report['motion_command_attempted'] = True
                        velocity = session.command(action)
                        completed_at = CLOCK()
                        command_gap = completed_at-last_command_at
                        last_command_at = completed_at
                        report['motion_command_sent'] = True
                        report['samples'].append({'elapsed_s': CLOCK()-start, 'state_si': obs.si(),
                                                  'raw_action': raw, 'velocity_mm_s': velocity,
                                                  'inference_s': inference_s,
                                                  'command_gap_s': command_gap,
                                                  'precommand_receive': drain,
                                                  'readback_in_flight': dict(refresh.pending) if refresh and refresh.pending else None})
                        stage = 'write_completion'
                        if completed_at >= receive_deadline:
                            raise RuntimeError('write_completion_deadline')
                        if refresh:
                            # 已成功写入新速度后才安排单条只读查询，不与其他查询叠加。
                            stage = 'readback_query'
                            query_at = CLOCK()
                            if query_at >= min(last_command_at+.004, end):
                                raise RuntimeError('readback_send_deadline')
                            refresh.send_if_due(query_at, end)
                            if CLOCK() >= min(last_command_at+.004, end):
                                raise RuntimeError('readback_write_deadline')
                    report['status'] = 'trial_ended'
                except KeyboardInterrupt:
                    report['status'] = 'interrupted'
                except Exception as exc:
                    report['status'] = 'error'
                    report['error'] = f'{type(exc).__name__}: {exc}'
                    # 在停车/串口清理前捕获失败；不读取串口、不重置任何截止。
                    failed_at = CLOCK()
                    failed_obs = session.observation if session else None
                    report['failure_snapshot'] = {'stage': stage, 'clock_s': failed_at,
                        'elapsed_s': failed_at-start if start is not None else None,
                        'actual_command_gap_s': failed_at-last_command_at if last_command_at is not None else None,
                        'last_command_clock_s': last_command_at, 'next_tick_clock_s': next_tick,
                        'observation_received_clock_s': failed_obs.received_at if failed_obs else None,
                        'host_observation_age_s': failed_at-failed_obs.received_at if failed_obs else None,
                        'precommand_receive': drain,
                        'readback_in_flight': dict(refresh.pending) if refresh and refresh.pending else None}
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
