#!/usr/bin/env python3
"""限时策略联调入口：无自动使能/配置；需要明确模型合同及现场启动请求。"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

from shadow import CLOCK, PROFILE, Policy, summary
from check import limits_from_profile
from model_integrity import verify_model_files
from receiver import Receiver
from session import Session, Phase


CONFIG_KEYS = ('VEL_MAX', 'ACC_MAX', 'WDT_MS', 'WDT_DEC', 'X_MARGIN_UM',
               'POLE_ZERO_URAD', 'POLE_INV', 'X_INV', 'STEP_LOSS_UM')
GRID_PERIOD_NS = 2_000_000
COMMAND_GAP_SOFT_S = .004
COMMAND_GAP_HARD_S = .006
OBSERVATION_MAX_AGE_S = .004


def candidate_refresh_allowed(now, observation_at, last_command_at, end):
    """只允许废弃写入前过期候选；原命令预算与试验结束时间绝不延长。"""
    return (now >= observation_at + OBSERVATION_MAX_AGE_S
            and now < min(last_command_at + COMMAND_GAP_HARD_S, end))


def next_grid_slot(slot, start_ns, completed_ns):
    # 只指向严格未来栅格；不循环补发已错过的slot。
    return max(slot + 1, (completed_ns-start_ns)//GRID_PERIOD_NS + 1)


CONFIG_QUERY_BYTES = tuple(f'CONFIG? {key}\n'.encode('ascii') for key in CONFIG_KEYS)


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
        if data not in (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n', b'DISABLE\n', *CONFIG_QUERY_BYTES):
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
    enforce_position_limits = False  # User-selected firmware-owned position limits.
    def __init__(self, transport, clock, limits, coordinates):
        self.coordinates = coordinates
        super().__init__(transport, clock, limits)

    @property
    def expected_readback(self):
        return {**super().expected_readback, **self.coordinates}


class DiagnosticTransport(ControlTransport):
    """唯一真实写入出口仅允许只读查询；动作/禁用返回虚拟完成。"""
    def write(self, data):
        queries = (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n', *CONFIG_QUERY_BYTES)
        if data in queries:
            return self.port.write(data)
        if data == b'DISABLE\n':
            return len(data)
        try:
            line = data.decode('ascii')
            velocity = int(line[4:-1])
            if line == f'VEL {velocity}\n' and abs(velocity) <= 1250:
                return len(data)
        except (ValueError, UnicodeError):
            pass
        raise ValueError('diagnostic_write_rejected')


class DiagnosticSession(BoundSession):
    required_active_mode = ('DISABLED', 'NONE')

    def receive(self, line):
        super().receive(line)
        if line.split()[:1] == ['STATE'] and hasattr(self, 'diagnostic_state_events'):
            self.diagnostic_state_events.append({'host_parsed_clock_s': CLOCK(),
                'valid': self.observation is not None, 'phase': self.phase.value})


class DiagnosticEnd(Exception):
    pass


class ReadbackCycle:
    """单组在途的只读刷新；跨控制周期收齐回执，绝不伪造TTL。"""
    groups = (
        ('QUERY', ()),
        ('CALIB?', ('CAL_STATE', 'CAL_LENGTH_UM', 'CAL_SAVED', 'CAL_STORAGE')),
        *((f'CONFIG? {key}', (key,)) for key in CONFIG_KEYS),
    )

    def __init__(self, transport, history, start):
        self.transport, self.history = transport, history
        self.next_at, self.index, self.pending = start+.05, 0, None

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
        self.next_at = now+.05
        self.index = (self.index+1) % len(self.groups)


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


def verify_bound_calibration(session):
    # Keep calibration validity/freshness and stable coordinates without a host position gate.
    if session._metadata_reason(CLOCK()):
        raise RuntimeError('position_calibration_not_ready')
    length = session.values['CAL_LENGTH_UM'][0]
    if length != getattr(session, 'bound_travel_um', None):
        raise RuntimeError('position_calibration_changed')


def state_guard(obs, session, starting=False):
    verify_bound_calibration(session)
    x, th, vx, w = obs.si()
    if not all(math.isfinite(v) for v in (x, th, vx, w)):
        return 'nonfinite_state'
    # 仅近直立平衡，不允许下垂起摆；行程保护完全交给当前固件。
    if abs(th) > (math.radians(10) if starting else math.radians(20)):
        return 'trial_angle_limit'
    if starting and (abs(vx) > .05 or abs(w) > .5):
        return 'initial_velocity_limit'
    return None


def guard_snapshot(obs, session, starting=False):
    return {'state_si': list(obs.si()) if obs else None,
            'host_received_at_s': obs.received_at if obs else None,
            'checked_at_s': CLOCK(), 'starting': starting,
            'limits': {'max_abs_x_m': None,
                       'position_owner': 'firmware',
                       'max_abs_theta_rad': math.radians(10 if starting else 20),
                       'max_abs_vx_m_s': .05 if starting else None,
                       'max_abs_omega_rad_s': .5 if starting else None},
            'reason': state_guard(obs, session, starting) if obs else 'measurement_unavailable'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--onnx', required=True, type=Path)
    p.add_argument('--contract', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    duration_args = p.add_mutually_exclusive_group()
    duration_args.add_argument('--seconds', type=float, default=.5)
    duration_args.add_argument('--continuous', action='store_true', help='受监控持续控制，直到Ctrl+C或门控停机；不能无人值守')
    p.add_argument('--diagnostic-timing', action='store_true', help='DISABLED只读同路径诊断；时序/物理门仅记录，真实故障退出')
    p.add_argument('--start-confirmed', action='store_true', help='现场明确请求本次运动；绝不自动使能')
    a = p.parse_args()
    if a.continuous and a.diagnostic_timing:
        p.error('--continuous不能与只读诊断组合')
    if not a.diagnostic_timing and not a.start_confirmed:
        p.error('需要现场明确 --start-confirmed；只读诊断请用 shadow.py')
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= (10. if a.diagnostic_timing else 5.):
        p.error('--seconds 运动在(0,5]，只读诊断在(0,10]' )
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
    report = {'loop_interval_scope': 'new-observation decision ticks, not serial command completion', 'runner_revision': 12, 'diagnostic_timing': a.diagnostic_timing, 'discarded_stale_candidates': 0, 'candidate_refresh_stages': {}, 'stale_observation_waits': 0, 'max_stale_observation_age_s': 0., 'grid_slots_skipped': 0, 'over_period_late_candidates': 0, 'readback_refresh': [], 'scheduler': 'absolute-grid-wait-fresh', 'duplicate_tick_count': 0, 'verified_model_files': verified_files, 'status': 'starting', 'scope': '限时真机联调；不是硬实时或真机验收',
              'timing_limits_s': {'command_gap_soft': COMMAND_GAP_SOFT_S, 'command_gap_hard': COMMAND_GAP_HARD_S, 'observation_max_age': OBSERVATION_MAX_AGE_S},
              'contract': contract, 'clock': vars(time.get_clock_info('perf_counter')),
              'continuous': a.continuous, 'requested_seconds': None if a.continuous else a.seconds, 'samples': [], 'received_states': [], 'inference_attempts_s': [], 'would_stop_count_scope': 'gate invocation counts, not independent faults or failed cycles', 'would_stop': {}, 'virtual_command_attempted': False, 'motion_command_sent': False, 'motion_command_attempted': False}
    if a.continuous:
        report['scope'] = '受监控持续控制，直到用户中断或门控停止；不是无人值守或安全认证'
        for key in ('samples', 'inference_attempts_s'):
            report[key] = deque(maxlen=10000)
        report['readback_refresh'] = deque(maxlen=512)
        report['retention'] = {'samples':10000,'inference_attempts':10000,'intervals':10000,
                               'readback_groups':512,'scope':'recent bounded tail, not complete run'}
    report['run_totals'] = {'command_completions':0,'command_gaps_over_4ms':0,
        'command_gaps_at_or_over_6ms':0,'maximum_command_gap_s':0.,
        'inference_attempts':0,'inference_total_s':0.,'maximum_inference_s':0.}
    if a.diagnostic_timing:
        report['scope'] = 'DISABLED同路径只读诊断；虚拟命令完成不包含真实串口动作写入/设备运动负载'
        report['diagnostic_differences'] = ['仅查询真实写入；VEL/DISABLE虚拟完成',
            '模式门要求DISABLED，物理门及时间超限记录后继续，不代表运动可以继续',
            'FAULT/速度失效/坏帧/元数据过期/接收器错误仍退出',
            '超限后的样本是反事实延续；真实runner会在首次超限停止']
    def gate(reason):
        if not a.diagnostic_timing:
            raise RuntimeError(reason)
        row = report['would_stop'].setdefault(reason, {'count': 0, 'first_clock_s': CLOCK(), 'last_clock_s': None})
        row['count'] += 1
        row['last_clock_s'] = CLOCK()
    def discard_expired_candidate(observation):
        if not candidate_refresh_allowed(CLOCK(), observation.received_at, last_command_at, end):
            return False
        report['discarded_stale_candidates'] += 1
        counts = report['candidate_refresh_stages']
        counts[stage] = counts.get(stage, 0) + 1
        # 不更新last_command_at、last_sample或栅格；外层poll重新核验状态并等待更新帧。
        return True
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as output:
        session = receiver = None
        intervals = deque(maxlen=10000) if a.continuous else []
        stage = 'startup'
        start = end = last_command_at = next_tick = next_slot = None
        drain = refresh = None
        # Startup evidence only; no synchronous disk writes in the active control loop.
        initial_manifest = {k:v for k,v in report.items() if not isinstance(v,deque)}
        json.dump(initial_manifest,output,ensure_ascii=False,indent=2);output.flush()
        import serial
        port = serial.Serial(port=None, baudrate=profile['measured']['protocol_baud'], timeout=0, write_timeout=.002)
        port.dtr = port.rts = False
        port.port = a.port
        try:
            with port:
                try:
                    port.reset_input_buffer()
                    transport = (DiagnosticTransport if a.diagnostic_timing else ControlTransport)(port)
                    session = (DiagnosticSession if a.diagnostic_timing else BoundSession)(transport, CLOCK, limits, contract['firmware_coordinates'])
                    if a.diagnostic_timing:
                        session.diagnostic_state_events = report['received_states']
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
                        if (session.preflight_reason() is None
                                and 'STEP_LOSS_UM' in session.values
                                and not transport.partial and transport.in_waiting == 0):
                            break
                        time.sleep(.0002)
                    reason = session.preflight_reason()
                    if reason:
                        raise RuntimeError('preflight:' + reason)
                    if session.values['CAL_LENGTH_UM'][0] != profile['measured']['calibrated_travel_um']:
                        raise RuntimeError('profile_travel_mismatch')
                    session.bound_travel_um = session.values['CAL_LENGTH_UM'][0]
                    report['position_boundary'] = {
                        'calibrated_travel_um': session.bound_travel_um,
                        'position_owner': 'firmware',
                        'firmware_x_margin_um': limits.margin_um,
                        'firmware_soft_limit_abs_um': session.track_limit_um,
                        'host_position_gate': False,
                        'host_predictive_braking_guard': False}
                    if a.continuous or a.seconds > .5:
                        # 老固件忽略带参数CONFIG?。运动前验证能力，不能在运动中才发现。
                        # 初始整组（包括最后STEP_LOSS）已收齐，旧VEL_MAX不能满足新回执时间。
                        stage = 'single_config_capability'
                        if 'STEP_LOSS_UM' not in session.values:
                            raise RuntimeError('initial_config_dump_incomplete')
                        probe_at = CLOCK()
                        probe = b'CONFIG? VEL_MAX\n'
                        if transport.write(probe) != len(probe):
                            raise RuntimeError('single_config_probe_write')
                        while CLOCK()-probe_at < .2:
                            if not receiver.poll() or session.failure:
                                raise RuntimeError(receiver.error or session.failure)
                            if CLOCK()-probe_at >= .2:
                                raise RuntimeError('single_config_readback_unsupported_or_timeout')
                            entry = session.values.get('VEL_MAX')
                            if (entry and entry[1] > probe_at and entry[0] == limits.speed_mm_s
                                    and not transport.partial and transport.in_waiting == 0):
                                report['single_config_capability'] = {
                                    'verified': True, 'response_s': CLOCK()-probe_at}
                                break
                            time.sleep(.0002)
                        else:
                            raise RuntimeError('single_config_readback_unsupported_or_timeout')
                        reason = session.preflight_reason()
                        if reason:
                            raise RuntimeError('post_probe_preflight:' + reason)
                    report['initial_state_check'] = guard_snapshot(session.observation, session, starting=True)
                    reason = report['initial_state_check']['reason']
                    if reason:
                        gate(reason)
                    if transport.partial or transport.in_waiting:
                        raise RuntimeError('preflight_receive_incomplete')
                    session.start()
                    grid_start_ns = time.perf_counter_ns()
                    start = next_tick = grid_start_ns / 1e9
                    next_slot = 0
                    next_tick_ns = grid_start_ns
                    end = math.inf if a.continuous else start + a.seconds
                    last_tick = last_sample = None
                    last_command_at = start
                    if a.continuous or a.seconds > .5:
                        refresh = ReadbackCycle(transport, report['readback_refresh'], start)
                    while CLOCK() < end:
                        # 长窗口轮转刷新；命令6ms硬限、观测4ms及1秒元数据TTL独立。
                        stage = 'poll'
                        if transport.in_waiting > 128:
                            raise RuntimeError('active_backlog')
                        if not receiver.poll() or session.phase != Phase.ACTIVE:
                            raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                        now_ns = time.perf_counter_ns()
                        now = now_ns / 1e9
                        if refresh:
                            refresh.update(session, now)
                        stage = 'gap_check'
                        if now-last_command_at >= COMMAND_GAP_HARD_S:
                            gate('command_gap_deadline')
                        if now_ns < next_tick_ns:
                            stage = 'wait_tick'
                            time.sleep(min(.0002, next_tick-now))
                            continue
                        stage = 'tick'
                        # 栅格迟到仅记录；实际命令6ms/原观测4ms为独立硬停止条件。
                        obs = session.observation
                        if obs is None:
                            raise RuntimeError('observation_missing')
                        # 已观察到的危险状态必须先停止，不能因缓存旧而等待恢复。
                        stage = 'cached_state_guard'
                        reason = state_guard(obs, session)
                        if reason:
                            report['failed_state_check'] = guard_snapshot(obs, session)
                            gate(reason)
                        if now-obs.received_at > OBSERVATION_MAX_AGE_S:
                            # 尚无推理候选：不用旧缓存，只消耗既有命令预算等新有效帧。
                            # ACTIVE的FAULT/VEL_VALID/metadata失效仍由每次poll立即锁存。
                            stage = 'wait_fresh_observation'
                            report['stale_observation_waits'] += 1
                            report['max_stale_observation_age_s'] = max(
                                report['max_stale_observation_age_s'], now-obs.received_at)
                            remaining = min(last_command_at+COMMAND_GAP_HARD_S, end)-CLOCK()
                            if remaining <= 0:
                                gate('wait_fresh_command_deadline')
                            time.sleep(min(.0001, remaining) if remaining > 0 else .0001)
                            continue
                        if obs.received_at == last_sample:
                            report['duplicate_tick_count'] += 1
                            stage = 'wait_fresh'
                            time.sleep(min(.0002, max(0., min(last_command_at+COMMAND_GAP_HARD_S, obs.received_at+OBSERVATION_MAX_AGE_S, end)-CLOCK())))
                            continue
                        if last_tick is not None:
                            intervals.append(now-last_tick)
                        last_tick = now
                        candidate_slot = next_slot
                        candidate_due_ns = next_tick_ns
                        grid_lateness_s = (now_ns-candidate_due_ns)/1e9
                        if now_ns-candidate_due_ns > GRID_PERIOD_NS:
                            report['over_period_late_candidates'] += 1
                        stage = 'state_guard'
                        reason = state_guard(obs, session)
                        if reason:
                            report['failed_state_check'] = guard_snapshot(obs, session)
                            gate(reason)
                        last_sample = obs.received_at
                        x, th, vx, w = obs.si()
                        stage = 'inference'
                        begin = CLOCK()
                        if begin >= min(last_command_at+COMMAND_GAP_HARD_S, obs.received_at+OBSERVATION_MAX_AGE_S, end):
                            if discard_expired_candidate(obs):
                                continue
                            gate('candidate_deadline')
                            if a.diagnostic_timing and begin >= obs.received_at+OBSERVATION_MAX_AGE_S:
                                continue
                        raw = policy.forward((x, contract['angle_sign']*th, vx, contract['angle_sign']*w))
                        inference_s = CLOCK()-begin
                        report['inference_attempts_s'].append(inference_s)
                        totals=report['run_totals']
                        totals['inference_attempts']+=1
                        totals['inference_total_s']+=inference_s
                        totals['maximum_inference_s']=max(totals['maximum_inference_s'],inference_s)
                        if CLOCK()-now > .002:
                            gate('inference_deadline')
                        if CLOCK()-obs.received_at >= OBSERVATION_MAX_AGE_S:
                            if discard_expired_candidate(obs):
                                continue
                            gate('inference_deadline')
                            if a.diagnostic_timing and CLOCK() >= obs.received_at+OBSERVATION_MAX_AGE_S:
                                continue
                        # 分片/新字节可在poll末尾到达；仅在固定原始截止内补收。
                        stage = 'precommand_receive'
                        receive_deadline = min(last_command_at+COMMAND_GAP_HARD_S, obs.received_at+OBSERVATION_MAX_AGE_S, end)
                        drain = {'deadline_clock_s': receive_deadline, 'polls': 0,
                                 'wait_s': 0., 'started_clock_s': CLOCK()}
                        retry_candidate = False
                        while True:
                            if a.diagnostic_timing and CLOCK() >= end:
                                raise DiagnosticEnd()
                            if CLOCK() >= receive_deadline:
                                if discard_expired_candidate(obs):
                                    retry_candidate = True
                                    break
                                gate('precommand_receive_deadline')
                            if transport.in_waiting > 128:
                                raise RuntimeError('precommand_backlog')
                            if not receiver.poll() or session.phase != Phase.ACTIVE:
                                raise RuntimeError(receiver.error or session.failure or 'session_inactive')
                            drain['polls'] += 1
                            if refresh:
                                refresh.update(session, CLOCK())
                            latest = session.observation
                            if latest is None or state_guard(latest, session):
                                report['failed_state_check'] = guard_snapshot(latest, session)
                                if latest is None:
                                    raise RuntimeError('latest_state_missing')
                                gate('latest_state_limit')
                            pending = transport.in_waiting
                            drain.update(pending_bytes=pending, partial=transport.partial,
                                         elapsed_s=CLOCK()-drain['started_clock_s'])
                            if CLOCK() >= receive_deadline:
                                if discard_expired_candidate(obs):
                                    retry_candidate = True
                                    break
                                gate('precommand_receive_deadline')
                            if pending > 128:
                                raise RuntimeError('precommand_backlog')
                            drain['readback_in_flight'] = bool(refresh and refresh.pending)
                            if not transport.partial and pending == 0:
                                break
                            if pending == 0:
                                begin = CLOCK()
                                time.sleep(min(.0001, max(0., receive_deadline-begin)) if begin < receive_deadline else .0001)
                                drain['wait_s'] += CLOCK()-begin
                        if retry_candidate:
                            continue  # 不使用旧raw；外层拿到更新帧后从头推理。
                        action = max(-1., min(1., raw))
                        stage = 'prewrite'
                        if CLOCK() >= receive_deadline:
                            if discard_expired_candidate(obs):
                                continue
                            gate('prewrite_fixed_deadline')
                            if a.diagnostic_timing and CLOCK() >= obs.received_at+OBSERVATION_MAX_AGE_S:
                                continue
                        report['virtual_command_attempted' if a.diagnostic_timing else 'motion_command_attempted'] = True
                        velocity = session.command(action)
                        completed_ns = time.perf_counter_ns()
                        completed_at = completed_ns / 1e9
                        command_gap = completed_at-last_command_at
                        totals=report['run_totals']
                        totals['command_completions']+=1
                        totals['command_gaps_over_4ms']+=int(command_gap>COMMAND_GAP_SOFT_S)
                        totals['command_gaps_at_or_over_6ms']+=int(command_gap>=COMMAND_GAP_HARD_S)
                        totals['maximum_command_gap_s']=max(totals['maximum_command_gap_s'],command_gap)
                        last_command_at = completed_at
                        if not a.diagnostic_timing:
                            report['motion_command_sent'] = True
                        report['samples'].append({'elapsed_s': CLOCK()-start, 'state_si': obs.si(),
                                                  'raw_action': raw, 'velocity_mm_s': velocity,
                                                  'inference_s': inference_s,
                                                  'command_gap_s': command_gap,
                                                  'command_gap_soft_warning': command_gap > COMMAND_GAP_SOFT_S,
                                                  'observation_received_clock_s': obs.received_at,
                                                  'observation_host_age_s': completed_at-obs.received_at,
                                                  'grid_slot': candidate_slot,
                                                  'grid_due_clock_s': candidate_due_ns/1e9,
                                                  'grid_lateness_s': grid_lateness_s,
                                                  'precommand_receive': drain,
                                                  'readback_in_flight': dict(refresh.pending) if refresh and refresh.pending else None})
                        stage = 'write_completion'
                        if completed_at >= receive_deadline:
                            gate('write_completion_deadline')
                        # 成功写入且通过硬截止后才推进；短间隔相位恢复并非固定周期保证。
                        next_slot = next_grid_slot(candidate_slot, grid_start_ns, completed_ns)
                        skipped = next_slot-candidate_slot-1
                        report['grid_slots_skipped'] += skipped
                        report['samples'][-1]['skipped_following_slots'] = skipped
                        next_tick_ns = grid_start_ns + next_slot*GRID_PERIOD_NS
                        next_tick = next_tick_ns/1e9
                        if refresh:
                            # 已成功写入新速度后才安排单条只读查询，不与其他查询叠加。
                            stage = 'readback_query'
                            query_at = CLOCK()
                            if query_at >= min(last_command_at+COMMAND_GAP_HARD_S, end):
                                gate('readback_send_deadline')
                            refresh.send_if_due(query_at, end)
                            if CLOCK() >= min(last_command_at+COMMAND_GAP_HARD_S, end):
                                gate('readback_write_deadline')
                    report['status'] = 'trial_ended'
                except DiagnosticEnd:
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
                        'grid_slot': next_slot,
                        'grid_lateness_s': failed_at-next_tick if next_tick is not None else None,
                        'observation_received_clock_s': failed_obs.received_at if failed_obs else None,
                        'host_observation_age_s': failed_at-failed_obs.received_at if failed_obs else None,
                        'precommand_receive': drain,
                        'readback_in_flight': dict(refresh.pending) if refresh and refresh.pending else None}
                finally:
                    report['active_elapsed_s']=CLOCK()-start if start is not None else None
                    if not a.diagnostic_timing and session is not None and (session.phase in (Phase.ACTIVE, Phase.STOPPING)
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
            report['received_state_count'] = len(report['received_states']) if a.diagnostic_timing else None
            report['received_state_scope'] = 'all parsed STATE lines including startup; host time, not device sampling time' if a.diagnostic_timing else 'not_recorded'
            measured_states = [r for r in report['received_states'] if start is not None and end is not None and start <= r['host_parsed_clock_s'] < end and r['valid'] and r['phase'] == 'active']
            report['received_state_measurement_count'] = len(measured_states) if a.diagnostic_timing else None
            report['received_state_host_gap_s'] = summary([b['host_parsed_clock_s']-a['host_parsed_clock_s'] for a,b in zip(measured_states,measured_states[1:])])
            report['received_state_gap_scope'] = 'valid ACTIVE STATE parses in measurement window; not device sample/wire arrival timing' if a.diagnostic_timing else 'not_recorded'
            report['summary_scope']='retained recent tail; run_totals are full-run' if a.continuous else 'complete recorded run'
            report['loop_interval_s'] = summary(intervals)
            report['completion_gap_s'] = summary([r['command_gap_s'] for r in report['samples']])
            report['completion_gap_scope'] = 'virtual_no_action_write' if a.diagnostic_timing else 'actual_host_write_completion'
            report['completion_gaps_over_6ms'] = sum(r['command_gap_s'] >= COMMAND_GAP_HARD_S for r in report['samples'])
            report['completion_gaps_over_4ms'] = sum(r['command_gap_s'] > COMMAND_GAP_SOFT_S for r in report['samples'])
            report['inference_s'] = summary(report['inference_attempts_s'])
            report['inference_scope'] = ('retained recent policy.forward attempts including discarded candidates; excludes warmup' if a.continuous else 'all measured policy.forward attempts including discarded candidates; excludes warmup')
            report['completed_candidate_inference_s'] = summary([r['inference_s'] for r in report['samples']])
            report['disabled_confirmed'] = bool(session and session.disabled_confirmed)
            report['last_observed_firmware_mode'] = session.mode if session else None
            report['stop_unconfirmed'] = bool((report['motion_command_attempted']
                or report.get('safe_stop_requested')) and not report['disabled_confirmed'])
            report['session_failure'] = session.failure if session else None
            report['receive_error'] = receiver.error if receiver else None
            report['status'] = ('complete' if report['status'] == 'trial_ended' and report['samples'] and (report['disabled_confirmed'] or a.diagnostic_timing)
                                and not report['session_failure'] and not report['receive_error'] else report['status'])
            if (a.continuous and report['status']=='interrupted' and report['disabled_confirmed']
                    and not report['session_failure'] and not report['receive_error']):
                report['status']='stopped_by_user'
            for key,value in list(report.items()):
                if isinstance(value,deque):report[key]=list(value)
            output.seek(0);output.truncate()
            json.dump(report, output, ensure_ascii=False, indent=2)
            output.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'received_states', 'inference_attempts_s')}, ensure_ascii=False, indent=2))
    return 0 if report['status'] in ('complete','stopped_by_user') else 1


if __name__ == '__main__':
    raise SystemExit(main())
