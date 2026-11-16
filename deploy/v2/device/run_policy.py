#!/usr/bin/env python3
"""单动作实验入口：模型目录包含训练清单，设备标定自动读取；不自动使能或配置。"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
import sys

from shadow import CLOCK, Policy, summary
from experiment_setup import (limits_from_manifest, PROTOCOL_BAUD, RIG_COORDINATES,
                              ANGLE_SIGN, report_path, result_summary)
from model_integrity import verify_model_files
from receiver import Receiver
from session import Session, Phase
from step_loss_events import StepLossEventMixin
from angle_pause import AnglePause


CONFIG_KEYS = ('VEL_MAX', 'ACC_MAX', 'WDT_MS', 'X_MARGIN_UM',
               'POLE_ZERO_URAD', 'POLE_INV', 'X_INV', 'STEP_LOSS_UM')
GRID_PERIOD_NS = 2_000_000
COMMAND_GAP_SOFT_S = .004
COMMAND_GAP_HARD_S = .006
OBSERVATION_MAX_AGE_S = .004


def timing_limits(profile):
    if profile == 'normal':
        return COMMAND_GAP_HARD_S, OBSERVATION_MAX_AGE_S, .002
    if profile == 'debug':
        return .015, .012, None
    raise ValueError('unknown_timing_profile')


def candidate_refresh_allowed(now, observation_at, last_command_at, end,
                              observation_max_age=OBSERVATION_MAX_AGE_S,
                              command_gap_hard=COMMAND_GAP_HARD_S):
    """只允许废弃写入前过期候选；原命令预算与试验结束时间绝不延长。"""
    return (now >= observation_at + observation_max_age
            and now < min(last_command_at + command_gap_hard, end))


def next_grid_slot(slot, start_ns, completed_ns, period_ns=GRID_PERIOD_NS):
    # 只指向严格未来栅格；不循环补发已错过的slot。
    return max(slot + 1, (completed_ns-start_ns)//period_ns + 1)


CONFIG_QUERY_BYTES = tuple(f'CONFIG? {key}\n'.encode('ascii') for key in CONFIG_KEYS)


def valid_motion_line(line):
    words = line.split()
    try:
        if len(words) == 2 and words[0] == 'VEL':
            v = int(words[1])
            return line == f'VEL {v}\n' and abs(v) <= 1250
    except ValueError:
        pass
    return False


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
        if data not in (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n', b'DISABLE\n', b'RUN?\n', *CONFIG_QUERY_BYTES):
            try:
                line = data.decode('ascii')
                valid = valid_motion_line(line)
            except (ValueError, UnicodeError):
                valid = False
            if not valid:
                raise ValueError('control_write_rejected')
        return self.port.write(data)


class BoundSession(Session):
    """把本次台位坐标参数纳入既有新鲜读回及运行中变化检查。"""
    enforce_position_limits = False  # User-selected firmware-owned position limits.
    def __init__(self, transport, clock, limits, coordinates):
        self.coordinates = dict(coordinates)
        super().__init__(transport, clock, limits)

    @property
    def expected_readback(self):
        return {**super().expected_readback, **self.coordinates}

    @property
    def readback_names(self):
        return super().readback_names | {'POLE_ZERO_URAD'}

    def _metadata_reason(self, now):
        reason = super()._metadata_reason(now)
        if reason:
            return reason
        zero = self.values.get('POLE_ZERO_URAD')
        if (zero is None or not -3141592 <= zero[0] <= 3141592
                or not self._fresh(zero[1], now, self.limits.preflight_timeout_s)):
            return 'readback_POLE_ZERO_URAD'
        return None

    def bind_calibration(self):
        reason = self._metadata_reason(self._now())
        if reason:
            raise RuntimeError('preflight:' + reason)
        self.coordinates['POLE_ZERO_URAD'] = self.values['POLE_ZERO_URAD'][0]
        self.bound_travel_um = self.values['CAL_LENGTH_UM'][0]
        return {key: entry[0] for key, entry in self.values.items()}


class ProbeSession(StepLossEventMixin, BoundSession):
    pass


class DiagnosticTransport(ControlTransport):
    """唯一真实写入出口仅允许只读查询；动作/禁用返回虚拟完成。"""
    def write(self, data):
        queries = (b'QUERY\n', b'CALIB?\n', b'CONFIG?\n', b'RUN?\n', *CONFIG_QUERY_BYTES)
        if data in queries:
            return self.port.write(data)
        if data == b'DISABLE\n':
            return len(data)
        try:
            line = data.decode('ascii')
            if valid_motion_line(line):
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


class TrialEnd(Exception):
    pass


def check_trial_end(now, end):
    if now >= end:
        raise TrialEnd()


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
    # 启动和运行不以摆角幅度拒绝控制；仍核验有效状态与启动速度。
    if starting and (abs(vx) > .05 or abs(w) > .5):
        return 'initial_velocity_limit'
    return None


def guard_snapshot(obs, session, starting=False):
    return {'state_si': list(obs.si()) if obs else None,
            'host_received_at_s': obs.received_at if obs else None,
            'checked_at_s': CLOCK(), 'starting': starting,
            'limits': {'max_abs_x_m': None,
                       'position_owner': 'firmware',
                       'max_abs_theta_rad': None,
                       'angle_limit_scope': 'terminal_gate_disabled; pause_resume_thresholds_in_report_angle_pause',
                       'max_abs_vx_m_s': .05 if starting else None,
                       'max_abs_omega_rad_s': .5 if starting else None},
            'reason': state_guard(obs, session, starting) if obs else 'measurement_unavailable'}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--model', required=True, type=Path, help='含 policy.onnx 与 model.json 的导出目录')
    p.add_argument('--output', type=Path, help='完整报告；默认在模型 runs 目录生成唯一文件')
    p.add_argument('--acceleration-limit-mm-s2', type=int, help='显式部署加速度上限，精确核验固件读回；不修改模型训练参数')
    p.add_argument('--velocity-limit-mm-s', type=int, help='显式降低部署速度上限；保留模型动作倍率并饱和限幅，不修改固件配置')
    p.add_argument('--angle-pause-deg', type=float, default=60., help='绝对摆角达到此角度时暂停速度命令')
    p.add_argument('--angle-resume-deg', type=float, default=58., help='绝对摆角回落至此角度时自动重新推理')
    p.add_argument('--command-gap-hard-ms', type=float, choices=(6., 9.), help='显式覆盖命令硬期限；不改变观测或推理期限')
    p.add_argument('--json', action='store_true', help='终端输出机器可读摘要 JSON')
    duration_args = p.add_mutually_exclusive_group()
    duration_args.add_argument('--seconds', type=float, default=5.)
    duration_args.add_argument('--continuous', action='store_true', help='受监控持续控制，直到Ctrl+C或门控停机；不能无人值守')
    event_args = p.add_mutually_exclusive_group()
    event_args.add_argument('--step-loss-monitor', action='store_true', help='仅记录STEP_LOSS事件及有界前后样本，模型持续控制，不切低速或按事件停车')
    p.add_argument('--diagnostic-timing', action='store_true', help='DISABLED只读同路径诊断；时序/物理门仅记录，真实故障退出')
    p.add_argument('--start-confirmed', action='store_true', help='现场明确请求本次运动；绝不自动使能')
    p.add_argument('--timing-profile', choices=('normal', 'debug'), default='normal',
                   help='debug显式采用15ms命令/12ms观测期限；不是硬实时保证')
    a = p.parse_args(argv)
    try:
        AnglePause(a.angle_pause_deg, a.angle_resume_deg)
    except ValueError:
        p.error('摆角必须满足 0 <= resume < pause <= 180')
    event_diag = a.step_loss_monitor
    if a.continuous and a.diagnostic_timing:
        p.error('--continuous不能与只读诊断组合')
    if not a.diagnostic_timing and not a.start_confirmed:
        p.error('需要现场明确 --start-confirmed；只读诊断请用 shadow.py')
    if not math.isfinite(a.seconds) or not 0 < a.seconds <= (10. if a.diagnostic_timing else 5.):
        p.error('--seconds 运动在(0,5]，只读诊断在(0,10]' )
    if event_diag and a.diagnostic_timing:
        p.error('--step-loss-monitor不能与只读诊断组合')
    a.output = a.output or report_path(a.model)
    return a


def main(a=None):
    from model_manifest import load_manifest
    a = a or parse_args()
    a.serial_open_attempted = False
    event_diag = a.step_loss_monitor
    model = a.model / 'policy.onnx'
    manifest = load_manifest(a.model / 'model.json', model)
    hard_limit_s, observation_limit_s, inference_limit_s = timing_limits(a.timing_profile)
    if a.command_gap_hard_ms is not None:
        hard_limit_s = a.command_gap_hard_ms / 1000.
    pause = AnglePause(a.angle_pause_deg, a.angle_resume_deg)
    period_ns = 1_000_000_000 // manifest['training_control_hz']
    limits = limits_from_manifest(manifest, velocity_limit_mm_s=getattr(a, "velocity_limit_mm_s", None),
                                  acceleration_limit_mm_s2=getattr(a, "acceleration_limit_mm_s2", None))
    policy = Policy(model)
    for _ in range(20):
        policy.forward((0., 0., 0., 0.))
    # 外部权重初始化前后均核验；在运行期间不可改动导出文件。
    verified_files = verify_model_files(model, manifest)
    report = {'loop_interval_scope': 'new-observation decision ticks, not serial command completion', 'runner_revision': 18, 'timing_profile': a.timing_profile, 'control_hz': limits.control_hz, 'grid_period_ns': period_ns, 'action_dim': 1, 'host_angle_gate': not a.diagnostic_timing, 'host_angle_gate_mode': 'disabled_diagnostic' if a.diagnostic_timing else 'pause_resume', 'diagnostic_timing': a.diagnostic_timing, 'discarded_stale_candidates': 0, 'candidate_refresh_stages': {}, 'stale_observation_waits': 0, 'max_stale_observation_age_s': 0., 'grid_slots_skipped': 0, 'over_period_late_candidates': 0, 'readback_refresh': [], 'scheduler': 'absolute-grid-wait-fresh', 'duplicate_tick_count': 0, 'verified_model_files': verified_files, 'status': 'starting', 'scope': '限时真机联调；不是硬实时或真机验收',
              'timing_limits_s': {'command_gap_soft': COMMAND_GAP_SOFT_S, 'command_gap_hard': hard_limit_s, 'observation_max_age': observation_limit_s},
              'rig_requirements': {'protocol_baud': PROTOCOL_BAUD, 'config': limits.config,
                                   'coordinates': dict(RIG_COORDINATES), 'angle_sign': ANGLE_SIGN,
                                   'x_sign': 1, 'action_sign': 1},
              'deployment_acceleration': {'training_mm_s2': limits.acceleration_mm_s2, 'firmware_limit_mm_s2': limits.firmware_acceleration_mm_s2, 'distribution_shift': limits.firmware_acceleration_mm_s2 != limits.acceleration_mm_s2}, 'deployment_velocity': {'model_scale_mm_s': limits.speed_mm_s, 'firmware_limit_mm_s': limits.firmware_speed_mm_s, 'mapping': 'training_scale_then_saturation', 'distribution_shift': limits.firmware_speed_mm_s != limits.speed_mm_s}, 'model_manifest': manifest, 'clock': vars(time.get_clock_info('perf_counter')),
              'continuous': a.continuous, 'requested_seconds': None if a.continuous else a.seconds, 'samples': [], 'received_states': [], 'inference_attempts_s': [], 'would_stop_count_scope': 'gate invocation counts, not independent faults or failed cycles', 'would_stop': {}, 'virtual_command_attempted': False, 'motion_command_sent': False, 'motion_command_attempted': False}
    if a.step_loss_monitor:
        report['step_loss_monitor_config'] = {'model_continues': True, 'event_run_matches_current_firmware_run_verified': False, 'event_wait_timeout_s': None, 'pre_event_sample_limit': 400, 'post_event_sample_limit': 2000, 'scope': '首个同run事件的控制决策样本窗口；非全部串口STATE、非固定物理时长；完成后不被尾缓冲覆盖'}
        report['scope'] = 'STEP_LOSS仅记录；固件仍须READY，FAULT和时序失败仍停车'
    if a.continuous:
        if not event_diag:
            report['scope'] = '受监控持续控制，直到用户中断或门控停止；不是无人值守或安全认证'
        for key in ('samples', 'inference_attempts_s'):
            report[key] = deque(maxlen=10000)
        report['readback_refresh'] = deque(maxlen=512)
        report['retention'] = {'samples':10000,'inference_attempts':10000,'intervals':10000,
                               'readback_groups':512,'scope':'recent bounded tail, not complete run'}
    report['angle_pause'] = {'enabled': not a.diagnostic_timing,
        'enter_deg': a.angle_pause_deg, 'resume_deg': a.angle_resume_deg,
        'events': [],
        'pause_count': 0, 'resume_count': 0, 'intentional_completion_gaps': 0}
    report['run_totals'] = {'command_completions':0,'command_gaps_over_4ms':0,
        'command_gaps_at_or_over_6ms':0,'command_gaps_at_or_over_hard_limit':0,'maximum_command_gap_s':0.,'maximum_active_command_gap_s':0.,
        'command_gap_scope':'maximum_command_gap_s includes intentional pauses; active maximum and over-limit counts exclude marked pause gaps',
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
    def update_pause(observation, now):
        nonlocal budget_at
        if a.diagnostic_timing:
            return None
        transition = pause.check(observation, now, budget_at, hard_limit_s, observation_limit_s)
        if transition:
            info = report['angle_pause']
            info['pause_count' if transition == 'paused' else 'resume_count'] += 1
            info['events'].append({'event': transition, 'elapsed_s': now-start,
                                   'abs_angle_deg': abs(math.degrees(observation.si()[1]))})
            del info['events'][:-512]
            if transition == 'resumed':
                budget_at = now
        return transition
    def discard_expired_candidate(observation):
        check_trial_end(CLOCK(), end)
        if not candidate_refresh_allowed(CLOCK(), observation.received_at, budget_at, end,
                                         observation_limit_s, hard_limit_s):
            return False
        report['discarded_stale_candidates'] += 1
        counts = report['candidate_refresh_stages']
        counts[stage] = counts.get(stage, 0) + 1
        # 不更新last_command_at、last_sample或栅格；外层poll重新核验状态并等待更新帧。
        return True
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open('x', encoding='utf-8') as output:
        session = receiver = probe = None
        intervals = deque(maxlen=10000) if a.continuous else []
        stage = 'startup'
        start = end = last_command_at = budget_at = next_tick = next_slot = None
        drain = refresh = None
        # Startup evidence only; no synchronous disk writes in the active control loop.
        initial_manifest = {k:v for k,v in report.items() if not isinstance(v,deque)}
        json.dump(initial_manifest,output,ensure_ascii=False,indent=2);output.flush()
        import serial
        port = serial.Serial(port=None, baudrate=PROTOCOL_BAUD, timeout=0, write_timeout=.002)
        port.dtr = port.rts = False
        port.port = a.port
        try:
            a.serial_open_attempted = True
            with port:
                try:
                    port.reset_input_buffer()
                    transport = (DiagnosticTransport if a.diagnostic_timing else ControlTransport)(port)
                    session_class = DiagnosticSession if a.diagnostic_timing else ProbeSession if event_diag else BoundSession
                    session = session_class(transport, CLOCK, limits, RIG_COORDINATES)
                    probe = session.step_loss_probe if event_diag else None
                    if probe and a.step_loss_monitor:
                        probe.enable_monitor()
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
                    report['device_snapshot'] = session.bind_calibration()
                    report['coordinate_mapping'] = {'x_sign': 1, 'action_sign': 1, 'angle_sign': ANGLE_SIGN}
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
                        config_probe = b'CONFIG? VEL_MAX\n'
                        if transport.write(config_probe) != len(config_probe):
                            raise RuntimeError('single_config_probe_write')
                        while CLOCK()-probe_at < .2:
                            if not receiver.poll() or session.failure:
                                raise RuntimeError(receiver.error or session.failure)
                            if CLOCK()-probe_at >= .2:
                                raise RuntimeError('single_config_readback_unsupported_or_timeout')
                            entry = session.values.get('VEL_MAX')
                            if (entry and entry[1] > probe_at and entry[0] == limits.firmware_speed_mm_s
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
                    if event_diag:
                        session.run_query_pending = True
                        if transport.write(b'RUN?\n') != 5:
                            raise RuntimeError('run_query_write')
                        run_deadline = CLOCK() + .2
                        while session.run_reply is None and CLOCK() < run_deadline:
                            if not receiver.poll() or session.failure:
                                raise RuntimeError(receiver.error or session.failure)
                            time.sleep(.0002)
                        if session.run_reply is None:
                            raise RuntimeError('run_query_unsupported_or_timeout')
                        report['verified_firmware_run_id'] = session.run_reply
                        report['step_loss_monitor_config']['event_run_matches_current_firmware_run_verified'] = True
                        if not a.json:
                            print('事件记录：同 run STEP_LOSS 仅记录；FAULT 仍停车，Ctrl+C 停止。', flush=True)
                    if not a.json:
                        duration = '持续运行，Ctrl+C 停止' if a.continuous else f'{a.seconds:g} 秒'
                        print(f'预检通过：{limits.control_hz:g} Hz，{limits.speed_mm_s} mm/s 模型倍率，{limits.firmware_speed_mm_s} mm/s 部署限幅，{limits.firmware_acceleration_mm_s2} mm/s² 部署加速度（训练 {limits.acceleration_mm_s2}）；{duration}。', flush=True)
                        print(f'报告：{a.output}', flush=True)
                    session.start()
                    grid_start_ns = time.perf_counter_ns()
                    start = next_tick = grid_start_ns / 1e9
                    next_slot = 0
                    next_tick_ns = grid_start_ns
                    end = math.inf if a.continuous else start + a.seconds
                    last_tick = last_sample = None
                    last_command_at = budget_at = start
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
                        stage = 'angle_pause_check'
                        # 非暂停仍先检查原命令预算；暂停只豁免故意停发的间隔。
                        transition = update_pause(session.observation, now)
                        if transition == 'resumed':
                            next_slot = next_grid_slot(next_slot, grid_start_ns, now_ns, period_ns)
                            next_tick_ns = grid_start_ns + next_slot*period_ns
                            next_tick = next_tick_ns/1e9
                            last_sample = last_tick = None
                        if pause.paused and not a.diagnostic_timing:
                            stage = 'angle_paused'
                            if state_guard(session.observation, session):
                                raise RuntimeError('paused_state_guard')
                            if refresh:
                                refresh.send_if_due(now, end)
                            # 读回写入也不允许遮蔽新的观测超时。
                            update_pause(session.observation, CLOCK())
                            check_trial_end(CLOCK(), end)
                            time.sleep(.0001)
                            continue
                        stage = 'gap_check'
                        if now-budget_at >= hard_limit_s:
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
                        if now-obs.received_at > observation_limit_s:
                            # 尚无推理候选：不用旧缓存，只消耗既有命令预算等新有效帧。
                            # ACTIVE的FAULT/STATE双路有效性/metadata失效仍由每次poll立即锁存。
                            stage = 'wait_fresh_observation'
                            report['stale_observation_waits'] += 1
                            report['max_stale_observation_age_s'] = max(
                                report['max_stale_observation_age_s'], now-obs.received_at)
                            remaining = min(budget_at+hard_limit_s, end)-CLOCK()
                            if remaining <= 0:
                                if CLOCK() >= budget_at+hard_limit_s:
                                    gate('wait_fresh_command_deadline')
                                check_trial_end(CLOCK(), end)
                            time.sleep(min(.0001, remaining) if remaining > 0 else .0001)
                            continue
                        if obs.received_at == last_sample:
                            report['duplicate_tick_count'] += 1
                            stage = 'wait_fresh'
                            time.sleep(min(.0002, max(0., min(budget_at+hard_limit_s, obs.received_at+observation_limit_s, end)-CLOCK())))
                            continue
                        if last_tick is not None:
                            intervals.append(now-last_tick)
                        last_tick = now
                        candidate_slot = next_slot
                        candidate_due_ns = next_tick_ns
                        grid_lateness_s = (now_ns-candidate_due_ns)/1e9
                        if now_ns-candidate_due_ns > period_ns:
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
                        check_trial_end(begin, end)
                        if begin >= min(budget_at+hard_limit_s, obs.received_at+observation_limit_s, end):
                            if discard_expired_candidate(obs):
                                continue
                            gate('candidate_deadline')
                            if a.diagnostic_timing and begin >= obs.received_at+observation_limit_s:
                                continue
                        raw = None
                        inference_s = None
                        raw = policy.forward((x, ANGLE_SIGN*th, vx, ANGLE_SIGN*w))
                        inference_s = CLOCK()-begin
                        report['inference_attempts_s'].append(inference_s)
                        totals=report['run_totals']
                        totals['inference_attempts']+=1
                        totals['inference_total_s']+=inference_s
                        totals['maximum_inference_s']=max(totals['maximum_inference_s'],inference_s)
                        if inference_limit_s is not None and CLOCK()-now > inference_limit_s:
                            gate('inference_deadline')
                        if CLOCK()-obs.received_at >= observation_limit_s:
                            if discard_expired_candidate(obs):
                                continue
                            gate('inference_deadline')
                            if a.diagnostic_timing and CLOCK() >= obs.received_at+observation_limit_s:
                                continue
                        # 分片/新字节可在poll末尾到达；仅在固定原始截止内补收。
                        stage = 'precommand_receive'
                        receive_deadline = min(budget_at+hard_limit_s, obs.received_at+observation_limit_s, end)
                        drain = {'deadline_clock_s': receive_deadline, 'polls': 0,
                                 'wait_s': 0., 'started_clock_s': CLOCK()}
                        retry_candidate = False
                        while True:
                            check_trial_end(CLOCK(), end)
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
                            if update_pause(latest, CLOCK()) == 'paused':
                                retry_candidate = True
                                break  # 已发生暂停：不发送旧候选，恢复后重新推理。
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
                            if a.diagnostic_timing and CLOCK() >= obs.received_at+observation_limit_s:
                                continue
                        report['virtual_command_attempted' if a.diagnostic_timing else 'motion_command_attempted'] = True
                        acceleration = None
                        command_phase = 'model'
                        velocity = session.command(action)
                        completed_ns = time.perf_counter_ns()
                        completed_at = completed_ns / 1e9
                        command_gap = completed_at-last_command_at
                        totals=report['run_totals']
                        totals['command_completions']+=1
                        intentional_gap = pause.resumed_since_command and not a.diagnostic_timing
                        pause.resumed_since_command = False
                        if intentional_gap:
                            report['angle_pause']['intentional_completion_gaps'] += 1
                        totals['command_gaps_over_4ms']+=int(not intentional_gap and command_gap>COMMAND_GAP_SOFT_S)
                        totals['command_gaps_at_or_over_6ms']+=int(not intentional_gap and command_gap>=.006)
                        totals['command_gaps_at_or_over_hard_limit']+=int(not intentional_gap and command_gap>=hard_limit_s)
                        totals['maximum_command_gap_s']=max(totals['maximum_command_gap_s'],command_gap)
                        if not intentional_gap:
                            totals['maximum_active_command_gap_s']=max(totals['maximum_active_command_gap_s'],command_gap)
                        last_command_at = budget_at = completed_at
                        if not a.diagnostic_timing:
                            report['motion_command_sent'] = True
                        report['samples'].append({'elapsed_s': CLOCK()-start, 'state_si': obs.si(),
                                                  'control_source': command_phase, 'raw_action': raw, 'velocity_mm_s': velocity, 'acceleration_mm_s2': acceleration,
                                                  'inference_s': inference_s,
                                                  'command_gap_s': command_gap,
                                                  'intentional_angle_pause_gap': intentional_gap,
                                                  'command_gap_soft_warning': not intentional_gap and command_gap > COMMAND_GAP_SOFT_S,
                                                  'observation_received_clock_s': obs.received_at,
                                                  'host_observation_age_s': completed_at-obs.received_at,
                                                  'grid_slot': candidate_slot,
                                                  'grid_due_clock_s': candidate_due_ns/1e9,
                                                  'grid_lateness_s': grid_lateness_s,
                                                  'precommand_receive': drain,
                                                  'readback_in_flight': dict(refresh.pending) if refresh and refresh.pending else None})
                        if probe:
                            probe.record_sample(report['samples'][-1])
                        stage = 'write_completion'
                        if completed_at >= receive_deadline:
                            gate('write_completion_deadline')
                        # 成功写入且通过硬截止后才推进；短间隔相位恢复并非固定周期保证。
                        next_slot = next_grid_slot(candidate_slot, grid_start_ns, completed_ns, period_ns)
                        skipped = next_slot-candidate_slot-1
                        report['grid_slots_skipped'] += skipped
                        report['samples'][-1]['skipped_following_slots'] = skipped
                        next_tick_ns = grid_start_ns + next_slot*period_ns
                        next_tick = next_tick_ns/1e9
                        if refresh:
                            # 已成功写入新速度后才安排单条只读查询，不与其他查询叠加。
                            stage = 'readback_query'
                            query_at = CLOCK()
                            if query_at >= budget_at+hard_limit_s:
                                gate('readback_send_deadline')
                            check_trial_end(query_at, end)
                            refresh.send_if_due(query_at, end)
                            query_done = CLOCK()
                            if query_done >= budget_at+hard_limit_s:
                                gate('readback_write_deadline')
                            check_trial_end(query_done, end)
                    report['status'] = 'trial_ended'
                except TrialEnd:
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
            report['completion_gap_s'] = summary([r['command_gap_s'] for r in report['samples'] if not r.get('intentional_angle_pause_gap')])
            report['completion_gap_scope'] = 'virtual_no_action_write' if a.diagnostic_timing else 'actual_host_write_completion_excluding_marked_angle_pause_gaps'
            report['completion_gaps_over_6ms'] = sum(r['command_gap_s'] >= .006 for r in report['samples'] if not r.get('intentional_angle_pause_gap'))
            report['completion_gaps_at_or_over_hard_limit'] = sum(r['command_gap_s'] >= hard_limit_s for r in report['samples'] if not r.get('intentional_angle_pause_gap'))
            report['completion_gaps_over_4ms'] = sum(r['command_gap_s'] > COMMAND_GAP_SOFT_S for r in report['samples'] if not r.get('intentional_angle_pause_gap'))
            report['inference_s'] = summary(report['inference_attempts_s'])
            report['inference_scope'] = ('retained recent policy.forward attempts including discarded candidates; excludes warmup' if a.continuous else 'all measured policy.forward attempts including discarded candidates; excludes warmup')
            report['completed_candidate_inference_s'] = summary([r['inference_s'] for r in report['samples'] if r['inference_s'] is not None])
            if probe is not None:
                report['step_loss_monitor'] = probe.report()
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
    if a.json:
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'received_states', 'inference_attempts_s')}, ensure_ascii=False, indent=2))
    else:
        print(result_summary(report, a.output))
    return 0 if report['status'] in ('complete','stopped_by_user') else 1


def entrypoint(argv=None):
    args = parse_args(argv)
    args.serial_open_attempted = False
    try:
        return main(args)
    except Exception as exc:
        # 正常控制异常由 main 保留报告并停车；这里处理初始化/报告文件等外围失败。
        error = f'{type(exc).__name__}: {exc}'
        opened = args.serial_open_attempted
        if args.json:
            print(json.dumps({'status': 'error', 'error': error,
                              'serial_open_attempted': opened}, ensure_ascii=False))
        else:
            state = '串口已尝试打开，请检查设备状态' if opened else '未打开串口，未发送运动指令'
            print(f'运行失败：{error}\n{state}。', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(entrypoint())
