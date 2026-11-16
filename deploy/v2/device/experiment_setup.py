"""当前装置的固定映射与实验输出；训练要求来自模型清单。"""
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
import math
from session import Limits

PROTOCOL_BAUD = 460800
RIG_COORDINATES = {'POLE_INV': 1, 'X_INV': 0}
ANGLE_SIGN = -1


def limits_from_manifest(manifest):
    def milliscale(name):
        value = manifest[name] * 1000
        if not math.isfinite(value) or not math.isclose(value, round(value), rel_tol=0, abs_tol=1e-9):
            raise ValueError('firmware_unit_precision:' + name)
        return round(value)
    return Limits(manifest['training_control_hz'], milliscale('action_scale_m_s'),
                  milliscale('training_acceleration_m_s2'), 10000)


def report_path(model_directory, kind='run'):
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    return Path(model_directory) / 'runs' / f'{kind}-{stamp}-{uuid4().hex}.json'


def result_summary(report, path):
    status = {'complete':'运行完成', 'stopped_by_user':'已手动停止'}.get(report['status'], '运行未完成')
    count = report.get('run_totals', {}).get('command_completions', 0)
    lines = [f'{status}；已完成 {count} 次控制指令。']
    if report.get('error'):
        lines.append('原因：' + report['error'])
    if report.get('diagnostic_timing'):
        lines.append('只读诊断：无真实运动写入，不作为停机验证。')
    else:
        lines.append('停机：' + ('已确认 DISABLED' if report.get('disabled_confirmed') else '未确认 DISABLED，请检查设备状态'))
    lines.append(f'报告：{path}')
    return '\n'.join(lines)
