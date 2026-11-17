"""评估允许显式覆盖的执行器配置；与训练使用相同的配置解析入口。"""
import argparse
import json
import math


def environment_override(text):
    """校验单项数值覆盖，不接受其他配置域或 Hydra 表达式。"""
    key, separator, value = text.partition('=')
    scalar = {f'env.actions.joint_effort.{name}': name for name in ('vel_max', 'acc_max', 'tau')}
    ranges = {f'env.events.randomize_tracking.params.{name}_range': name
              for name in ('vel_max', 'acc_max', 'tau')}
    if not separator or key not in scalar | ranges:
        raise argparse.ArgumentTypeError('仅允许动作 vel_max、acc_max、tau 及对应重置随机范围')
    try:
        parsed = json.loads(value, parse_int=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('覆盖值必须为数值或二元素数值数组') from exc
    values = parsed if key in ranges else [parsed]
    if (not isinstance(values, list) or len(values) != (2 if key in ranges else 1)
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                   for v in values)):
        raise argparse.ArgumentTypeError('覆盖值必须为有限数值或二元素有限数值数组')
    name = (scalar | ranges)[key]
    if any(v < 0 if name == 'tau' else v <= 0 for v in values) or values[0] > values[-1]:
        raise argparse.ArgumentTypeError('速度和加速度必须为正，tau非负，范围必须有序')
    return text


def add_environment_overrides(parser):
    parser.add_argument('--env-override', action='append', default=[], type=environment_override,
                        metavar='env.KEY=VALUE', help='可重复；覆盖动作参数及reset范围，与训练配置一致')
