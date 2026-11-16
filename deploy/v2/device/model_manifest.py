"""从训练配置生成模型事实；不包含设备标定、现场批准或隐式默认值。"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

from model_integrity import verify_model_files

OBSERVATION_ORDER = ['x', 'theta', 'xdot', 'thetadot']
OBSERVATION_UNITS = ['m', 'rad', 'm/s', 'rad/s']
_ACTION = 'cartpole.tasks.manager_based.cartpole.mdp.vel_tracking_action:JointVelocityTrackingAction'
_POSITION = 'isaaclab.envs.mdp.observations:joint_pos_rel'
_VELOCITIES = {'isaaclab.envs.mdp.observations:joint_vel_rel',
               'cartpole.tasks.manager_based.cartpole.mdp.velocity_window_observation:JointVelocityWindow'}


def _positive(value):
    return type(value) in (float, int) and math.isfinite(value) and value > 0


def training_contract(config):
    """验证保存的单动作训练配置，拒绝未知动作或观察语义。"""
    try:
        dt, decimation = config['sim']['dt'], config['decimation']
        actions = {k: v for k, v in config['actions'].items() if v is not None}
        if set(actions) != {'joint_effort'}:
            raise ValueError('single_action_required')
        action = actions['joint_effort']
        if action['class_type'] not in (_ACTION, '{DIR}.vel_tracking_action:JointVelocityTrackingAction') or action['joint_names'] != ['slider_to_cart']:
            raise ValueError('single_action_semantics_unknown')
        if (action.get('scale', 1) != 1 or action.get('offset', 0) != 0
                or action.get('clip') is not None or action.get('asset_name', 'robot') != 'robot'
                or action.get('use_default_offset', False) is not False):
            raise ValueError('action_transform_unsupported')
        speed, acceleration = action['vel_max'], action['acc_max']
        if not _positive(dt) or type(decimation) is not int or decimation <= 0:
            raise ValueError('training_configuration_invalid_period')
        frequency = 1.0 / (dt * decimation)
        if not any(math.isclose(frequency, hz, rel_tol=0, abs_tol=1e-8) for hz in (400, 500)):
            raise ValueError('training_configuration_unsupported_frequency')
        if speed != 1.25 or not _positive(acceleration):
            raise ValueError('training_configuration_invalid_actuator')
        policy = config['observations']['policy']
        terms = [(k, v) for k, v in policy.items() if isinstance(v, dict) and 'func' in v]
        if ([k for k, _ in terms] != ['joint_pos_rel', 'joint_vel_rel']
                or terms[0][1]['func'] != _POSITION or terms[1][1]['func'] not in _VELOCITIES
                or policy.get('concatenate_terms') is not True
                or policy.get('history_length', 0) not in (None, 0)
                or policy.get('concatenate_dim', -1) != -1
                or policy.get('flatten_history_dim', True) is not True):
            raise ValueError('observation_semantics_unknown')
        # Custom scaling/history would invalidate the four SI values used by the runner.
        for name, term in terms:
            if (term.get('scale') not in (None, 1, 1.0)
                    or term.get('history_length', 0) not in (0, None)
                    or term.get('clip') is not None or term.get('modifiers') not in (None, [])):
                raise ValueError('observation_transform_unsupported')
            params = term.get('params', {})
            if term['func'].endswith(':JointVelocityWindow'):
                if not isinstance(params, dict) or set(params) != {'window_s'} or not _positive(params['window_s']):
                    raise ValueError('observation_window_parameters_unknown')
            elif params != {}:
                raise ValueError('observation_joint_selection_unknown')
        randomization = config['events']['randomize_tracking']['params']
        if list(randomization['vel_max_range']) != [speed, speed]:
            raise ValueError('training_randomization_changes_action_scale')
        acc_range = list(randomization['acc_max_range'])
        if (len(acc_range) != 2 or not all(_positive(v) for v in acc_range)
                or not acc_range[0] <= acc_range[1] <= acceleration):
            raise ValueError('training_randomization_exceeds_acceleration')
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError('training_configuration_missing_or_invalid:' + str(exc)) from exc
    return {'observation_order': OBSERVATION_ORDER.copy(), 'observation_units': OBSERVATION_UNITS.copy(),
            'action_type': 'cart_target_velocity', 'action_dim': 1,
            'action_scale_m_s': speed, 'training_control_hz': round(frequency),
            'training_acceleration_m_s2': acceleration}


def validate_manifest(value):
    """验证清单的结构与已支持语义；文件指纹由 load_manifest 核验。"""
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1:
        raise ValueError('model_manifest_schema_unsupported')
    if (value.get('observation_order') != OBSERVATION_ORDER
            or value.get('observation_units') != OBSERVATION_UNITS
            or value.get('action_type') != 'cart_target_velocity'
            or type(value.get('action_dim')) is not int or value['action_dim'] != 1
            or type(value.get('training_control_hz')) is not int or value['training_control_hz'] not in (400, 500)
            or value.get('action_scale_m_s') != 1.25
            or not _positive(value.get('training_acceleration_m_s2'))):
        raise ValueError('model_manifest_semantics_invalid')
    source = value.get('training_source')
    if (not isinstance(source, dict)
            or source.get('association') != 'operator_selected_checkpoint_and_onnx'
            or not isinstance(source.get('checkpoint_name'), str) or not source['checkpoint_name']
            or any(not isinstance(source.get(k), str) or re.fullmatch('[0-9a-f]{64}', source[k]) is None
                   for k in ('env_yaml_sha256', 'checkpoint_sha256'))):
        raise ValueError('model_manifest_source_required')
    return value


def load_manifest(path, model):
    value = validate_manifest(json.loads(Path(path).read_text(encoding='utf-8-sig')))
    verify_model_files(model, value)
    return value
