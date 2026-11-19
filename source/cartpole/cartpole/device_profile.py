"""当前设备档案和纯模型参数推导。配置目标不等于实测运动能力。"""
import json
import math
from pathlib import Path


def load_profile():
    return json.loads((Path(__file__).parent / 'data/device-profile.json').read_text(encoding='utf-8'))


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} 必须为有限数值')
    return value


def _training_fraction(training, name):
    value = _finite_number(training.get(name), name)
    if not 0 <= value < 1:
        raise ValueError(f'{name} 必须在 [0, 1) 内')
    return value


def _damping_range(training, name):
    bounds = training.get(name)
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
        raise ValueError(f'{name} 必须包含两个端点')
    lower = _finite_number(bounds[0], f'{name}[0]')
    upper = _finite_number(bounds[1], f'{name}[1]')
    if not 0 <= lower <= upper:
        raise ValueError(f'{name} 必须满足 0 ≤ 下限 ≤ 上限')
    return lower, upper


def _randomized_range(nominal, training, name):
    fraction = _training_fraction(training, name)
    return nominal * (1 - fraction), nominal * (1 + fraction)


def model_parameters(profile):
    measured, configured, training = profile['measured'], profile['configured'], profile['training']
    mass = measured['pole_mass_kg']
    length = measured['pole_total_length_m']
    cart_mass = measured['cart_moving_mass_kg']
    hz = training['control_hz']
    travel = measured['calibrated_travel_um']
    velocity = configured['target_speed_mm_s']
    acceleration = configured['acceleration_mm_s2']
    for name, value in (('pole_mass_kg', mass), ('pole_total_length_m', length),
                        ('cart_moving_mass_kg', cart_mass), ('control_hz', hz),
                        ('calibrated_travel_um', travel), ('target_speed_mm_s', velocity),
                        ('acceleration_mm_s2', acceleration)):
        if _finite_number(value, name) <= 0:
            raise ValueError(f'{name} 必须为正数')
    margin = _finite_number(configured['soft_limit_margin_um'], 'soft_limit_margin_um')
    if not 0 <= margin < travel / 2:
        raise ValueError('软限位余量必须非负且小于半行程')
    pole_com_m = length / 2
    return {
        'pole_mass_kg': mass,
        'pole_length_m': length,
        'cart_mass_kg': cart_mass,
        'pole_com_m': pole_com_m,
        'pole_inertia_com': mass * length**2 / 12,
        'pole_inertia_pivot': mass * length**2 / 3,
        'soft_limit_m': travel / 2e6 - margin / 1e6,
        'policy_dt': 1 / hz,
        'cart_mass_range_kg': _randomized_range(cart_mass, training, 'cart_mass_randomization_fraction'),
        'pole_mass_range_kg': _randomized_range(mass, training, 'pole_mass_randomization_fraction'),
        'pole_com_offset_range_m': _randomized_range(pole_com_m, training, 'pole_com_randomization_fraction'),
        'pole_inertia_scale_range': _randomized_range(1, training, 'pole_inertia_randomization_fraction'),
        'cart_damping_range_n_s_per_m': _damping_range(training, 'cart_damping_range_n_s_per_m'),
        'pole_damping_range_n_m_s_per_rad': _damping_range(training, 'pole_damping_range_n_m_s_per_rad'),
        'velocity_m_s': velocity / 1000,
        'acceleration_m_s2': acceleration / 1000,
    }
