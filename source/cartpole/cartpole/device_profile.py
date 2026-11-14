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


def model_parameters(profile):
    measured, configured = profile['measured'], profile['configured']
    mass = measured['pole_mass_kg']
    length = measured['pole_total_length_m']
    cart_mass = measured['cart_moving_mass_kg']
    hz = profile['training']['control_hz']
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
    fraction = _finite_number(profile['training'].get('cart_mass_randomization_fraction', .1),
                              'cart_mass_randomization_fraction')
    if not 0 <= fraction < 1:
        raise ValueError('质量随机化比例必须在 [0, 1) 内')
    return {
        'pole_mass_kg': mass,
        'pole_length_m': length,
        'cart_mass_kg': cart_mass,
        'pole_com_m': length / 2,
        'pole_inertia_com': mass * length**2 / 12,
        'pole_inertia_pivot': mass * length**2 / 3,
        'soft_limit_m': travel / 2e6 - margin / 1e6,
        'policy_dt': 1 / hz,
        'cart_mass_randomization_fraction': fraction,
        'velocity_m_s': velocity / 1000,
        'acceleration_m_s2': acceleration / 1000,
    }
