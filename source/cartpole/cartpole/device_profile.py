"""当前设备档案和纯模型参数推导。配置目标不等于实测运动能力。"""
import json
import math
from pathlib import Path


def load_profile():
    return json.loads((Path(__file__).parent / 'data/device-profile.json').read_text(encoding='utf-8'))


def model_parameters(profile):
    measured, configured = profile['measured'], profile['configured']
    mass = measured['pole_mass_kg']
    length = measured['pole_total_length_m']
    cart_mass = measured['cart_moving_mass_kg']
    hz = profile['training']['control_hz']
    values = (mass, length, cart_mass, hz, measured['calibrated_travel_um'])
    if any(not math.isfinite(v) or v <= 0 for v in values):
        raise ValueError('设备质量、长度和频率必须为有限正数')
    return {
        'pole_mass_kg': mass,
        'pole_length_m': length,
        'cart_mass_kg': cart_mass,
        'pole_com_m': length / 2,
        'pole_inertia_com': mass * length**2 / 12,
        'pole_inertia_pivot': mass * length**2 / 3,
        'soft_limit_m': measured['calibrated_travel_um'] / 2e6 - configured['soft_limit_margin_um'] / 1e6,
        'policy_dt': 1 / hz,
        'cart_mass_randomization_fraction': profile['training'].get('cart_mass_randomization_fraction', 0.1),
        'velocity_m_s': configured['target_speed_mm_s'] / 1000,
        'acceleration_m_s2': configured['acceleration_mm_s2'] / 1000,
    }
