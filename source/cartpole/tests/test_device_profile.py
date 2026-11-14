"""模型派生量的纯函数单元测试，不读取文件或启动模拟器。"""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('device_profile', Path(__file__).parents[1] / 'cartpole/device_profile.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class DeviceProfileTests(unittest.TestCase):
    def test_uniform_end_pivot_rod(self):
        p = m.model_parameters({'measured': {'pole_mass_kg': .2316, 'pole_total_length_m': .49,
            'cart_moving_mass_kg': .065, 'calibrated_travel_um': 853325},
            'configured': {'soft_limit_margin_um': 10000, 'target_speed_mm_s': 1250,
            'acceleration_mm_s2': 10000}, 'training': {'control_hz': 500}})
        self.assertAlmostEqual(p['pole_com_m'], .245)
        self.assertAlmostEqual(p['pole_inertia_com'], .00463393)
        self.assertAlmostEqual(p['pole_inertia_pivot'], .01853572)
        self.assertAlmostEqual(p['soft_limit_m'], .4166625)
        self.assertEqual(p['policy_dt'], .002)
        self.assertEqual(p['velocity_m_s'], 1.25)
        self.assertEqual(p['acceleration_m_s2'], 10)

    def test_missing_mass_does_not_become_zero(self):
        with self.assertRaises((KeyError, ValueError, TypeError)):
            m.model_parameters({'measured': {}})

class InvalidProfileTests(unittest.TestCase):
    def profile(self):
        return {'measured': {'pole_mass_kg': .2316, 'pole_total_length_m': .49,
            'cart_moving_mass_kg': .065, 'calibrated_travel_um': 853325},
            'configured': {'soft_limit_margin_um': 10000, 'target_speed_mm_s': 1250,
            'acceleration_mm_s2': 10000}, 'training': {'control_hz': 500,
            'cart_mass_randomization_fraction': .1}}

    def test_positive_fields_reject_invalid_values(self):
        fields = [('measured', key) for key in ('pole_mass_kg', 'pole_total_length_m',
                  'cart_moving_mass_kg', 'calibrated_travel_um')]
        fields += [('configured', 'target_speed_mm_s'), ('configured', 'acceleration_mm_s2'),
                   ('training', 'control_hz')]
        for section, key in fields:
            for value in (0, -1, float('nan'), float('inf'), True, '500', None):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    p = self.profile(); p[section][key] = value
                    m.model_parameters(p)

    def test_margin_and_mass_fraction_bounds(self):
        for section, key, values in (
            ('configured', 'soft_limit_margin_um', (-1, 426662.5, 900000, float('nan'))),
            ('training', 'cart_mass_randomization_fraction', (-.1, 1., float('inf')))):
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    p = self.profile(); p[section][key] = value
                    m.model_parameters(p)

    def test_zero_margin_and_randomization_are_valid(self):
        p = self.profile(); p['configured']['soft_limit_margin_um'] = 0
        p['training']['cart_mass_randomization_fraction'] = 0
        result = m.model_parameters(p)
        self.assertEqual(result['soft_limit_m'], .4266625)
        self.assertEqual(result['cart_mass_randomization_fraction'], 0)
