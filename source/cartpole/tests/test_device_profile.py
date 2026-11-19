"""模型派生量的纯函数单元测试，不读取文件或启动模拟器。"""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('device_profile', Path(__file__).parents[1] / 'cartpole/device_profile.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

def sample_profile():
    return {
        'measured': {'pole_mass_kg': .2316, 'pole_total_length_m': .49,
                     'cart_moving_mass_kg': .065, 'calibrated_travel_um': 853325},
        'configured': {'soft_limit_margin_um': 10000, 'target_speed_mm_s': 1250,
                       'acceleration_mm_s2': 10000},
        'training': {
            'control_hz': 500,
            'cart_mass_randomization_fraction': .2,
            'pole_mass_randomization_fraction': .25,
            'pole_com_randomization_fraction': .1,
            'pole_inertia_randomization_fraction': .3,
            'cart_damping_range_n_s_per_m': [.2, .4],
            'pole_damping_range_n_m_s_per_rad': [.003, .007],
        },
    }


FRACTION_FIELDS = (
    'cart_mass_randomization_fraction', 'pole_mass_randomization_fraction',
    'pole_com_randomization_fraction', 'pole_inertia_randomization_fraction',
)
DAMPING_FIELDS = ('cart_damping_range_n_s_per_m', 'pole_damping_range_n_m_s_per_rad')


class DeviceProfileTests(unittest.TestCase):
    def test_uniform_end_pivot_rod(self):
        p = m.model_parameters(sample_profile())
        self.assertAlmostEqual(p['pole_com_m'], .245)
        self.assertAlmostEqual(p['pole_inertia_com'], .00463393)
        self.assertAlmostEqual(p['pole_inertia_pivot'], .01853572)
        self.assertAlmostEqual(p['soft_limit_m'], .4166625)
        self.assertEqual(p['policy_dt'], .002)
        self.assertEqual(p['velocity_m_s'], 1.25)
        self.assertEqual(p['acceleration_m_s2'], 10)

    def test_profile_widths_generate_reset_ranges(self):
        result = m.model_parameters(sample_profile())
        for key, expected in (
            ('cart_mass_range_kg', (.052, .078)),
            ('pole_mass_range_kg', (.1737, .2895)),
            ('pole_com_offset_range_m', (.2205, .2695)),
            ('pole_inertia_scale_range', (.7, 1.3)),
        ):
            with self.subTest(key=key):
                self.assertEqual(len(result[key]), 2)
                self.assertAlmostEqual(result[key][0], expected[0])
                self.assertAlmostEqual(result[key][1], expected[1])
        self.assertEqual(result['cart_damping_range_n_s_per_m'], (.2, .4))
        self.assertEqual(result['pole_damping_range_n_m_s_per_rad'], (.003, .007))

    def test_missing_mass_does_not_become_zero(self):
        with self.assertRaises((KeyError, ValueError, TypeError)):
            m.model_parameters({'measured': {}})

class InvalidProfileTests(unittest.TestCase):
    def test_positive_fields_reject_invalid_values(self):
        fields = [('measured', key) for key in ('pole_mass_kg', 'pole_total_length_m',
                  'cart_moving_mass_kg', 'calibrated_travel_um')]
        fields += [('configured', 'target_speed_mm_s'), ('configured', 'acceleration_mm_s2'),
                   ('training', 'control_hz')]
        for section, key in fields:
            for value in (0, -1, float('nan'), float('inf'), True, '500', None):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    p = sample_profile(); p[section][key] = value
                    m.model_parameters(p)

    def test_margin_bounds(self):
        for value in (-1, 426662.5, 900000, float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                p = sample_profile()
                p['configured']['soft_limit_margin_um'] = value
                m.model_parameters(p)

    def test_randomization_fields_are_required(self):
        for field in (*FRACTION_FIELDS, *DAMPING_FIELDS):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, field):
                p = sample_profile()
                del p['training'][field]
                m.model_parameters(p)

    def test_randomization_fractions_reject_invalid_fields(self):
        for field in FRACTION_FIELDS:
            for value in (-.1, 1., float('inf'), True, '0.1', None):
                with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, field):
                    p = sample_profile()
                    p['training'][field] = value
                    m.model_parameters(p)

    def test_damping_ranges_reject_invalid_bounds(self):
        for field in DAMPING_FIELDS:
            for value in (None, '01', [], [0], [0, 1, 2], [-1, 1], [1, 0], [0, float('inf')],
                          [float('nan'), 1], [False, 1], [0, '1']):
                with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, field):
                    p = sample_profile()
                    p['training'][field] = value
                    m.model_parameters(p)

    def test_zero_margin_and_randomization_are_valid(self):
        p = sample_profile()
        p['configured']['soft_limit_margin_um'] = 0
        for field in FRACTION_FIELDS:
            p['training'][field] = 0
        for field in DAMPING_FIELDS:
            p['training'][field] = [0, 0]
        result = m.model_parameters(p)
        self.assertEqual(result['soft_limit_m'], .4266625)
        self.assertEqual(result['cart_mass_range_kg'], (.065, .065))
        self.assertEqual(result['pole_mass_range_kg'], (.2316, .2316))
        self.assertEqual(result['pole_com_offset_range_m'], (.245, .245))
        self.assertEqual(result['pole_inertia_scale_range'], (1, 1))
        self.assertEqual(result['cart_damping_range_n_s_per_m'], (0, 0))
        self.assertEqual(result['pole_damping_range_n_m_s_per_rad'], (0, 0))
