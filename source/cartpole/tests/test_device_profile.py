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
