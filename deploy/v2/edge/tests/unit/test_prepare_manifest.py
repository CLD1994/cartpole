import importlib.util
from pathlib import Path
import unittest
import json

spec = importlib.util.spec_from_file_location('prepare_package', Path(__file__).parents[2] / 'prepare_package.py')
m = importlib.util.module_from_spec(spec)

class PackageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec.loader.exec_module(m)

    def test_render_model_facts(self):
        text = m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                                  'training_acceleration_m_s2': 10}, 'a' * 64)
        self.assertEqual(json.loads(text), {
            'schema_version': 2, 'model_cpu': 'policy.fp32', 'model_npu': None,
            'sha256_cpu': 'a'*64, 'sha256_npu': None, 'control_hz': 400,
            'action_scale_mm_s': 900, 'velocity_limit_mm_s': 900,
            'acceleration_mm_s2': 10000, 'angle_sign': -1, 'pause_deg': 20, 'resume_deg': 18})

    def test_render_dual_backend_json(self):
        data=json.loads(m.render_manifest({'training_control_hz':400,'action_scale_m_s':.9,
            'training_acceleration_m_s2':10}, 'a'*64, 'b'*64))
        self.assertEqual(data['model_npu'], 'policy.om')
        self.assertEqual(data['sha256_npu'], 'b'*64)

    def test_reject_nonintegral_firmware_units(self):
        with self.assertRaises(ValueError):
            m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                               'training_acceleration_m_s2': 10.00001}, 'a' * 64)

    def test_reject_invalid_hash(self):
        with self.assertRaises(ValueError):
            m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                               'training_acceleration_m_s2': 10}, 'oops')

if __name__ == '__main__': unittest.main()
