import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('prepare_package', Path(__file__).parents[2] / 'prepare_package.py')
m = importlib.util.module_from_spec(spec)

class PackageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec.loader.exec_module(m)

    def test_render_model_facts(self):
        text = m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                                  'training_acceleration_m_s2': 10}, 'a' * 64)
        self.assertIn('control_hz=400\n', text)
        self.assertIn('action_scale_mm_s=900\nvelocity_limit_mm_s=900\n', text)
        self.assertIn('acceleration_mm_s2=10000\nangle_sign=-1\n', text)

    def test_reject_nonintegral_firmware_units(self):
        with self.assertRaises(ValueError):
            m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                               'training_acceleration_m_s2': 10.00001}, 'a' * 64)

    def test_reject_invalid_hash(self):
        with self.assertRaises(ValueError):
            m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                               'training_acceleration_m_s2': 10}, 'oops')

if __name__ == '__main__': unittest.main()
