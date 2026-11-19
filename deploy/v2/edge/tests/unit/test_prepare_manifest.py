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

    def test_firmware_unit_errors_identify_code_and_field(self):
        for field in ('action_scale_m_s', 'training_acceleration_m_s2'):
            for value, code in ((.00001, m.ERROR_FIRMWARE_UNIT_PRECISION),
                                (3000000, m.ERROR_FIRMWARE_UNIT_RANGE)):
                with self.subTest(field=field, value=value):
                    manifest = dict(training_control_hz=400, action_scale_m_s=.9,
                                    training_acceleration_m_s2=10)
                    manifest[field] = value
                    with self.assertRaises(ValueError) as caught:
                        m.render_manifest(manifest, 'a'*64)
                    error = m.parse_firmware_unit_error(str(caught.exception))
                    self.assertEqual(error.code, code)
                    self.assertEqual(error.field, field)

    def test_unit_error_parser_rejects_other_errors_and_malformed_fields(self):
        for message in (None, '', m.ERROR_INVALID_SHA256,
                        m.ERROR_FIRMWARE_UNIT_PRECISION + ':unknown',
                        m.ERROR_FIRMWARE_UNIT_RANGE + ':action_scale_m_s:extra',
                        m.ERROR_FIRMWARE_UNIT_RANGE + ':action_scale_m_s\n'):
            with self.subTest(message=message):
                with self.assertRaises(ValueError) as caught:
                    m.parse_firmware_unit_error(message)
                self.assertEqual(str(caught.exception), m.ERROR_INVALID_FIRMWARE_UNIT_ERROR)

    def test_reject_invalid_hash(self):
        with self.assertRaises(ValueError) as caught:
            m.render_manifest({'training_control_hz': 400, 'action_scale_m_s': .9,
                               'training_acceleration_m_s2': 10}, 'oops')
        self.assertEqual(str(caught.exception), m.ERROR_INVALID_SHA256)

if __name__ == '__main__': unittest.main()
