"""只读入口契约：模型清单、报告和输出不依赖真实设备。"""
import contextlib
import io
import json
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import shadow
import diagnose_timing
from cli_output import print_readonly_summary
import cli_output


class DiagnosticCliTests(unittest.TestCase):
    def test_entrypoint_reports_missing_manifest_without_success_claim(self):
        output = io.StringIO()
        def missing():
            raise FileNotFoundError('model.json')
        with contextlib.redirect_stdout(output):
            code = cli_output.readonly_entrypoint(missing, ['--json'])
        report = json.loads(output.getvalue())
        self.assertEqual(code, 1)
        self.assertEqual(report['status'], 'error')
        self.assertFalse(report['verification_complete'])
        self.assertFalse(report['motion_sent'])
        self.assertIn('model.json', report['error'])

    def test_entrypoint_reports_invalid_manifest_concisely(self):
        output = io.StringIO()
        def invalid():
            raise ValueError('unsupported_model_schema')
        with contextlib.redirect_stdout(output):
            code = cli_output.readonly_entrypoint(invalid, [])
        self.assertEqual(code, 1)
        self.assertIn('unsupported_model_schema', output.getvalue())
        self.assertNotIn('Traceback', output.getvalue())

    def test_default_summary_preserves_failure_without_dumping_samples(self):
        output = io.StringIO()
        report = {'status': 'error', 'motion_sent': False, 'error': 'read_timeout',
                  'samples': [{'private_detail': 'do_not_print'}]}
        with contextlib.redirect_stdout(output):
            print_readonly_summary(report, 'report.json')
        text = output.getvalue()
        self.assertIn('read_timeout', text)
        self.assertIn('report.json', text)
        self.assertNotIn('do_not_print', text)
        self.assertLessEqual(len(text.splitlines()), 4)

    def run_failed_open(self, module):
        manifest = {'training_control_hz': 400, 'action_scale_m_s': 1.25,
                    'training_acceleration_m_s2': 9.0}
        stream = io.StringIO()
        report_file = Mock()
        report_file.__enter__ = Mock(return_value=report_file)
        report_file.__exit__ = Mock(return_value=False)
        serial = Mock()
        serial.Serial.side_effect = OSError('test_port_unavailable')
        with patch('model_manifest.load_manifest', return_value=manifest), \
             patch.object(module, 'Policy'), patch.object(Path, 'mkdir'), \
             patch.object(Path, 'open', return_value=report_file), \
             patch.dict('sys.modules', {'serial': serial}), \
             patch('sys.argv', ['tool', '--port', 'TEST', '--model', 'model-dir', '--json']), \
             contextlib.redirect_stdout(stream):
            code = module.main()
        return code, json.loads(stream.getvalue())

    def test_shadow_keeps_port_failure_and_model_frequency(self):
        code, report = self.run_failed_open(shadow)
        self.assertEqual(code, 1)
        self.assertIn('test_port_unavailable', report['error'])
        self.assertEqual(report['target_control_hz'], 400)
        self.assertFalse(report['motion_sent'])

    def test_timing_uses_model_period_not_fixed_500hz(self):
        code, report = self.run_failed_open(diagnose_timing)
        self.assertEqual(code, 1)
        self.assertEqual(report['control_period_s'], .0025)
        self.assertIn('test_port_unavailable', report['error'])
        self.assertFalse(report['motion_sent'])
