"""实验设置的纯行为契约。"""
import unittest
from pathlib import Path
import experiment_setup as setup

class SetupTests(unittest.TestCase):
    def test_manifest_limits_preserve_training_acceleration(self):
        limits = setup.limits_from_manifest({'training_control_hz':400,'action_scale_m_s':1.25,'training_acceleration_m_s2':9})
        self.assertEqual(limits.control_hz,400)
        self.assertEqual(limits.config,{'VEL_MAX':1250,'ACC_MAX':9000,'WDT_MS':100,'X_MARGIN_UM':10000})
    def test_unrepresentable_acceleration_rejected(self):
        with self.assertRaisesRegex(ValueError,'firmware_unit_precision'):
            setup.limits_from_manifest({'training_control_hz':400,'action_scale_m_s':1.25,'training_acceleration_m_s2':9.0001})
    def test_summary_keeps_failure_and_stop_outcome(self):
        text=setup.result_summary({'status':'error','error':'RuntimeError: inference_deadline','disabled_confirmed':True,'run_totals':{'command_completions':12}},Path('log.json'))
        self.assertIn('inference_deadline',text)
        self.assertIn('12',text)
        self.assertIn('log.json',text)
        self.assertNotIn('"run_totals"',text)
    def test_report_paths_do_not_collide(self):
        a=setup.report_path(Path('model'));b=setup.report_path(Path('model'))
        self.assertNotEqual(a,b)
        self.assertEqual(a.parent,Path('model/runs'))


class CommandLineTests(unittest.TestCase):
    def test_model_directory_is_only_required_artifact(self):
        from run_policy import parse_args
        args=parse_args(['--model','exported','--port','COM11','--start-confirmed'])
        self.assertEqual(args.model,Path('exported'))
        self.assertEqual(args.output.parent,Path('exported/runs'))
        self.assertFalse(args.json)
    def test_machine_output_is_explicit(self):
        from run_policy import parse_args
        args=parse_args(['--model','exported','--port','COM11','--diagnostic-timing','--json','--output','result.json'])
        self.assertTrue(args.json)
        self.assertEqual(args.output,Path('result.json'))

class EntryOutputTests(unittest.TestCase):
    def test_preparation_error_is_short_and_does_not_claim_device_stop(self):
        import io
        from unittest.mock import patch
        import run_policy
        stream=io.StringIO()
        with patch.object(run_policy,'main',side_effect=ValueError('bad_model')),patch('sys.stderr',stream):
            code=run_policy.entrypoint(['--port','COM11','--model','model','--start-confirmed'])
        self.assertEqual(code,1)
        self.assertIn('bad_model',stream.getvalue())
        self.assertIn('未打开串口',stream.getvalue())
        self.assertNotIn('Traceback',stream.getvalue())
    def test_diagnostic_summary_does_not_claim_disable_failure(self):
        text=setup.result_summary({'status':'complete','diagnostic_timing':True},Path('report.json'))
        self.assertIn('只读',text)
        self.assertNotIn('未确认 DISABLED',text)

if __name__ == '__main__':
    unittest.main()
