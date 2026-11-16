"""单动作运行器边界；不打开串口，不加载模型。"""
import json
import unittest
from unittest.mock import Mock, patch
from run_policy import (ControlTransport, DiagnosticTransport, BoundSession,
                        timing_limits, valid_motion_line, next_grid_slot,
                        candidate_refresh_allowed)
from shadow import Policy

class RunnerTests(unittest.TestCase):
    def test_normal_expiry_is_separate_from_failure(self):
        from run_policy import check_trial_end, TrialEnd
        check_trial_end(4.99,5)
        check_trial_end(100,float('inf'))
        with self.assertRaises(TrialEnd):check_trial_end(5,5)
    def test_motion_whitelist(self):
        for line in ('VEL 0\n', 'VEL -1250\n', 'VEL 1250\n'):
            self.assertTrue(valid_motion_line(line))
        for line in ('VELA 1 100\n', 'VEL 1251\n', 'ENABLE\n', 'VEL +1\n', 'VEL 1\nVEL 2\n'):
            self.assertFalse(valid_motion_line(line))
        port = Mock(); port.write.side_effect = len
        transport = ControlTransport(port)
        for data in (b'ENABLE\n', b'VELA?\n', b'CONFIG ACC_MAX 9000\n'):
            with self.assertRaises(ValueError): transport.write(data)
        self.assertEqual(transport.write(b'CONFIG? ACC_MAX\n'),16)
    def test_readonly_transport(self):
        port=Mock(); port.write.side_effect=len
        t=DiagnosticTransport(port)
        self.assertEqual(t.write(b'VEL 10\n'),7)
        self.assertEqual(t.write(b'DISABLE\n'),8)
        port.write.assert_not_called()
        t.write(b'QUERY\n'); port.write.assert_called_once_with(b'QUERY\n')
    def test_timing_profiles_are_explicit(self):
        self.assertEqual(timing_limits('normal'),(.006,.004,.002))
        self.assertEqual(timing_limits('debug'),(.015,.012,None))
        with self.assertRaises(ValueError):timing_limits('monitor')
    def test_skip_missed_slots_without_burst(self):
        self.assertEqual(next_grid_slot(0,0,9_000_000,2_500_000),4)
        self.assertEqual(next_grid_slot(0,0,2_000_000,2_000_000),2)
    def test_refresh_cannot_extend_command_deadline(self):
        self.assertTrue(candidate_refresh_allowed(.005,0,0,1))
        self.assertFalse(candidate_refresh_allowed(.006,0,0,1))
    def test_no_host_position_gate(self):
        self.assertFalse(BoundSession.enforce_position_limits)
    def test_dual_policy_refused_before_runtime_import(self):
        with self.assertRaisesRegex(ValueError,'unsupported_action_dimension'): Policy('unused',2)


class ExpiryCallsiteTests(unittest.TestCase):
    """执行运行器中的实际期限分支；外部传输/时钟为替身。"""
    def source_tree(self):
        import ast
        from pathlib import Path
        return ast.parse(Path(__file__).with_name('run_policy.py').read_text())
    def run_nodes(self, nodes, now):
        import ast
        from run_policy import check_trial_end
        def gate(reason):raise RuntimeError(reason)
        refresh=Mock()
        context=dict(CLOCK=lambda:now, end=5., last_command_at=4.999,
                     hard_limit_s=.006, gate=gate, check_trial_end=check_trial_end,
                     refresh=refresh, remaining=-.001)
        exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])), '<runner-branch>', 'exec'),context)
        return refresh
    def test_wait_fresh_expiry_and_real_gap(self):
        import ast
        from run_policy import TrialEnd
        node=next(n for n in ast.walk(self.source_tree()) if isinstance(n,ast.If) and ast.unparse(n.test)=='remaining <= 0')
        with self.assertRaises(TrialEnd):self.run_nodes([node],5.)
        with self.assertRaisesRegex(RuntimeError,'wait_fresh_command_deadline'):self.run_nodes([node],5.006)
    def test_readback_expiry_and_real_gap(self):
        import ast
        from run_policy import TrialEnd
        node=next(n for n in ast.walk(self.source_tree()) if isinstance(n,ast.If) and ast.unparse(n.test)=='refresh' and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='query_at' for t in x.targets) for x in n.body))
        with self.assertRaises(TrialEnd):self.run_nodes(node.body,5.)
        with self.assertRaisesRegex(RuntimeError,'readback_send_deadline'):self.run_nodes(node.body,5.006)
        self.run_nodes(node.body,4.999).send_if_due.assert_called_once_with(4.999,5.)


class CalibrationBindingTests(unittest.TestCase):
    def ready_session(self):
        from session import Limits
        s=BoundSession(Mock(), lambda: 10., Limits(400,1250,9000,10000), {'POLE_INV':1,'X_INV':0})
        s.transport.write.side_effect=len
        s.mode=('READY','NONE');s._mode_at=10.
        s.values={k:(v,10.) for k,v in s.expected_readback.items()}
        s.values['CAL_LENGTH_UM']=(854280,10.)
        return s
    def test_zero_required_before_binding(self):
        s=self.ready_session()
        self.assertEqual(s._metadata_reason(10.),'readback_POLE_ZERO_URAD')
    def test_invalid_zero_refused(self):
        s=self.ready_session();s.values['POLE_ZERO_URAD']=(4000000,10.)
        self.assertEqual(s._metadata_reason(10.),'readback_POLE_ZERO_URAD')
    def test_binding_freezes_live_calibration(self):
        s=self.ready_session();s.values['POLE_ZERO_URAD']=(2179945,10.)
        snapshot=s.bind_calibration()
        self.assertEqual(s.expected_readback['POLE_ZERO_URAD'],2179945)
        self.assertEqual(s.bound_travel_um,854280)
        s.values['POLE_ZERO_URAD']=(100,10.)
        self.assertEqual(snapshot['POLE_ZERO_URAD'],2179945)
        self.assertEqual(s._metadata_reason(10.),'readback_POLE_ZERO_URAD')

class BoundActiveTests(CalibrationBindingTests):
    def test_bound_zero_change_stops_active_session(self):
        from session import Phase
        s=self.ready_session();s.values['POLE_ZERO_URAD']=(2179945,10.)
        from session import Observation
        s.bind_calibration();s._valid=True;s._valid_at=10.
        s.observation=Observation(0,0,0,0,10.);s.phase=Phase.ACTIVE
        s.receive('OK POLE_ZERO_URAD 2179946')
        self.assertEqual(s.failure,'readback_changed')
        self.assertEqual(s.phase,Phase.STOPPING)
    def test_stale_zero_cannot_be_bound(self):
        s=self.ready_session();s.values['POLE_ZERO_URAD']=(2179945,8.)
        with self.assertRaisesRegex(RuntimeError,'preflight:readback_POLE_ZERO_URAD'):
            s.bind_calibration()
    def test_device_acceleration_does_not_replace_training_requirement(self):
        s=self.ready_session();s.values['POLE_ZERO_URAD']=(2179945,10.)
        s.values['ACC_MAX']=(10000,10.)
        with self.assertRaisesRegex(RuntimeError,'preflight:readback_ACC_MAX'):
            s.bind_calibration()

if __name__ == '__main__':
    unittest.main()
