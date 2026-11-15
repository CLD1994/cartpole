"""单动作运行器边界；不打开串口，不加载模型。"""
import json
import unittest
from unittest.mock import Mock, patch
from run_policy import (ControlTransport, DiagnosticTransport, BoundSession,
                        timing_limits, valid_motion_line, next_grid_slot,
                        candidate_refresh_allowed, load_contract)
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
    def test_contract_dual_rejected(self):
        path=Mock(); path.read_text.return_value=json.dumps({'action_type':'cart_target_velocity_acceleration'})
        with self.assertRaisesRegex(ValueError,'single_action_contract_mismatch'):load_contract(path,'unused')
    def test_single_contract_400_and_500(self):
        c={'schema_version':1,'observation_order':['x','theta','xdot','thetadot'],
           'observation_units':['m','rad','m/s','rad/s'],'action_scale_m_s':1.25,
           'x_sign':1,'action_sign':1,'action_type':'cart_target_velocity','angle_sign':-1,
           'firmware_coordinates':{'POLE_ZERO_URAD':0,'POLE_INV':1,'X_INV':0},
           'export_contract_reviewed':True,'coordinate_evidence_reviewed':True,
           'timing_mismatch_acknowledged':True,'onsite_stop_ready':True,'evidence_note':'test'}
        for hz in (400,500):
            c['training_control_hz']=hz; path=Mock();path.read_text.return_value=json.dumps(c)
            with patch('run_policy.verify_model_files') as verify:
                self.assertEqual(load_contract(path,'model')['training_control_hz'],hz)
                verify.assert_called_once()
        for key,value in [('training_control_hz',400.0),('action_dim',True),('schema_version',True)]:
            bad={**c,key:value};path=Mock();path.read_text.return_value=json.dumps(bad)
            with self.assertRaisesRegex(ValueError,'single_action_contract_mismatch'):load_contract(path,'unused')

if __name__=='__main__':unittest.main()
