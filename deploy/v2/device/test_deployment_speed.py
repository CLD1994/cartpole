"""部署限幅保留模型动作倍率，且不放宽设备配置门禁。"""
import unittest
from dataclasses import replace
from session import Limits, Session
from test_session import Clock, Port
from test_state_session import frame
from experiment_setup import limits_from_manifest
from run_policy import parse_args

class DeploymentSpeedTests(unittest.TestCase):
    manifest={'training_control_hz':400,'action_scale_m_s':1.25,'training_acceleration_m_s2':10}
    def limits(self, cap=900):
        return limits_from_manifest(self.manifest, velocity_limit_mm_s=cap)
    def ready(self, cap=900):
        port=Port(); s=Session(port, Clock(), self.limits(cap))
        s.receive('MODE READY NONE')
        for k,v in dict(CAL_STATE=2,CAL_SAVED=1,CAL_STORAGE=0,CAL_LENGTH_UM=920531,
                        **s.limits.config).items():s.receive(f'OK {k} {v}')
        s.receive(frame()); return s,port
    def test_limit_preserves_scale_and_requires_device_cap(self):
        limits=self.limits()
        self.assertEqual(limits.speed_mm_s,1250)
        self.assertEqual(limits.config['VEL_MAX'],900)
        self.assertEqual(limits.config['ACC_MAX'],10000)
        self.assertEqual(self.manifest['action_scale_m_s'],1.25)
    def test_action_scaled_then_saturated_not_rescaled(self):
        s,p=self.ready();s.start()
        for action,expected in ((.5,625),(-.5,-625),(1,900),(-1,-900),(0,0)):
            self.assertEqual(s.command(action),expected)
            self.assertEqual(p.sent[-1],f'VEL {expected}\n'.encode())
    def test_no_override_retains_old_semantics(self):
        limits=limits_from_manifest(self.manifest)
        self.assertEqual(limits.config['VEL_MAX'],1250)
    def test_invalid_or_increased_limit_rejected(self):
        for cap in (0,-1,1251,2001,True,900.5):
            with self.subTest(cap=cap),self.assertRaises(ValueError):self.limits(cap)
    def test_mismatched_device_limit_still_blocks(self):
        s,p=self.ready();s.receive('OK VEL_MAX 1250')
        with self.assertRaisesRegex(RuntimeError,'VEL_MAX'):s.start()
        self.assertEqual(p.sent,[])
    def test_expired_state_still_stops(self):
        s,p=self.ready();s.start();s.clock.time+=.2
        with self.assertRaises(RuntimeError):s.command(1)
        self.assertNotIn(b'VEL 900\n',p.sent)
    def test_acceleration_override_preserves_training_contract(self):
        m=dict(self.manifest, training_acceleration_m_s2=9)
        limits=limits_from_manifest(m, velocity_limit_mm_s=900, acceleration_limit_mm_s2=10000)
        self.assertEqual(limits.acceleration_mm_s2,9000)
        self.assertEqual(limits.config["ACC_MAX"],10000)
        for cap in (0,99,30001,True,10000.5):
            with self.subTest(cap=cap),self.assertRaises(ValueError):
                limits_from_manifest(m, acceleration_limit_mm_s2=cap)
    def test_acceleration_readback_requires_explicit_deployment_value(self):
        s,p=self.ready()
        s.limits=replace(s.limits, acceleration_mm_s2=9000, deployment_acceleration_mm_s2=10000)
        self.assertIsNone(s.preflight_reason())
        s.receive("OK ACC_MAX 9000")
        self.assertIn("ACC_MAX",s.preflight_reason())
        self.assertEqual(limits_from_manifest(dict(self.manifest, training_acceleration_m_s2=9)).config["ACC_MAX"],9000)
    def test_runtime_acceleration_change_still_stops(self):
        s,p=self.ready()
        s.limits=replace(s.limits, acceleration_mm_s2=9000, deployment_acceleration_mm_s2=10000)
        s.start();s.receive("OK ACC_MAX 9000")
        with self.assertRaises(RuntimeError):s.command(1)
        self.assertNotIn(b"VEL 900\n",p.sent)
    def test_acceleration_cli_override_is_explicit(self):
        a=parse_args(["--port","COM1","--model","old","--diagnostic-timing","--acceleration-limit-mm-s2","10000"])
        self.assertEqual(a.acceleration_limit_mm_s2,10000)
    def test_cli_override_is_explicit(self):
        a=parse_args(['--port','COM1','--model','old','--diagnostic-timing','--velocity-limit-mm-s','900'])
        self.assertEqual(a.velocity_limit_mm_s,900)

if __name__=='__main__':unittest.main()
