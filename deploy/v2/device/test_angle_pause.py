import math
import unittest
from angle_pause import AnglePause
from session import Observation

def obs(deg, at=1.):
    return Observation(0, round(math.radians(deg)*1e6), 0, 0, at)

class AnglePauseTests(unittest.TestCase):
    def test_hysteresis_and_resume_budget(self):
        p=AnglePause()
        self.assertEqual(p.check(obs(21),1.,.999,.006,.004),'paused')
        self.assertEqual(p.check(obs(19,2.),2.,.999,.006,.004),None)
        self.assertTrue(p.paused)
        self.assertEqual(p.check(obs(17,3.),3.,.999,.006,.004),'resumed')
        self.assertFalse(p.paused)
    def test_expired_active_budget_cannot_enter_pause(self):
        p=AnglePause()
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            p.check(obs(80),1.,.99,.006,.004)
        self.assertFalse(p.paused)
    def test_pause_waits_on_stale_and_requires_fresh_resume(self):
        p=AnglePause();p.check(obs(80),1.,1.,.006,.004)
        self.assertIsNone(p.check(obs(80),1.004114,1.,.006,.004))
        self.assertIsNone(p.check(obs(0),1.005,1.,.006,.004))
        self.assertTrue(p.paused)
        self.assertIsNone(p.check(None,1.006,1.,.006,.004))
        self.assertEqual(p.check(obs(0,1.007),1.007,1.,.006,.004),'resumed')
    def test_first_pause_cannot_use_stale_measurement(self):
        with self.assertRaisesRegex(RuntimeError,'angle_pause_observation_deadline'):
            AnglePause().check(obs(80),1.005,1.004,.015,.004)
    def test_invalid_thresholds(self):
        for pair in ((60,60),(60,61),(0,0),(181,58),(float('nan'),58)):
            with self.assertRaises(ValueError):AnglePause(*pair)
    def test_repeat_does_not_emit_transition(self):
        p=AnglePause();p.check(obs(-80),1.,1.,.006,.004)
        self.assertIsNone(p.check(obs(-80,2),2,1,.006,.004))

    def test_normal_stale_observation_keeps_existing_runner_wait_path(self):
        self.assertIsNone(AnglePause().check(obs(10),1.005,1.004,.006,.004))
    def test_cli_explicit_nine_ms_does_not_change_profile(self):
        from run_policy import parse_args, timing_limits
        args=parse_args(['--port','COM11','--model','unused','--start-confirmed','--command-gap-hard-ms','9'])
        self.assertEqual(args.command_gap_hard_ms,9.)
        self.assertEqual(timing_limits(args.timing_profile),(.006,.004,.002))

    def test_default_twenty_eighteen_exact_both_sign_boundaries(self):
        from types import SimpleNamespace
        from run_policy import parse_args
        args=parse_args(['--port','FAKE','--model','unused','--start-confirmed'])
        self.assertEqual((args.angle_pause_deg,args.angle_resume_deg),(20.,18.))
        for sign in (-1,1):
            p=AnglePause()
            def frame(deg):return SimpleNamespace(si=lambda:(0,sign*math.radians(deg),0,0),received_at=1.)
            self.assertIsNone(p.check(frame(19.999),1.,1.,.025,.004))
            self.assertEqual(p.check(frame(20),1.,1.,.025,.004),'paused')
            self.assertIsNone(p.check(frame(18.001),1.,1.,.025,.004))
            self.assertEqual(p.check(frame(18),1.,1.,.025,.004),'resumed')
