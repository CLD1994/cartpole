import importlib.util
from pathlib import Path
import unittest
import run_policy as r

class PausedWaitTests(unittest.TestCase):
    def harness(self):
        path=Path(r.__file__).parent/'integration_tests'/'test_angle_pause_runner.py'
        spec=importlib.util.spec_from_file_location('pause_wait_fixture',path)
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        return mod.RunnerCombinationTests()
    def test_short_stale_pause_waits_without_commands(self):
        h=self.harness()
        port,report,calls=h.run_loop(lambda t:80,.02,no_new_state=True,hard_limit=.015)
        self.assertEqual(report['angle_pause']['pause_count'],1)
        self.assertEqual(calls,[])
        self.assertFalse(any(data.startswith(b'VEL') for _,data in port.sent))
    def test_long_no_state_keeps_session_timeout(self):
        h=self.harness()
        with self.assertRaisesRegex(RuntimeError,'measurement_timeout'):
            h.run_loop(lambda t:80,.2,no_new_state=True,hard_limit=.015)
        port,report,calls,env=h.last_run
        self.assertEqual(calls,[])
        self.assertAlmostEqual(env['CLOCK'](),10.1,places=3)
    def test_fault_during_pause_still_exits(self):
        h=self.harness()
        with self.assertRaises(RuntimeError):
            h.run_loop(lambda t:80,.02,fault_at=.006,hard_limit=.015)
        self.assertEqual(h.last_run[2],[])
