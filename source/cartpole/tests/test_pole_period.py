"""周期判定的纯单元测试，不加载仿真器或物理 API 替身。"""
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("check_pole_physics", Path(__file__).parents[3] / "scripts/check_pole_physics.py")
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class PeriodTests(unittest.TestCase):
    def test_known_period(self):
        times = [i * .001 for i in range(8000)]
        angles = [.05 * math.cos(2 * math.pi * t / 1.2) for t in times]
        result = probe.measure_period(times, angles, 1.2)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["median_period_s"], 1.2, places=5)

    def test_wrong_period_rejected(self):
        times = [i * .01 for i in range(1000)]
        angles = [.05 * math.sin(2 * math.pi * t / 2) for t in times]
        self.assertFalse(probe.measure_period(times, angles, 1)["passed"])

    def test_static_or_short_trace_is_not_success(self):
        with self.assertRaises(ValueError):
            probe.measure_period([0, .001, .002], [0, 0, 0], 1)

    def test_invalid_time_or_values(self):
        for times, angles in [([0, 0], [0, 1]), ([0, 1], [0, float("nan")]), ([0], [1, 2])]:
            with self.subTest(times=times), self.assertRaises(ValueError):
                probe.measure_period(times, angles, 1)


if __name__ == "__main__":
    unittest.main()
