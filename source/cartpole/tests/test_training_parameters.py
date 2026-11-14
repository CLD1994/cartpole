"""训练区间契约的纯函数测试，不导入仿真器。"""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('training_parameters', Path(__file__).parents[1] / 'cartpole/training_parameters.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class TrainingRangesTests(unittest.TestCase):
    def check(self, **overrides):
        args = dict(soft_limit_m=.4166625, boundary_inset_m=.13,
                    initial_position_range=(-.15, .15), acceleration_range=(8., 10.))
        args.update(overrides)
        return m.validate_training_ranges(**args)

    def test_valid_ranges(self):
        self.assertAlmostEqual(self.check(), .2866625)

    def test_rejects_invalid_training_ranges(self):
        cases = [dict(soft_limit_m=float('nan')), dict(boundary_inset_m=-.1),
                 dict(boundary_inset_m=.4166625), dict(initial_position_range=(-.3, .15)),
                 dict(initial_position_range=(.2, -.2)), dict(initial_position_range=(float('nan'), .1)),
                 dict(acceleration_range=(10., 8.)), dict(acceleration_range=(0., 10.)),
                 dict(acceleration_range=(8., float('inf')))]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                self.check(**case)

    def test_initial_range_may_touch_termination_boundary(self):
        self.assertEqual(self.check(soft_limit_m=.5, boundary_inset_m=.25,
                                   initial_position_range=(-.25, .25)), .25)
