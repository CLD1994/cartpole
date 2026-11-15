"""采样窗口契约测试；注入性能采样器，不导入仿真器或写文件。"""
import unittest
from itertools import count
from unittest.mock import Mock
import cProfile
from pathlib import Path

from playback_timing import measure_playback
from playback_rtf import parse_args


class ProfileTests(unittest.TestCase):
    def test_profile_is_opt_in(self):
        args = parse_args(['--checkpoint', 'model.pt'], add_launcher_args=lambda p: None)
        self.assertIsNone(args.profile)

    def test_profile_path(self):
        args = parse_args(['--checkpoint', 'model.pt', '--profile', 'run.prof', '--steps', '500'],
                          add_launcher_args=lambda p: None)
        self.assertEqual(args.profile, Path('run.prof'))
        self.assertEqual(args.steps, 500)

    def test_only_measured_steps_are_profiled(self):
        profiler = Mock(spec=cProfile.Profile)
        active = False
        observed = []
        def enable():
            nonlocal active
            active = True
        def disable():
            nonlocal active
            active = False
        profiler.enable.side_effect = enable
        profiler.disable.side_effect = disable
        ticks = count()
        result = measure_playback(lambda: observed.append(active), dt=.002, warmup=2, steps=3,
                                  profiler=profiler, clock=lambda: next(ticks))
        self.assertEqual(observed, [False, False, True, True, True])
        self.assertFalse(active)
        self.assertEqual(result['steps'], 3)
        profiler.enable.assert_called_once_with()
        profiler.disable.assert_called_once_with()

    def test_step_error_disables_profiler_and_propagates(self):
        profiler = Mock(spec=cProfile.Profile)
        def step():
            raise RuntimeError('simulation failed')
        with self.assertRaises(RuntimeError):
            measure_playback(step, dt=.002, warmup=0, steps=3, profiler=profiler)
        profiler.disable.assert_called_once_with()

    def test_warmup_error_does_not_start_profiler(self):
        profiler = Mock(spec=cProfile.Profile)
        def step():
            raise RuntimeError('warmup failed')
        with self.assertRaises(RuntimeError):
            measure_playback(step, dt=.002, warmup=1, steps=3, profiler=profiler)
        profiler.enable.assert_not_called()
