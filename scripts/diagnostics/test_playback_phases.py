import unittest
from playback_timing import HostPhaseTimer, measure_playback

class PhaseTests(unittest.TestCase):
    def test_returns_result_and_measures_host_elapsed(self):
        times = iter([1., 1.004, 2., 2.010])
        timer = HostPhaseTimer(clock=lambda: next(times))
        self.assertEqual(timer.call('policy', lambda value: value + 1, 3), 4)
        timer.call('policy', lambda: None)
        data = timer.summary()['policy']
        self.assertEqual(data['calls'], 2)
        self.assertAlmostEqual(data['total_s'], .014)
        self.assertAlmostEqual(data['p50_ms'], 4.)
        self.assertAlmostEqual(data['p95_ms'], 10.)
    def test_clear_excludes_warmup(self):
        times = iter([0., 5., 6., 6.001])
        timer = HostPhaseTimer(clock=lambda: next(times))
        timer.call('environment', lambda: None)
        timer.clear()
        timer.call('policy', lambda: None)
        self.assertNotIn('environment', timer.summary())
        self.assertEqual(timer.summary()['policy']['calls'], 1)
    def test_callback_excludes_warmup_and_its_own_time(self):
        now = [0.]
        samples = []
        def step(): now[0] += .004; samples.append(now[0])
        def start(): samples.clear(); now[0] += 10.
        result = measure_playback(step, dt=.002, warmup=3, steps=2,
                                  clock=lambda: now[0], on_measurement_start=start)
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(result['wall_s'], .008)
    def test_error_propagates(self):
        timer = HostPhaseTimer()
        def fail(): raise RuntimeError('failure')
        with self.assertRaises(RuntimeError): timer.call('policy', fail)
