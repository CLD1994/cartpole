import unittest
from playback_timing import measure_playback

class Clock:
    def __init__(self, cost): self.now, self.cost, self.steps, self.syncs = 0., cost, 0, 0
    def clock(self): return self.now
    def sleep(self, seconds): self.now += seconds
    def step(self): self.now += self.cost; self.steps += 1
    def sync(self): self.syncs += 1

class TimingTests(unittest.TestCase):
    def run_case(self, cost, real_time=False):
        c = Clock(cost)
        r = measure_playback(c.step, dt=.002, warmup=5, steps=10, real_time=real_time,
                             clock=c.clock, sleep=c.sleep, synchronize=c.sync)
        self.assertEqual(c.steps, 15)
        self.assertEqual(c.syncs, 2)
        return r
    def test_warmup_excluded(self):
        r = self.run_case(.004)
        self.assertAlmostEqual(r['wall_s'], .04)
        self.assertAlmostEqual(r['rtf'], .5)
    def test_unpaced_does_not_sleep(self):
        self.assertAlmostEqual(self.run_case(.001)['rtf'], 2.)
    def test_real_time_waits_for_fast_steps(self):
        self.assertAlmostEqual(self.run_case(.001, True)['rtf'], 1.)
    def test_real_time_cannot_catch_up_slow_steps(self):
        self.assertAlmostEqual(self.run_case(.004, True)['rtf'], .5)
    def test_start_sync_excluded_and_end_sync_included(self):
        c = Clock(.004)
        delays = iter((10., .003))
        def synchronize(): c.now += next(delays)
        result = measure_playback(c.step, dt=.002, warmup=5, steps=10,
                                  clock=c.clock, sleep=c.sleep, synchronize=synchronize)
        self.assertAlmostEqual(result['wall_s'], .043)
        self.assertAlmostEqual(result['step_loop_wall_s'], .040)
        self.assertAlmostEqual(result['completion_sync_s'], .003)
        self.assertAlmostEqual(result['sim_s'], .020)

    def test_bad_duration_rejected(self):
        for dt in (0, -1, float('nan')):
            with self.assertRaises(ValueError): measure_playback(lambda: None, dt=dt)
    def test_step_exception_propagates(self):
        def fail(): raise RuntimeError('simulation stopped')
        with self.assertRaises(RuntimeError): measure_playback(fail, dt=.002)

if __name__ == '__main__': unittest.main()
