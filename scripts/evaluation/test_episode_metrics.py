"""完整回合统计契约；不需要仿真器。"""
import unittest
from episode_metrics import EpisodeMetrics, run_cohort, summarize


def sample(states, *, terminated=None, truncated=None, reasons=None, actions=None):
    n = len(states)
    return dict(states=states, actions=actions or [0.0]*n,
                terminated=terminated or [False]*n, truncated=truncated or [False]*n,
                reasons=reasons or {})

class EpisodeTests(unittest.TestCase):
    def test_finished_slots_are_not_counted_again(self):
        metrics = EpisodeMetrics(num_envs=2, seed=17, dt=.002)
        metrics.record(**sample([[.1,.2,0,0],[.2,.3,0,0]], terminated=[True,False]))
        metrics.record(**sample([[9,9,0,0],[.3,.4,0,0]], truncated=[True,True]))
        result = metrics.report()
        self.assertEqual(len(result['episodes']), 2)
        self.assertEqual(result['episodes'][0]['steps'], 1)
        self.assertAlmostEqual(result['episodes'][0]['max_abs_x_m'], .1)
        self.assertEqual(result['episodes'][1]['steps'], 2)
        self.assertEqual(result['successes'], 1)

    def test_timeout_with_failure_is_failure_and_keeps_both_reasons(self):
        metrics = EpisodeMetrics(num_envs=1, seed=1, dt=.002)
        metrics.record(**sample([[.4,1.2,0,0]], terminated=[True], truncated=[True],
          reasons={'time_out':[True], 'cart_out_of_bounds':[True], 'pole_out_of_bounds':[True]}))
        episode = metrics.report()['episodes'][0]
        self.assertFalse(episode['success'])
        self.assertEqual(episode['terminal_state'], [.4,1.2,0,0])
        self.assertEqual(episode['termination_reasons'], ['time_out','cart_out_of_bounds','pole_out_of_bounds'])

    def test_incomplete_episodes_are_not_successes(self):
        metrics = EpisodeMetrics(num_envs=2, seed=1, dt=.002)
        metrics.record(**sample([[0,0,0,0],[0,0,0,0]], truncated=[True,False]))
        result = metrics.report()
        self.assertEqual(result['status'], 'incomplete')
        self.assertEqual(result['completed'],1)
        self.assertEqual(result['incomplete_env_ids'],[1])
        self.assertIsNone(result['success_rate'])

    def test_manual_reset_of_unfinished_slot_is_rejected(self):
        metrics = EpisodeMetrics(num_envs=2, seed=1, dt=.002)
        with self.assertRaises(RuntimeError):
            metrics.check_reset([0])

    def test_automatic_reset_of_completed_slot_is_allowed(self):
        metrics = EpisodeMetrics(num_envs=2, seed=1, dt=.002)
        metrics.record(**sample([[0,0,0,0],[0,0,0,0]], terminated=[True,False]))
        metrics.check_reset([0])
        with self.assertRaises(RuntimeError):
            metrics.check_reset(slice(None))

    def test_rms_includes_terminal_sample(self):
        metrics = EpisodeMetrics(num_envs=1, seed=1, dt=.002)
        metrics.record(**sample([[0,.3,0,0]]))
        metrics.record(**sample([[0,.4,0,0]], truncated=[True]))
        self.assertAlmostEqual(metrics.report()['episodes'][0]['theta_rms_rad'], (.125)**.5)

    def test_nonfinite_sample_rejected(self):
        metrics = EpisodeMetrics(num_envs=1, seed=1, dt=.002)
        with self.assertRaises(ValueError):
            metrics.record(**sample([[0,float('nan'),0,0]]))

    def test_cohort_stops_when_all_first_episodes_finish(self):
        metrics = EpisodeMetrics(num_envs=1, seed=1, dt=.002)
        calls=[]
        def step():
            calls.append(1)
            metrics.record(**sample([[0,0,0,0]], truncated=[True]))
        result = run_cohort(metrics, step=step, max_steps=3)
        self.assertEqual(len(calls),1)
        self.assertEqual(result['status'],'complete')

    def test_guard_limit_reports_incomplete(self):
        metrics = EpisodeMetrics(num_envs=1, seed=1, dt=.002)
        result = run_cohort(metrics, step=lambda: metrics.record(**sample([[0,0,0,0]])),max_steps=2)
        self.assertEqual(result['status'],'incomplete')
        self.assertEqual(result['completed'],0)

    def test_aggregate_weights_episodes_not_seed_means(self):
        result = summarize([dict(status='complete', episodes=[dict(success=True)]),
                            dict(status='complete', episodes=[dict(success=False)]*3)])
        self.assertEqual(result['success_rate'],.25)
        self.assertEqual(result['episodes'],4)
        self.assertLess(result['wilson_95_low'], .25)
        self.assertGreater(result['wilson_95_high'], .25)

if __name__ == '__main__': unittest.main()
