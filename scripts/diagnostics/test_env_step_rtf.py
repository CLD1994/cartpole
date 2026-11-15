import unittest
from types import SimpleNamespace as NS
from env_step_rtf import configure_baseline, measure_zero_actions, parse_args


def launcher(parser):
    parser.add_argument('--device', default='cuda:0')


class BaselineTests(unittest.TestCase):
    def test_default_timing_and_backend(self):
        args = parse_args([], add_launcher_args=launcher)
        self.assertEqual((args.task, args.physics_dt, args.decimation, args.physics),
                         ('Isaac-Cartpole', .001, 2, 'isaacsim_physx'))

    def test_invalid_values_rejected(self):
        for flag, value in [('--steps', '0'), ('--warmup', '-1'), ('--physics-dt', 'nan'),
                            ('--physics-dt', '0'), ('--decimation', '0'), ('--render-interval', '-1')]:
            with self.assertRaises(SystemExit):
                parse_args([flag, value], add_launcher_args=launcher)

    def test_config_preserves_task_terms(self):
        terms = object()
        cfg = NS(sim=NS(), scene=NS(), actions=terms)
        configure_baseline(cfg, parse_args([], add_launcher_args=launcher))
        self.assertIs(cfg.actions, terms)
        self.assertEqual((cfg.sim.dt, cfg.decimation, cfg.scene.num_envs), (.001, 2, 1))
        self.assertIsNone(cfg.log_dir)

    def test_zero_action_reused_and_reset_before_measurement(self):
        events, submissions = [], []
        action = object()
        env = NS(unwrapped=NS(num_envs=1, device='cuda:0', step_dt=.002,
                             action_manager=NS(total_action_dim=1)),
                 reset=lambda: events.append('reset'), step=lambda a: submissions.append(a))
        def zeros(shape, *, device):
            self.assertEqual((shape, device), ((1, 1), 'cuda:0'))
            return action
        sync = object()
        def measure(step, **kwargs):
            self.assertEqual(events, ['reset'])
            self.assertEqual(kwargs, dict(dt=.002, steps=10, warmup=2, synchronize=sync))
            step(); step()
            return {'rtf': 1.0}
        result = measure_zero_actions(env, make_zeros=zeros, measure=measure,
                                      synchronize=sync, steps=10, warmup=2)
        self.assertEqual(result, {'rtf': 1.0})
        self.assertTrue(all(a is action for a in submissions))
