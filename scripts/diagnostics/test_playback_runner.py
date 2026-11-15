"""策略加载边界测试；不导入 Isaac Lab、不访问模型文件。"""
import unittest
from types import SimpleNamespace
from playback_rtf import load_policy, parse_args


class DiagnosticArgumentsTests(unittest.TestCase):
    def parse(self, *extra):
        return parse_args(['--checkpoint', 'existing.pt', *extra],
                          add_launcher_args=lambda parser: None)

    def test_render_interval_preserves_task_default(self):
        self.assertIsNone(self.parse().render_interval)

    def test_render_interval_accepts_positive_integer(self):
        self.assertEqual(self.parse('--render-interval', '20').render_interval, 20)

    def test_render_interval_rejects_invalid_values(self):
        from contextlib import redirect_stderr
        from io import StringIO
        for value in ('0', '-1', '1.5', 'invalid'):
            with self.subTest(value=value), redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit) as error:
                    self.parse('--render-interval', value)
                self.assertEqual(error.exception.code, 2)


class PolicyLoadingTests(unittest.TestCase):
    def test_runner_uses_actual_environment_device(self):
        for device in ('cpu', 'cuda:1'):
            with self.subTest(device=device):
                env = SimpleNamespace(unwrapped=SimpleNamespace(device=device))
                cfg = SimpleNamespace(device='cuda:0')
                seen = {}
                policy = object()
                class Runner:
                    def load(self, path): seen['checkpoint'] = path
                    def get_inference_policy(self, *, device):
                        seen['policy_device'] = device
                        return policy
                def factory(actual_env, actual_cfg, *, log_dir):
                    seen.update(device_at_construction=actual_cfg.device, log_dir=log_dir)
                    return Runner()
                self.assertIs(load_policy(env, cfg, 'existing.pt', create_runner=factory), policy)
                self.assertEqual(seen, dict(device_at_construction=device, log_dir=None,
                                           checkpoint='existing.pt', policy_device=device))

    def test_checkpoint_failure_propagates(self):
        class Runner:
            def load(self, path): raise ValueError('checkpoint mismatch')
        env = SimpleNamespace(unwrapped=SimpleNamespace(device='cpu'))
        with self.assertRaises(ValueError):
            load_policy(env, SimpleNamespace(device='cuda:0'), 'bad.pt',
                        create_runner=lambda *args, **kwargs: Runner())

if __name__ == '__main__': unittest.main()
