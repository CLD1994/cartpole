"""显式评估配置的参数边界；不启动仿真或读写文件。"""
import unittest
from evaluate_policy import parse_args

class EnvironmentOverrideTests(unittest.TestCase):
    def parse(self, *values):
        argv = ['--checkpoint', 'model.pt']
        for value in values:
            argv += ['--env-override', value]
        return parse_args(argv, add_launcher_args=lambda parser: None)

    def test_accepts_scalar_and_reset_range(self):
        values = ['env.actions.joint_effort.vel_max=0.9',
                  'env.events.randomize_tracking.params.vel_max_range=[0.9,0.9]']
        self.assertEqual(self.parse(*values).env_override, values)

    def test_default_is_current_task(self):
        self.assertEqual(self.parse().env_override, [])

    def test_rejects_unrelated_or_invalid_overrides(self):
        from contextlib import redirect_stderr
        from io import StringIO
        for value in ['agent.seed=1', 'env.scene.num_envs=1',
                      'env.actions.joint_effort.vel_max=nan',
                      'env.actions.joint_effort.vel_max=0',
                      'env.actions.joint_effort.tau=-1',
                      'env.events.randomize_tracking.params.acc_max_range=[10,9]',
                      'env.events.randomize_tracking.params.tau_range=[0]',
                      'env.actions.joint_effort.vel_max=${other}',
                      'env.actions.joint_effort.vel_max=true',
                      'env.actions.joint_effort.vel_max=' + '9' * 400]:
            with self.subTest(value=value), redirect_stderr(StringIO()), self.assertRaises(SystemExit):
                self.parse(value)

class SelectionOverrideTests(unittest.TestCase):
    def test_selection_and_holdout_forward_identical_overrides(self):
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import MagicMock, patch
        from select_checkpoint import start_evaluation
        values = ['env.actions.joint_effort.vel_max=0.9',
                  'env.events.randomize_tracking.params.vel_max_range=[0.9,0.9]']
        args = SimpleNamespace(task='task', num_envs=1, device='cpu', horizon_s=None,
                               initial_cart_velocity_range=None, initial_pole_velocity_range=None,
                               env_override=values)
        output = MagicMock(spec=Path)
        candidate = dict(path='/run/candidate_2544.pt', iteration=2544, sha256='abc')
        with patch('select_checkpoint.digest', return_value='abc'), \
             patch('select_checkpoint.subprocess.Popen') as launch:
            for stage, seeds in [('selection', [101]), ('holdout', [1001])]:
                job = start_evaluation(candidate, args, output, seeds, stage)
                cmd = launch.call_args.args[0]
                forwarded = [cmd[i+1] for i, token in enumerate(cmd) if token == '--env-override']
                self.assertEqual(forwarded, values)
                job['log'].close()

    def test_report_with_different_override_is_rejected(self):
        import json
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        from select_checkpoint import finish_evaluation
        report_path = MagicMock(spec=Path)
        report_path.read_text.return_value = json.dumps(dict(status='complete',
            summary=dict(status='complete'), checkpoint_sha256='abc', seeds=[101], num_envs=1,
            task='task', env_overrides=[]))
        target = MagicMock(spec=Path)
        target.glob.return_value = [report_path]
        job = dict(candidate=dict(sha256='abc'), target=target, process=SimpleNamespace(returncode=0),
                   log=MagicMock())
        args = SimpleNamespace(num_envs=1, task='task',
                               env_override=['env.actions.joint_effort.vel_max=0.9'])
        with self.assertRaises(ValueError):
            finish_evaluation(job, args, [101])
