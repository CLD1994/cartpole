"""候选选择契约；文件使用临时目录，进程与张量加载由替身隔离。"""
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from select_checkpoint import digest, finish_evaluation, nominate, start_evaluation


class SelectionContracts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name).resolve()

    def test_default_nomination_uses_only_saved_dynamic_candidates(self):
        directory = self.run / 'training-candidates'
        directory.mkdir()
        paths = [directory / f'candidate_{i}.pt' for i in (1701, 1703, 1706)]
        for path in [*paths, self.run / 'model_1200.pt', self.run / 'model_1999.pt']:
            path.write_bytes(b'checkpoint placeholder')
        (directory / 'candidates.json').write_text(json.dumps({
            'schema_version': 1, 'status': 'complete',
            'candidates': [{'checkpoint': str(path)} for path in paths],
        }))
        loaded = []
        def load(path, *, map_location, weights_only):
            loaded.append(Path(path))
            self.assertEqual(map_location, 'cpu')
            self.assertTrue(weights_only)
            return {'iter': int(Path(path).stem.split('_')[1])}
        with patch.dict('sys.modules', {'torch': SimpleNamespace(load=load)}):
            selected, metadata = nominate(self.run, 3, 20, None)
        self.assertEqual([Path(row['path']) for row in selected], paths)
        self.assertEqual(set(loaded), set(paths))
        self.assertEqual(metadata['method'], 'saved_training_candidates')

    def test_missing_saved_candidates_does_not_silently_select_periodic_model(self):
        (self.run / 'model_1200.pt').write_bytes(b'checkpoint placeholder')
        with patch.dict('sys.modules', {'torch': SimpleNamespace()}):
            with self.assertRaises(ValueError):
                nominate(self.run, 3, 20, None)

    def test_holdout_reuses_only_checkpoint_identity_and_its_own_result(self):
        path = self.run / 'candidate_1706.pt'
        path.write_bytes(b'checkpoint placeholder')
        selected = dict(path=str(path), iteration=1706, sha256=digest(path),
                        task='task', report='selection-report', summary={'successes': 0})
        args = SimpleNamespace(task='task', num_envs=1, device='cpu', horizon_s=None,
                               initial_cart_velocity_range=None, initial_pole_velocity_range=None)
        process = SimpleNamespace(returncode=0)
        with patch('select_checkpoint.subprocess.Popen', return_value=process):
            job = start_evaluation(selected, args, self.run, [1001], 'holdout')
        self.addCleanup(job['log'].close)
        target = job['target'] / 'evaluation-test'
        target.mkdir()
        (target / 'env.yaml').write_text('fixed config')
        (target / 'report.json').write_text(json.dumps(dict(
            status='complete', summary=dict(status='complete', successes=1),
            checkpoint_sha256=selected['sha256'], seeds=[1001], num_envs=1, task='task',
            cohorts=[dict(episodes=[dict(success=True, cart_mean_abs_velocity_m_s=.1,
                                        cart_net_displacement_m=.2)])], code={'commit': 'fixed'},
        )))
        result = finish_evaluation(job, args, [1001])
        self.assertEqual(result['summary']['successes'], 1)
        self.assertEqual(result['report'], str(target / 'report.json'))
        self.assertEqual(result['sha256'], selected['sha256'])
        self.assertTrue(job['log'].closed)


if __name__ == '__main__':
    unittest.main()
