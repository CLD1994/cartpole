"""报告的公开生命周期接口：真实写文件，仿真启动/退出由替身隔离。"""
import json
from contextlib import contextmanager
from pathlib import Path
import tempfile
import unittest
from evaluate_policy import reported_run, persist_progress


def cohort(seed=101):
    return dict(seed=seed,status='complete',requested=1,completed=1,
                episodes=[dict(env_id=0,success=True)])


class ReportLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path=Path(self.directory.name)/'report.json'
        self.report=dict(status='starting',seeds=[101],num_envs=1,cohorts=[])

    def read(self):
        return json.loads(self.path.read_text(encoding='utf8'))

    def test_final_report_written_before_launcher_exit(self):
        @contextmanager
        def launcher():
            yield
            self.assertEqual(self.read()['status'],'complete')
            raise SystemExit(0)  # Kit关闭后不再运行外层正常路径
        with self.assertRaises(SystemExit) as error:
            with launcher():
                with reported_run(self.path,self.report,active=[None],emit=lambda text:None):
                    self.report['cohorts'].append(cohort())
                    persist_progress(self.path,self.report)
        self.assertEqual(error.exception.code,0)
        saved=self.read()
        self.assertEqual(saved['summary']['status'],'complete')
        self.assertEqual(saved['summary']['success_rate'],1)

    def test_progress_has_no_final_success_rate_even_if_current_seed_complete(self):
        self.report['seeds']=[101,202]
        self.report['cohorts']=[cohort()]
        persist_progress(self.path,self.report)
        saved=self.read()
        self.assertEqual(saved['status'],'running')
        self.assertEqual(saved['summary']['status'],'running')
        self.assertIsNone(saved['summary']['success_rate'])

    def test_missing_seed_is_incomplete_and_exits_nonzero_before_shutdown(self):
        self.report['seeds']=[101,202]
        with self.assertRaises(SystemExit) as error:
            with reported_run(self.path,self.report,active=[None],emit=lambda text:None):
                self.report['cohorts'].append(cohort())
        self.assertEqual(error.exception.code,1)
        self.assertEqual(self.read()['status'],'incomplete')
        self.assertEqual(self.read()['summary']['status'],'incomplete')
        self.assertIsNone(self.read()['summary']['success_rate'])

    def test_error_clears_success_summary_and_preserves_partial_evidence(self):
        class Metrics:
            def report(self): return {'seed':202,'completed':0,'status':'incomplete'}
        self.report['cohorts']=[cohort()]
        with self.assertRaises(ValueError):
            with reported_run(self.path,self.report,active=[Metrics()],emit=lambda text:None):
                raise ValueError('measurement failed')
        saved=self.read()
        self.assertEqual(saved['status'],'error')
        self.assertEqual(saved['summary']['status'],'error')
        self.assertIsNone(saved['summary']['success_rate'])
        self.assertEqual(saved['interrupted_cohort']['seed'],202)

    def test_early_zero_exit_is_error_not_success(self):
        with self.assertRaises(SystemExit) as error:
            with reported_run(self.path,self.report,active=[None],emit=lambda text:None):
                raise SystemExit(0)
        self.assertEqual(error.exception.code,1)
        self.assertEqual(self.read()['status'],'error')

    def test_duplicate_seed_does_not_satisfy_requested_set(self):
        self.report['seeds']=[101,202]
        with self.assertRaises(SystemExit):
            with reported_run(self.path,self.report,active=[None],emit=lambda text:None):
                self.report['cohorts']=[cohort(),cohort()]
        self.assertEqual(self.read()['status'],'incomplete')

if __name__=='__main__': unittest.main()
