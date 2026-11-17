import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import run_policy as r
class SlowInferenceTests(unittest.TestCase):
    def run_branch(self,elapsed=.008,enabled=True,hard=.015):
        tree=ast.parse(Path(r.__file__).read_text(encoding='utf-8'))
        node=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and 'inference_limit_s is not None' in ast.unparse(n.test))
        stats={'slow_count':0,'maximum_decision_elapsed_s':0.,'maximum_forward_s':0.,'last_discard':None}
        def gate(reason): raise RuntimeError(reason)
        ctx=dict(CLOCK=lambda:elapsed,now=0.,inference_s=elapsed,inference_limit_s=.002,
                 a=NS(retry_slow_inference=enabled),report={'slow_inference':stats},
                 budget_at=0.,hard_limit_s=hard,end=1.,check_trial_end=r.check_trial_end,
                 gate=gate,obs=NS(received_at=0.),last_sample=0.)
        wrapper=ast.While(test=ast.Constant(True),body=[node,ast.Assign(targets=[ast.Name(id='action_reached',ctx=ast.Store())],value=ast.Constant(True)),ast.Break()],orelse=[])
        # A continue is the outer-loop return: stop this extracted loop to observe it.
        class Rewrite(ast.NodeTransformer):
            def visit_Continue(self,n):return ast.copy_location(ast.Break(),n)
        wrapper=Rewrite().visit(wrapper)
        try:exec(compile(ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[])),'<real slow branch>','exec'),ctx)
        finally:self.last=ctx
    def test_slow_candidate_discarded_even_if_under_observation_limit(self):
        self.run_branch(.003)
        self.assertNotIn('action_reached',self.last)
        self.assertEqual(self.last['last_sample'],0.)
        self.assertEqual(self.last['report']['slow_inference']['slow_count'],1)
    def test_eight_ms_stale_candidate_never_sent(self):
        self.run_branch(.008)
        self.assertNotIn('action_reached',self.last)
        self.assertEqual(self.last['budget_at'],0.)
    def test_hard_budget_remains_bounded(self):
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):self.run_branch(.015)
    def test_default_still_stops(self):
        with self.assertRaisesRegex(RuntimeError,'inference_deadline'):self.run_branch(enabled=False)
    def test_exact_two_ms_is_not_slow(self):
        self.run_branch(.002)
        self.assertTrue(self.last['action_reached'])
    def test_cli_optin_default_off(self):
        base=['--port','FAKE','--model','unused','--start-confirmed']
        self.assertFalse(r.parse_args(base).retry_slow_inference)
        self.assertTrue(r.parse_args(base+['--retry-slow-inference']).retry_slow_inference)


class SlowOuterLoopTests(unittest.TestCase):
    def harness(self):
        import importlib.util
        path=Path(r.__file__).parent/'integration_tests'/'test_angle_pause_runner.py'
        spec=importlib.util.spec_from_file_location('existing_angle_fixture',path)
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        return mod.RunnerCombinationTests()
    def test_slow_then_new_receipt_recomputes_and_sends(self):
        h=self.harness(); first=[True]
        def infer(clock):
            if first[0]:clock.now+=.008;first[0]=False
        port,report,calls=h.run_loop(lambda t:0,.02,policy_hook=infer,retry_slow=True,hard_limit=.015)
        self.assertEqual(report['slow_inference']['slow_count'],1)
        self.assertGreater(len(calls),1)
        self.assertGreater(calls[1][0],calls[0][0])
        self.assertTrue(report['samples'])
        self.assertTrue(all(s['observation_received_clock_s']>10. for s in report['samples']))
    def test_no_new_receipt_stops_at_gap_not_reinference(self):
        h=self.harness()
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            h.run_loop(lambda t:0,.03,policy_hook=lambda c:setattr(c,'now',c.now+.008),retry_slow=True,hard_limit=.015,no_new_state=True)
        port,report,calls,env=h.last_run
        self.assertEqual(len(calls),1)
        self.assertEqual(report['samples'],[])
        self.assertEqual(env['budget_at'],10.)
        self.assertAlmostEqual(env['CLOCK'](),10.015,places=9)
    def test_queued_fault_is_processed_before_retry(self):
        h=self.harness()
        with self.assertRaises(RuntimeError):
            h.run_loop(lambda t:0,.03,policy_hook=lambda c:setattr(c,'now',c.now+.008),retry_slow=True,hard_limit=.015,fault_at=.007)
        port,report,calls,env=h.last_run
        self.assertEqual(len(calls),1)
        self.assertEqual(report['samples'],[])
