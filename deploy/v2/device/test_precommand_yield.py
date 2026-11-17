import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

class PrecommandYieldTests(unittest.TestCase):
    def run_loop(self, pending=27, partial=True, clear=True, fault=False, deadline=.00025):
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf-8'))
        loop=min((n for n in ast.walk(tree) if isinstance(n,ast.While) and any(isinstance(x,ast.Constant) and x.value=='precommand_poll_begin' for x in ast.walk(n))), key=lambda n:sum(1 for _ in ast.walk(n)))
        clock=[0.]; sleeps=[]; polls=[0]
        transport=NS(in_waiting=pending,partial=partial)
        session=NS(phase='active',failure=None,observation=object())
        def poll():
            polls[0]+=1
            if polls[0]>10: raise AssertionError('busy spin without yielding')
            return not fault
        def sleep(delay):
            sleeps.append(delay); clock[0]+=delay
            if clear: transport.in_waiting=0; transport.partial=False
        def gate(reason): raise RuntimeError(reason)
        context=dict(CLOCK=lambda:clock[0],end=1.,receive_deadline=deadline,
            check_trial_end=lambda *a:None,discard_expired_candidate=lambda o:False,
            obs=object(),gate=gate,transport=transport,receiver=NS(poll=poll,error='fault' if fault else None),
            session=session,Phase=NS(ACTIVE='active'),timeline=NS(mark=lambda *a:None),cycle=1,
            drain={'polls':0,'wait_s':0.,'started_clock_s':0.},refresh=None,
            state_guard=lambda *a:False,report={},update_pause=lambda *a:None,time=NS(sleep=sleep))
        try: exec(compile(ast.Module(body=[loop],type_ignores=[]),'<real precommand loop>','exec'),context)
        finally: self.last=(clock,sleeps,polls,context)
    def test_cached_positive_pending_yields_and_worker_progress_exits(self):
        self.run_loop()
        clock,sleeps,polls,ctx=self.last
        self.assertEqual(sleeps,[.0001]); self.assertEqual(polls[0],2)
        self.assertEqual(ctx['receive_deadline'],.00025)
    def test_partial_without_pending_yields(self):
        self.run_loop(pending=0)
        self.assertEqual(self.last[1],[.0001])
    def test_deadline_is_not_extended(self):
        with self.assertRaisesRegex(RuntimeError,'precommand_receive_deadline'):
            self.run_loop(clear=False)
        clock,sleeps,polls,ctx=self.last
        self.assertAlmostEqual(clock[0],.00025)
        self.assertTrue(all(0<=s<=.0001 for s in sleeps))
        self.assertEqual(ctx['receive_deadline'],.00025)
    def test_fault_and_clean_view_do_not_sleep(self):
        with self.assertRaisesRegex(RuntimeError,'fault'): self.run_loop(fault=True)
        self.assertEqual(self.last[1],[])
        self.run_loop(pending=0,partial=False)
        self.assertEqual(self.last[1],[])
