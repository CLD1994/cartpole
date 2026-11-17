import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

class PrecommandYieldTests(unittest.TestCase):
    def run_loop(self, pending=27, partial=True, clear=True, fault=False, deadline=.00025, reserve=False):
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf-8'))
        loop=min((n for n in ast.walk(tree) if isinstance(n,ast.While) and any(isinstance(x,ast.Constant) and x.value=='precommand_poll_begin' for x in ast.walk(n))), key=lambda n:sum(1 for _ in ast.walk(n)))
        clock=[0.]; sleeps=[]; polls=[0]
        transport=NS(in_waiting=pending,partial=partial)
        session=NS(pole_waiting=False,phase='active',failure=None,observation=object())
        def poll():
            polls[0]+=1
            if polls[0]>10: raise AssertionError('busy spin without yielding')
            return not fault
        def sleep(delay):
            sleeps.append(delay); clock[0]+=delay
            if clear: transport.in_waiting=0; transport.partial=False
        def gate(reason): raise RuntimeError(reason)
        context=dict(discard_write_budget=lambda *a:False, write_budget=NS(cutoff=lambda d:d), CLOCK=lambda:clock[0],end=1.,receive_deadline=deadline,
            check_trial_end=lambda *a:None,discard_expired_candidate=lambda o:False,
            obs=object(),gate=gate,transport=transport,receiver=NS(backlog_ready=lambda obs:True,poll=poll,error='fault' if fault else None),
            session=session,Phase=NS(ACTIVE='active'),timeline=NS(mark=lambda *a:None),cycle=1,
            drain={'polls':0,'wait_s':0.,'started_clock_s':0.},refresh=None,
            state_guard=lambda *a:False,report={},update_pause=lambda *a:None,time=NS(sleep=sleep))
        if reserve:
            from write_budget import WriteBudget
            fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='discard_write_budget')
            context.update(write_budget=WriteBudget(),budget_at=0.,hard_limit_s=.009,stage='precommand_receive')
            context['obs']=NS(received_at=0.)
            context['report']['write_budget']={'discard_count':0,'discard_stages':{},'last_discard':None}
            exec(compile(ast.Module(body=[fn],type_ignores=[]),'<real discard>','exec'),context)
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

    def test_real_budget_discards_before_poll_when_only_22us_left(self):
        self.run_loop(deadline=.0000227,reserve=True)
        self.assertTrue(self.last[3]['retry_candidate'])
        self.assertEqual(self.last[2][0],0)
        self.assertEqual(self.last[3]['receive_deadline'],.0000227)
    def test_real_budget_clean_view_with_enough_time(self):
        self.run_loop(deadline=.004,pending=0,partial=False,reserve=True)
        self.assertEqual(self.last[2][0],1)
        self.assertEqual(self.last[1],[])
    def test_real_drain_stops_at_reserved_cutoff_without_extending_deadline(self):
        self.run_loop(deadline=.0012,reserve=True,clear=False)
        clock,sleeps,polls,ctx=self.last
        self.assertTrue(ctx['retry_candidate'])
        self.assertAlmostEqual(clock[0],.0002)
        self.assertEqual(ctx['receive_deadline'],.0012)
        self.assertTrue(all(0<=s<=.0001 for s in sleeps))
