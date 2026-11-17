import unittest
from write_budget import WriteBudget
class WriteBudgetTests(unittest.TestCase):
    def test_reserve_floor(self):
        self.assertEqual(WriteBudget().reserve_s,.001)
    def test_old_candidate_budget_not_renewed(self):
        b=WriteBudget(); self.assertTrue(b.insufficient(1.003978,1.004))
        self.assertFalse(b.insufficient(1.0029,1.004))
        self.assertTrue(b.insufficient(1.004,1.004))
    def test_cutoff_never_extends_deadline(self):
        b=WriteBudget(); self.assertAlmostEqual(b.cutoff(1.004),1.003)

class BudgetCallsiteTests(unittest.TestCase):
    def test_actual_discard_preserves_observation_and_gap_deadlines(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace as NS
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf-8'))
        fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='discard_write_budget')
        now=[.0036]; info={'discard_count':0,'discard_stages':{},'last_discard':None}
        def gate(reason): raise RuntimeError(reason)
        ctx=dict(CLOCK=lambda:now[0],write_budget=WriteBudget(),check_trial_end=lambda *a:None,
                 end=1.,budget_at=0.,hard_limit_s=.009,gate=gate,report={'write_budget':info},stage='prewrite')
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<real discard>','exec'),ctx)
        obs=NS(received_at=0.)
        self.assertTrue(ctx['discard_write_budget'](obs,.004))
        self.assertEqual(info['last_discard']['observation_received_clock_s'],0.)
        self.assertEqual(ctx['budget_at'],0.)
        now[0]=.0086
        self.assertTrue(ctx['discard_write_budget'](obs,.009))
        now[0]=.009
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            ctx['discard_write_budget'](obs,.009)
    def test_final_guard_precedes_write_and_same_observation_waits(self):
        import ast
        from pathlib import Path
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf-8'))
        loops=[n for n in ast.walk(tree) if isinstance(n,ast.While)]
        outer=next(n for n in loops if any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='velocity' for t in x.targets) for x in n.body))
        index=next(i for i,n in enumerate(outer.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='velocity' for t in n.targets))
        guard=outer.body[index-3]
        self.assertIsInstance(guard,ast.If)
        self.assertEqual(guard.test.func.id,'discard_write_budget')
        self.assertIsInstance(guard.body[0],ast.Continue)
        duplicate=next(n for n in outer.body if isinstance(n,ast.If) and ast.unparse(n.test)=='obs.received_at == last_sample')
        self.assertIsInstance(duplicate.body[-1],ast.Continue)
        # Consumed receipt remains assigned before inference, never renewed by latest STATE.
        assigned=[ast.unparse(n) for n in outer.body if isinstance(n,ast.Assign)]
        self.assertIn('last_sample = obs.received_at',assigned)
        self.assertNotIn('obs = latest',assigned)
