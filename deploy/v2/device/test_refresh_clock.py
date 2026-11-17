import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

class RefreshClockTests(unittest.TestCase):
    def run_prefix(self,refresh_delay=0.,pause_delay=0.):
        tree=ast.parse(Path(__file__).with_name('run_policy.py').read_text(encoding='utf-8'))
        loop=next(n for n in ast.walk(tree) if isinstance(n,ast.While) and ast.unparse(n.test)=='CLOCK() < end')
        stop=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and ast.unparse(n)=="stage = 'tick'")
        body=loop.body[:stop]+[ast.Break()]
        now=[.001]; sleeps=[]; pauses=[]
        def sleep(delay): sleeps.append(delay); raise StopIteration
        def update_pause(obs,t): pauses.append(t); now[0]+=pause_delay
        def gate(reason): raise RuntimeError(reason)
        def refresh(*a): now[0]+=refresh_delay
        env=dict(CLOCK=lambda:now[0],time=NS(perf_counter_ns=lambda:round(now[0]*1e9),sleep=sleep),
                 cycle=0,timeline=NS(mark=lambda *a:None),transport=NS(in_waiting=0),
                 receiver=NS(backlog_ready=lambda obs:True,poll=lambda:True,error=None),session=NS(pole_waiting=False,phase='active',failure=None,observation=None),
                 Phase=NS(ACTIVE='active'),refresh=NS(update=refresh),update_pause=update_pause,
                 pause=NS(paused=False),a=NS(diagnostic_timing=False),budget_at=0.,hard_limit_s=.009,
                 gate=gate,next_tick_ns=2500000,next_tick=.0025,end=1.)
        code=compile(ast.fix_missing_locations(ast.Module(body=[ast.While(test=ast.Constant(True),body=body,orelse=[])],type_ignores=[])),'<real runner prefix>','exec')
        try:exec(code,env)
        except StopIteration:pass
        finally:self.last=(sleeps,pauses,env)
    def test_long_refresh_gates_without_extra_sleep(self):
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            self.run_prefix(refresh_delay=.009)
        self.assertEqual(self.last[0],[])
        self.assertAlmostEqual(self.last[1][0],.010)
        self.assertEqual(self.last[2]['budget_at'],0.)
    def test_refresh_crossing_grid_does_not_sleep(self):
        self.run_prefix(refresh_delay=.002)
        self.assertEqual(self.last[0],[])
    def test_pause_stage_delay_also_refreshes_gap(self):
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            self.run_prefix(pause_delay=.009)
        self.assertEqual(self.last[0],[])
    def test_normal_wait_remains_bounded(self):
        self.run_prefix()
        self.assertEqual(self.last[0],[.0002])
    def test_exact_gap_boundary_gates_without_sleep(self):
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            self.run_prefix(refresh_delay=.008)
        self.assertEqual(self.last[0],[])
        self.assertEqual(self.last[2]['now_ns'],9000000)
        self.assertEqual(self.last[2]['now'],.009)
