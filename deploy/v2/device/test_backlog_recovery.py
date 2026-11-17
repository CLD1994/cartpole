import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from threaded_receiver import ThreadedReceiver

class BacklogRecoveryTests(unittest.TestCase):
    def make(self):
        now=[10.]
        source=NS(in_waiting=145)
        reader=ThreadedReceiver(source,Mock(),lambda:now[0]);reader.active=True
        return reader,source,now
    def test_historical_peak_is_not_current_pending(self):
        r,s,n=self.make();self.assertEqual(r.raw.source.in_waiting,145)
        s.in_waiting=0;self.assertEqual(r.raw.source.in_waiting,0)
        self.assertEqual(r.in_waiting,0)
        self.assertEqual(r.active_backlog_seen,145)
    def test_requires_post_drain_observation_not_just_backlog_batch(self):
        r,s,n=self.make();r.raw.source.in_waiting
        s.in_waiting=0;r.raw.source.in_waiting
        self.assertFalse(r.backlog_ready(NS(received_at=10.)))
        n[0]=10.001
        self.assertFalse(r.backlog_ready(NS(received_at=10.)))
        self.assertTrue(r.backlog_ready(NS(received_at=10.001)))
        self.assertFalse(r.backlog_ready(NS(received_at=9.999)))
    def test_partial_and_inflight_delay_barrier(self):
        r,s,n=self.make();r.raw.source.in_waiting;s.in_waiting=0;r.raw.source.in_waiting
        for field in ('_partial','_polling','_busy'):
            setattr(r,field,True)
            self.assertFalse(r.backlog_ready(NS(received_at=10.001)))
            setattr(r,field,False)
        self.assertFalse(r.backlog_ready(NS(received_at=10.001)))
    def test_queue_accumulation_recovers_without_dropping_event(self):
        r,s,n=self.make();s.in_waiting=0
        rows=['STATE 3 0 0 0 0']*8+['EVENT STEP_LOSS 1 1 42','MODE FAULT MT6835']
        for row in rows:r.enqueue(row,10.)
        self.assertFalse(r.backlog_ready(NS(received_at=10.)))
        r.poll()
        self.assertEqual([c.args[0] for c in r.session.receive_at.call_args_list],rows)
        self.assertFalse(r.backlog_ready(NS(received_at=10.)))
    def test_new_congestion_invalidates_previous_candidate(self):
        r,s,n=self.make();r.raw.source.in_waiting;s.in_waiting=0;r.raw.source.in_waiting
        r.backlog_ready(NS(received_at=10.));n[0]=10.001
        self.assertTrue(r.backlog_ready(NS(received_at=10.001)))
        s.in_waiting=150;r.raw.source.in_waiting
        self.assertFalse(r.backlog_ready(NS(received_at=10.001)))
    def test_failure_never_recovers(self):
        r,s,n=self.make();r.error='receive_queue_overflow'
        self.assertFalse(r.backlog_ready(NS(received_at=10.001)))

    def test_actual_runner_precommand_discards_congested_action(self):
        import ast
        from pathlib import Path
        import run_policy
        tree=ast.parse(Path(run_policy.__file__).read_text(encoding='utf-8'))
        loop=min((n for n in ast.walk(tree) if isinstance(n,ast.While) and any(
            isinstance(x,ast.Constant) and x.value=='precommand_poll_begin' for x in ast.walk(n))),
            key=lambda n:sum(1 for _ in ast.walk(n)))
        r,s,n=self.make();r.raw.source.in_waiting
        context=dict(CLOCK=lambda:10.,end=11.,check_trial_end=lambda *a:None,
            discard_write_budget=lambda *a:False,receive_deadline=10.004,
            receiver=r,obs=NS(received_at=9.999),retry_candidate=False)
        exec(compile(ast.Module(body=[loop],type_ignores=[]),'<precommand>','exec'),context)
        self.assertTrue(context['retry_candidate'])
    def test_recovery_wait_does_not_reset_active_budget(self):
        import ast
        from pathlib import Path
        import run_policy
        tree=ast.parse(Path(run_policy.__file__).read_text(encoding='utf-8'))
        loop=next(n for n in ast.walk(tree) if isinstance(n,ast.While) and ast.unparse(n.test)=='CLOCK() < end')
        start=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and ast.unparse(n)=="stage = 'gap_check'")
        stop=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.If) and ast.unparse(n.test)=='now_ns < next_tick_ns')
        body=loop.body[start:stop]
        clock=[10.];r,s,n=self.make();r.raw.source.in_waiting
        def gate(reason):raise RuntimeError(reason)
        context=dict(CLOCK=lambda:clock[0],time=NS(perf_counter_ns=lambda:round(clock[0]*1e9),sleep=lambda delay:clock.__setitem__(0,clock[0]+max(delay,.000001))),
            receiver=r,session=NS(observation=NS(received_at=10.)),budget_at=10.,hard_limit_s=.015,end=11.,gate=gate,
            timeline=Mock(),cycle=1)
        code=compile(ast.fix_missing_locations(ast.Module(body=[ast.While(test=ast.Constant(True),body=body,orelse=[])],type_ignores=[])),'<recovery>','exec')
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):exec(code,context)
        self.assertEqual(context['budget_at'],10.)
        self.assertLess(clock[0],10.01501)

    def test_runner_reinfers_after_congestion_in_inference(self):
        from test_paused_wait import PausedWaitTests
        harness=PausedWaitTests().harness()
        gate={'at':None,'armed':True}
        def infer(clock):
            if gate['armed']:
                gate['at']=clock.now;gate['armed']=False
        def ready(obs):
            return gate['at'] is None or obs.received_at > gate['at']+.001
        port,report,calls=harness.run_loop(lambda t:0,.01,policy_hook=infer,hard_limit=.015,backlog_ready=ready)
        self.assertGreaterEqual(len(calls),2)
        writes=[at for at,data in port.sent if data.startswith(b'VEL')]
        self.assertTrue(writes)
        self.assertGreater(writes[0],calls[0][0]+.001)
        self.assertEqual(writes[0],calls[1][0])
    def test_fault_is_processed_during_backlog_wait(self):
        from test_paused_wait import PausedWaitTests
        harness=PausedWaitTests().harness()
        with self.assertRaisesRegex(RuntimeError,'firmware_mode|session_inactive|firmware_fault'):
            harness.run_loop(lambda t:10,.02,hard_limit=.015,fault_at=.005,backlog_ready=lambda obs:False)
        self.assertEqual(harness.last_run[2],[])

    def test_poll_lifecycle_publication_is_atomic_with_drain_check(self):
        class CheckedReceiver(ThreadedReceiver):
            monitor=False
            def __setattr__(self,key,value):
                if self.monitor and key in ('_polling','_partial'):
                    if not self.lock.locked():raise AssertionError('poll state outside drain lock')
                super().__setattr__(key,value)
        reader=CheckedReceiver(Mock(),Mock(),lambda:10.)
        reader.monitor=True
        def poll():reader.stop_event.set();return True
        reader.raw=NS(poll=poll,_pending=b'')
        reader._run()
        self.assertIsNone(reader.error)
        self.assertFalse(reader.partial)
