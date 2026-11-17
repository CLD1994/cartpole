"""实际 runner while 循环与 Receiver/BoundSession/ReadbackCycle 组合；无硬件。"""
import ast
import math
import re
from pathlib import Path
from types import SimpleNamespace
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unittest.mock import patch
import run_policy as r
from receiver import Receiver
from session import Limits

class Clock:
    now = 10.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += max(seconds, .00001)

class Port:
    partial = False
    def __init__(self, clock, angle):
        self.clock, self.angle = clock, angle
        self.pending=b''; self.last=None; self.sent=[]
        self.values={'CAL_STATE':2,'CAL_SAVED':1,'CAL_STORAGE':0,'CAL_LENGTH_UM':853940,
            'VEL_MAX':1250,'ACC_MAX':10000,'WDT_MS':100,'X_MARGIN_UM':10000,
            'POLE_ZERO_URAD':0,'POLE_INV':1,'X_INV':0,'STEP_LOSS_UM':3000}
    @property
    def in_waiting(self):
        if self.last != self.clock.now and not (getattr(self,'no_new_state',False) and self.clock.now>10.):
            self.pending += f'STATE 3 0 {round(math.radians(self.angle())*1e6)} 0 0\n'.encode()
            self.last=self.clock.now
        return len(self.pending)
    def read(self, n):
        out,self.pending=self.pending[:n],self.pending[n:];return out
    def write(self, data):
        self.sent.append((self.clock.now,data))
        if data==b'QUERY\n': self.pending+=b'MODE READY NONE\n'
        elif data==b'CALIB?\n':
            self.pending+=''.join(f'OK {k} {self.values[k]}\n' for k in ('CAL_STATE','CAL_LENGTH_UM','CAL_SAVED','CAL_STORAGE')).encode()
        elif data.startswith(b'CONFIG? '):
            k=data.decode().split()[1];self.pending+=f'OK {k} {self.values[k]}\n'.encode()
        return len(data)

class RunnerCombinationTests(unittest.TestCase):
    def run_loop(self, angle, duration, policy_hook=None, diagnostic=False, fault_at=None, retry_slow=False, hard_limit=.009, no_new_state=False, backlog_ready=None, invalid_pole=None, resume_hook=None, read_hook=None):
        clock=Clock();port=Port(clock,lambda:angle(clock.now-10))
        port.no_new_state=no_new_state
        original_read=port.read
        fault_sent=[False]
        def read(n):
            if read_hook:read_hook(clock)
            if fault_at is not None and clock.now-10 >= fault_at and not fault_sent[0]:
                port.pending=b'MODE FAULT MT6835\n'+port.pending
                fault_sent[0]=True
            data=original_read(n)
            if invalid_pole and invalid_pole(clock.now-10):
                data=re.sub(rb'(?m)^STATE [^\n]*\n', b'STATE 1 0 0 0 0\n', data)
            assert len(data)<=n, (n, data)
            return data
        port.read=read
        session=r.BoundSession(port,clock,Limits(500,1250,10000,10000),{'POLE_INV':1,'X_INV':0})
        session.receive('MODE READY NONE')
        for k,v in port.values.items():session.receive(f'OK {k} {v}')
        session.receive('STATE 3 0 0 0 0');session.bind_calibration();session.start()
        receiver=Receiver(port,session,clock)
        receiver.pause_calls=[]
        receiver.request_pause=lambda:receiver.pause_calls.append(clock.now)
        receiver.resume_ready=lambda obs:resume_hook(clock) if resume_hook else True
        receiver.active_backlog_seen=0
        receiver.backlog_ready=backlog_ready or (lambda obs:True)
        report={'angle_pause':{'events':[],'pause_count':0,'resume_count':0,'intentional_completion_gaps':0},
            'readback_refresh':[],'run_totals':dict.fromkeys(('command_completions','command_gaps_over_4ms',
            'command_gaps_at_or_over_6ms','command_gaps_at_or_over_hard_limit','maximum_command_gap_s','maximum_active_command_gap_s',
            'inference_attempts','inference_total_s','maximum_inference_s'),0),
            'samples':[],'inference_attempts_s':[],'discarded_stale_candidates':0,'candidate_refresh_stages':{},
            'duplicate_tick_count':0,'stale_observation_waits':0,'max_stale_observation_age_s':0.,
            'over_period_late_candidates':0,'grid_slots_skipped':0}
        report['write_budget']={'discard_count':0,'discard_stages':{},'last_discard':None}
        report['run_totals']['command_gaps_at_or_over_9ms']=0
        report['slow_inference']={'slow_count':0,'maximum_decision_elapsed_s':0.,'maximum_forward_s':0.,'last_discard':None}
        calls=[]
        def forward(state):
            calls.append((clock.now,state))
            if policy_hook:policy_hook(clock)
            return .1
        tree=ast.parse(Path(r.__file__).read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        funcs=[]
        for node in main.body:
            if isinstance(node,ast.FunctionDef) and node.name in ('update_pause','discard_expired_candidate','discard_write_budget'):
                for i,n in enumerate(node.body):
                    if isinstance(n,ast.Nonlocal):node.body[i]=ast.Global(names=n.names)
                funcs.append(node)
        loop=next(n for n in ast.walk(main) if isinstance(n,ast.While) and ast.unparse(n.test)=='CLOCK() < end'
                  and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='stage' for t in x.targets)
                          and isinstance(x.value,ast.Constant) and x.value.value=='poll' for x in n.body))
        def gate(reason):
            if not diagnostic:raise RuntimeError(reason)
        env=dict(vars(r));env.update(CLOCK=clock,time=SimpleNamespace(perf_counter_ns=lambda:round(clock.now*1e9),sleep=clock.sleep),
            session=session,receiver=receiver,transport=port,policy=SimpleNamespace(forward=forward),
            write_budget=r.WriteBudget(),timeline=r.GapTimeline(False),cycle=0,report=report,a=SimpleNamespace(diagnostic_timing=diagnostic,retry_slow_inference=retry_slow),pause=r.AnglePause(),gate=gate,
            hard_limit_s=hard_limit,observation_limit_s=.004,inference_limit_s=.002,
            pole_resumed_since_command=False,budget_at=10.,last_command_at=10.,start=10.,end=10.+duration,
            grid_start_ns=10_000_000_000,next_tick_ns=10_000_000_000,next_tick=10.,next_slot=0,
            period_ns=2_000_000,last_sample=None,last_tick=None,intervals=[],probe=None,
            refresh=r.ReadbackCycle(port,report['readback_refresh'],10.))
        self.last_run=(port,report,calls,env)
        with patch.object(r,'CLOCK',clock):
            code=compile(ast.fix_missing_locations(ast.Module(body=funcs+[loop],type_ignores=[])),'<actual-runner-loop>','exec')
            try:exec(code,env)
            except r.TrialEnd:pass
        return port,report,calls
    def test_pole_recovery_waits_past_command_budget_and_reinfers(self):
        port,report,calls=self.run_loop(lambda t:0,.08,invalid_pole=lambda t:.004<t<.05)
        self.assertFalse(any(.004<at-10<.05 for at,_ in calls))
        self.assertFalse(any(.004<at-10<.05 and data.startswith(b'VEL') for at,data in port.sent))
        self.assertTrue(any(at-10>=.05 for at,_ in calls))
        self.assertTrue(any(row.get('intentional_pole_recovery_gap') for row in report['samples']))

    def test_pole_invalid_stream_past_watchdog_waits_without_velocity_writes(self):
        port,report,calls=self.run_loop(lambda t:0,.3,invalid_pole=lambda t:.004<t<.25)
        self.assertFalse(any(.004<at-10<.25 and data.startswith(b'VEL') for at,data in port.sent))
        self.assertTrue(any(at-10>=.25 for at,_ in calls))

    def test_fault_during_pole_wait_ends_control(self):
        with self.assertRaisesRegex(RuntimeError,'firmware_mode|session_inactive'):
            self.run_loop(lambda t:0,.03,invalid_pole=lambda t:t>.004,fault_at=.02)

    def test_recovery_candidate_expiring_during_handshake_waits_for_another(self):
        count=[0]
        def resume(clock):
            count[0]+=1
            if count[0]==1:clock.now+=.005
            return True
        port,report,calls=self.run_loop(lambda t:0,.05,invalid_pole=lambda t:.004<t<.01,resume_hook=resume)
        self.assertGreater(count[0],1)
        self.assertTrue(any(at-10>.015 for at,_ in calls))

    def test_invalid_again_during_recovery_requests_pause_again(self):
        until=[0.];tries=[0]
        def resume(clock):
            tries[0]+=1
            if tries[0]==1:
                until[0]=clock.now-10+.004
                return False
            return True
        def invalid(t):return .004<t<.01 or .01<t<until[0]
        port,report,calls=self.run_loop(lambda t:0,.05,invalid_pole=invalid,resume_hook=resume)
        receiver=self.last_run[3]['receiver']
        self.assertTrue(any(.01<at-10<until[0] for at in receiver.pause_calls))
        self.assertGreater(tries[0],1)
        self.assertTrue(any(at-10>=until[0] for at,_ in calls))

    def test_overdue_active_gap_before_first_invalid_is_not_forgiven(self):
        delayed=[False]
        def read_delay(clock):
            if clock.now-10>.004 and not delayed[0]:
                delayed[0]=True;clock.now+=.008
        with self.assertRaisesRegex(RuntimeError,'command_gap_deadline'):
            self.run_loop(lambda t:0,.05,invalid_pole=lambda t:t>.004,read_hook=read_delay,hard_limit=.006)

    def test_pole_invalid_during_inference_discards_candidate(self):
        invalid={'yes':False}
        def infer(clock):invalid['yes']=True;clock.now+=.00005
        port,report,calls=self.run_loop(lambda t:0,.02,infer,invalid_pole=lambda t:invalid['yes'])
        self.assertEqual(len(calls),1)
        self.assertFalse(any(data.startswith(b'VEL') for _,data in port.sent))

    def test_pole_invalid_during_angle_pause_preserves_angle_pause(self):
        port,report,calls=self.run_loop(lambda t:80,.08,invalid_pole=lambda t:.004<t<.05)
        self.assertEqual(calls,[])
        self.assertFalse(any(data.startswith(b'VEL') for _,data in port.sent))

    def test_pause_over_one_second_refreshes_metadata_and_resumes(self):
        port,report,calls=self.run_loop(lambda t:80 if t<1.1 else 0,1.12)
        self.assertTrue(any(data==b'QUERY\n' for _,data in port.sent))
        self.assertFalse(any(data.startswith((b'VEL',b'DISABLE')) for at,data in port.sent if at<11.1))
        self.assertTrue(any(data.startswith(b'VEL') for at,data in port.sent if at>=11.1))
        self.assertTrue(all(at>=11.1 for at,_ in calls))
        self.assertEqual(report['angle_pause']['pause_count'],1)
        self.assertEqual(report['angle_pause']['resume_count'],1)
        first=report['samples'][0]
        self.assertTrue(first['intentional_angle_pause_gap'])
        self.assertGreater(first['command_gap_s'],1.)
    def test_prewrite_new_angle_discards_candidate(self):
        state={'outside':False}
        def infer(clock):state['outside']=True;clock.now+=.00005
        port,report,calls=self.run_loop(lambda t:80 if state['outside'] else 0,.01,infer)
        self.assertEqual(len(calls),1)
        self.assertFalse(any(data.startswith(b'VEL') for _,data in port.sent))
        self.assertEqual(report['angle_pause']['pause_count'],1)

    def test_fault_during_pause_still_ends_control(self):
        with self.assertRaisesRegex(RuntimeError,'firmware_mode|session_inactive|firmware_fault'):
            self.run_loop(lambda t:80,.02,fault_at=.005)
    def test_diagnostic_timing_does_not_pause(self):
        _,report,calls=self.run_loop(lambda t:80,.01,diagnostic=True)
        self.assertTrue(calls)
        self.assertEqual(report['angle_pause']['pause_count'],0)

if __name__=='__main__':unittest.main()
