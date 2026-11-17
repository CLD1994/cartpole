import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from threaded_receiver import ThreadedReceiver

class PausedReceiverPolicyTests(unittest.TestCase):
    def make(self,delay=.012):
        now=[1.];data=[b'STATE 3 0 0 0 0\n']
        class Source:
            @property
            def in_waiting(self):return len(data[0])
            def read(self,n):now[0]+=delay;out=data[0];data[0]=b'';return out
        r=ThreadedReceiver(Source(),Mock(),lambda:now[0])
        return r,now
    def test_paused_read_twelve_ms_ok_strict_read_still_fails(self):
        r,now=self.make();r.request_pause();r._select_policy()
        self.assertTrue(r.raw.poll());self.assertEqual(r.raw.limits.max_read_s,.1)
        strict,_=self.make();strict._select_policy()
        self.assertFalse(strict.raw.poll());self.assertEqual(strict.raw.error,'read_deadline')
    def test_pause_read_over100ms_still_fails(self):
        r,_=self.make(.101);r.request_pause();r._select_policy()
        self.assertFalse(r.raw.poll());self.assertEqual(r.raw.error,'read_deadline')
    def test_policy_request_does_not_change_inflight_limits(self):
        r,_=self.make();r._select_policy();old=r.raw.limits
        r.request_pause();self.assertIs(r.raw.limits,old)
        r._select_policy();self.assertIs(r.raw.limits,r.paused_limits)
    def test_resume_needs_strict_ack_new_receipt_and_clean_view(self):
        r,now=self.make();r.request_pause();r._select_policy()
        obs=NS(received_at=1.001)
        self.assertFalse(r.resume_ready(obs));self.assertTrue(r.resume_pending)
        self.assertFalse(r.resume_ready(obs)) # old paused ack
        now[0]=1.002;r._polling=True;r._select_policy()
        self.assertFalse(r.resume_ready(NS(received_at=1.003)))
        r._polling=False
        self.assertFalse(r.resume_ready(obs)) # before strict boundary
        fresh=NS(received_at=1.003)
        r.source_pending=20;self.assertFalse(r.resume_ready(fresh));r.source_pending=0
        r._partial=True;self.assertFalse(r.resume_ready(fresh));r._partial=False
        self.assertTrue(r.resume_ready(fresh))
    def test_cancel_resume_keeps_fault_and_active_backlog_latched(self):
        r,_=self.make();r.error='read_deadline';r.active_backlog_seen=150
        r.resume_pending=True;r.request_pause();r._select_policy()
        self.assertFalse(r.resume_pending);self.assertEqual(r.error,'read_deadline')
        self.assertEqual(r.active_backlog_seen,150)
        self.assertFalse(r.resume_ready(NS(received_at=2.)))

    def test_pause_poll_gap25ms_ok_active_still_fails(self):
        r,now=self.make(0.);r.request_pause();r._select_policy();self.assertTrue(r.raw.poll())
        now[0]+=.025;self.assertTrue(r.raw.poll())
        r,now=self.make(0.);r._select_policy();self.assertTrue(r.raw.poll())
        now[0]+=.025;self.assertFalse(r.raw.poll());self.assertEqual(r.raw.error,'poll_deadline')
    def test_paused_backlog_not_latched_active_backlog_is(self):
        source=NS(in_waiting=200);r=ThreadedReceiver(source,Mock(),lambda:1.)
        r.request_pause();r._polling=True;r._select_policy()
        self.assertEqual(r.raw.source.in_waiting,200);self.assertEqual(r.active_backlog_seen,0)
        r.pause_requested=False;r.resume_pending=True;r._select_policy()
        self.assertEqual(r.raw.source.in_waiting,200);self.assertEqual(r.active_backlog_seen,200)

    def test_legal_paused_long_read_does_not_poison_strict_poll_interval(self):
        r,now=self.make(.05);r.request_pause();r._select_policy()
        self.assertTrue(r.raw.poll())
        r.pause_requested=False;r.resume_pending=True;r._select_policy()
        self.assertEqual(r.raw.limits.max_read_s,.01)
        self.assertTrue(r.raw.poll(previous_poll_limit_s=r._previous_poll_limit_s))
        r._select_policy();self.assertIsNone(r._previous_poll_limit_s)
        now[0]+=.025
        self.assertFalse(r.raw.poll());self.assertEqual(r.raw.error,'poll_deadline')
    def test_transition_does_not_erase_over100ms_gap(self):
        r,now=self.make(0.);r.request_pause();r._select_policy();self.assertTrue(r.raw.poll())
        now[0]+=.101;r.pause_requested=False;r.resume_pending=True;r._select_policy()
        self.assertFalse(r.raw.poll(previous_poll_limit_s=r._previous_poll_limit_s))
        self.assertEqual(r.raw.error,'poll_deadline')
    def test_transition_current_strict_read_still_has10ms_limit(self):
        now=[1.];delays=iter([.05,.012])
        class Source:
            @property
            def in_waiting(self):return 16
            def read(self,n):now[0]+=next(delays);return b'STATE 3 0 0 0 0\n'
        r=ThreadedReceiver(Source(),Mock(),lambda:now[0]);r.request_pause();r._select_policy()
        self.assertTrue(r.raw.poll())
        r.pause_requested=False;r.resume_pending=True;r._select_policy()
        self.assertFalse(r.raw.poll(previous_poll_limit_s=r._previous_poll_limit_s))
        self.assertEqual(r.raw.error,'read_deadline')

