import unittest
from unittest.mock import Mock
from receiver import Receiver, ReceiveLimits
from session import Session
from gap_timeline import GapTimeline

class ReceiverTimelineTests(unittest.TestCase):
    def run_case(self, payload, read_error=False):
        calls=[]
        class Source:
            @property
            def in_waiting(self):
                calls.append('waiting')
                return len(payload) if len(calls)==1 else 0
            def read(self,count):
                calls.append(('read',count))
                if read_error: raise OSError('test')
                return payload
        t=GapTimeline(True,wall=lambda:1,cpu=lambda:0);t.mark('loop_poll_begin',42)
        r=Receiver(Source(),Mock(spec=Session),lambda:1.,timeline=t)
        result=r.poll()
        return result,r,calls,t.snapshot()['events'][1:]
    def test_records_actual_calls_without_extra_reads(self):
        ok,r,calls,events=self.run_case(b'A\n')
        self.assertTrue(ok);self.assertEqual(calls,['waiting',('read',2),'waiting'])
        self.assertEqual([e['stage'] for e in events],['receiver_count_call','receiver_count_return','receiver_read_call','receiver_read_return','receiver_remaining_call','receiver_remaining_return'])
        self.assertTrue(all(e['cycle']==42 for e in events))
        self.assertEqual(events[1]['io'],{'count':2})
        self.assertEqual(events[3]['io'],{'read_bytes':2})
        self.assertEqual(events[5]['io'],{'remaining':0, 'read_window_s':0.})
    def test_zero_count_records_skip_not_read(self):
        ok,r,calls,events=self.run_case(b'')
        self.assertTrue(ok);self.assertEqual(calls,['waiting','waiting'])
        self.assertIn('receiver_read_skipped',[e['stage'] for e in events])
        self.assertNotIn('receiver_read_call',[e['stage'] for e in events])
    def test_read_failure_preserves_original_error_and_unreturned_boundary(self):
        ok,r,calls,events=self.run_case(b'A',True)
        self.assertFalse(ok);self.assertEqual(r.error,'read_error')
        self.assertEqual(events[-1]['stage'],'receiver_read_call')
    def test_original_read_window_and_deadline_remain_unchanged(self):
        t=GapTimeline(True,wall=lambda:1,cpu=lambda:0)
        times=iter([1.,1.011])
        class Source:
            in_waiting=0
        r=Receiver(Source(),Mock(spec=Session),lambda:next(times),timeline=t)
        self.assertFalse(r.poll())
        self.assertEqual(r.error,'read_deadline')
        self.assertAlmostEqual(t.snapshot()['events'][-1]['io']['read_window_s'],.011)
    def test_exact_read_limit_is_accepted(self):
        t=GapTimeline(True,wall=lambda:1,cpu=lambda:0)
        times=iter([1.,1.125])
        class Source:
            in_waiting=0
        r=Receiver(Source(),Mock(spec=Session),lambda:next(times),ReceiveLimits(max_read_s=.125),timeline=t)
        self.assertTrue(r.poll())
        self.assertIsNone(r.error)
        self.assertEqual(t.snapshot()['events'][-1]['io']['read_window_s'],.125)
if __name__=='__main__': unittest.main()
