import unittest
from gap_timeline import GapTimeline

class TimelineTests(unittest.TestCase):
    def test_disabled_does_not_read_clocks(self):
        def fail(): raise AssertionError('clock touched')
        t=GapTimeline(False, wall=fail, cpu=fail)
        t.mark('poll'); self.assertIsNone(t.snapshot())
    def test_bounds_tail_and_preserves_intervals_and_cycle(self):
        w=iter([100,110,150]); c=iter([10,12,15])
        t=GapTimeline(True, capacity=2, wall=lambda:next(w), cpu=lambda:next(c))
        t.mark('write_done', 7); t.mark('sleep',7,.0002); t.mark('poll',8)
        s=t.snapshot(); self.assertEqual(s['total_marks'],3)
        self.assertEqual(len(s['events']),2)
        self.assertEqual(s['events'][1], {'stage':'poll','cycle':8,'wall_ns':150,'thread_cpu_ns':15,'wall_delta_ns':40,'thread_cpu_delta_ns':3,'requested_sleep_s':None})
        self.assertEqual(s['events'][0]['requested_sleep_s'],.0002)
    def test_freeze_prevents_stop_overwriting_failure(self):
        t=GapTimeline(True,wall=lambda:1,cpu=lambda:2)
        t.mark('failed',1); t.freeze(); t.mark('stop',1)
        self.assertEqual([x['stage'] for x in t.snapshot()['events']],['failed'])
if __name__=='__main__': unittest.main()
