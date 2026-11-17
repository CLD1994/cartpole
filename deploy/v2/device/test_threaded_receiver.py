import unittest
import threading
from unittest.mock import Mock
from threaded_receiver import ThreadedReceiver, ReceiverPort
from run_policy import ControlTransport
from session import Session, Limits, Phase
from test_session import Port, Clock

class ThreadedTests(unittest.TestCase):
    def test_fifo_does_not_drop_invalid_before_valid(self):
        s=Mock();r=ThreadedReceiver(Mock(),s,lambda:10.)
        r.enqueue('STATE 1 1 0 2 0',9.99);r.enqueue('STATE 3 1 2 3 4',9.991)
        self.assertTrue(r.poll())
        self.assertEqual([c.args for c in s.receive_at.call_args_list],[('STATE 1 1 0 2 0',9.99),('STATE 3 1 2 3 4',9.991)])
    def test_lock_contention_is_nonblocking_and_pending(self):
        r=ThreadedReceiver(Mock(),Mock(),lambda:10.);r.lock.acquire()
        try:self.assertTrue(r.poll());self.assertTrue(r.partial)
        finally:r.lock.release()
    def test_queue_overflow_is_explicit_failure(self):
        s=Mock();r=ThreadedReceiver(Mock(),s,lambda:10.,capacity=1)
        r.enqueue('MODE READY NONE',9.);r.enqueue('MODE FAULT MT6835',9.1)
        self.assertFalse(r.poll());self.assertEqual(r.error,'receive_queue_overflow');s.abort.assert_called_once()
    def test_transport_receive_properties_do_not_touch_port(self):
        port=Mock();r=ThreadedReceiver(port,Mock(),lambda:10.);t=ControlTransport(port);t.reader=r
        r.enqueue('STATE 3 1 2 3 4',9.)
        self.assertGreater(t.in_waiting,0);self.assertFalse(t.partial);port.read.assert_not_called()
    def test_received_state_keeps_original_host_time(self):
        clock=Clock();s=Session(Port(),clock,Limits(500,1250,10000,10000))
        s.receive_at('STATE 3 1 2 3 4',9.995)
        self.assertEqual(s.observation.received_at,9.995)
        self.assertEqual(s._last_time,10.)
    def test_old_disabled_reply_cannot_confirm_current_stop(self):
        clock=Clock();s=Session(Port(),clock,Limits(500,1250,10000,10000))
        s.phase=Phase.STOPPING;s._stop_at=9.9;s._last_stop_tx=10.;s._disable_sent=True;s._disable_at=9.999
        s.receive_at('MODE DISABLED NONE',9.998)
        self.assertFalse(s.disabled_confirmed)
        s.receive_at('MODE DISABLED NONE',10.)
        self.assertTrue(s.disabled_confirmed)
    def test_unfinished_reader_prevents_port_close(self):
        port=Mock();connection=ReceiverPort(port);connection.reader=Mock();connection.reader.stop_and_join.return_value=False
        with self.assertRaisesRegex(RuntimeError,'receiver_stop_unconfirmed'):connection.__exit__(None,None,None)
        port.close.assert_not_called()
    def test_invalid_then_valid_remains_failed_with_processing_stop_time(self):
        clock=Clock();s=Session(Port(),clock,Limits(500,1250,10000,10000))
        s.receive('MODE READY NONE')
        for key,value in {'CAL_STATE':2,'CAL_SAVED':1,'CAL_STORAGE':0,'CAL_LENGTH_UM':853325,'VEL_MAX':1250,'ACC_MAX':10000,'WDT_MS':100,'X_MARGIN_UM':10000}.items():s.receive(f'OK {key} {value}')
        s.receive('STATE 3 0 0 0 0');s.start();clock.time=10.01
        r=ThreadedReceiver(Mock(),s,clock);r.enqueue('STATE 1 1 0 2 0',10.005);r.enqueue('STATE 3 1 2 3 4',10.006)
        r.poll();self.assertEqual(s.failure,'measurement_invalid');self.assertEqual(s._stop_at,10.01)
    def test_old_frame_is_not_retimestamped_as_fresh(self):
        clock=Clock();s=Session(Port(),clock,Limits(500,1250,10000,10000))
        s.receive_at('STATE 3 0 0 0 0',9.99)
        self.assertGreaterEqual(clock()-s.observation.received_at,.004)
    def test_metadata_receipt_time_not_processing_time(self):
        clock=Clock();s=Session(Port(),clock,Limits(500,1250,10000,10000))
        s.receive_at('MODE READY NONE',9.99);s.receive_at('OK VEL_MAX 1250',9.991)
        self.assertEqual(s._mode_at,9.99);self.assertEqual(s.values['VEL_MAX'][1],9.991)
    def test_reader_done_controls_join_interrupt_result(self):
        r=ThreadedReceiver(Mock(),Mock(),lambda:10.);r.worker=Mock();r.worker.join.side_effect=KeyboardInterrupt
        self.assertFalse(r.stop_and_join());r.done.set();self.assertTrue(r.stop_and_join())
    def test_active_backend_backlog_is_latched_after_read(self):
        class Source:
            def __init__(self):self.data=b'A'*129+b'\n'
            @property
            def in_waiting(self):return len(self.data)
            def read(self,count):data=self.data;self.data=b'';return data
        r=ThreadedReceiver(Source(),Mock(),lambda:10.);r.active=True
        r.raw.poll();r.poll()
        self.assertEqual(r.source_pending,0);self.assertEqual(r.in_waiting,0)
        self.assertGreater(r.active_backlog_seen,128);self.assertTrue(r.backlog_pending)
    def test_inflight_poll_prevents_clean_receive_view(self):
        r=ThreadedReceiver(Mock(),Mock(),lambda:10.);r._polling=True
        self.assertTrue(r.partial)
    def test_queued_old_run_reply_does_not_answer_new_query(self):
        from step_loss_events import StepLossEventMixin
        class S(StepLossEventMixin,Session):pass
        clock=Clock();s=S(Port(),clock,Limits(500,1250,10000,10000))
        s.run_query_pending=True;s.run_query_sent_at=9.995
        s.receive_at('OK RUN 7',9.99)
        self.assertIsNone(s.run_reply);self.assertTrue(s.run_query_pending)
        s.receive_at('OK RUN 7',10.)
        self.assertEqual(s.run_reply,7);self.assertFalse(s.run_query_pending)
    def test_real_worker_stall_does_not_block_control_or_refresh_old_state(self):
        for delay in (.008,.012):
            entered=threading.Event();release=threading.Event();clock=Clock()
            class Source:
                def __init__(self):self.data=b'STATE 3 1 2 3 4\n'
                @property
                def in_waiting(self):return len(self.data)
                def read(self,count):
                    entered.set();release.wait(1)
                    data=self.data;self.data=b'';return data
            s=Session(Port(),clock,Limits(500,1250,10000,10000))
            s.receive_at('STATE 3 0 0 0 0',9.999)
            r=ThreadedReceiver(Source(),s,clock);r.start()
            try:
                self.assertTrue(entered.wait(1));clock.time=10.+delay
                self.assertTrue(r.poll());self.assertTrue(r.partial)
                self.assertEqual(s.observation.received_at,9.999)
                self.assertGreater(clock()-s.observation.received_at,.004)
            finally:
                release.set();self.assertTrue(r.stop_and_join())
            if delay>.01:
                self.assertIsNone(r.error);self.assertTrue(r.poll())
                self.assertTrue(r.backlog_pending)
                self.assertFalse(r.backlog_ready(s.observation))
                self.assertIsNone(s.failure)
            else:
                self.assertTrue(r.poll());self.assertEqual(s.observation.received_at,10.+delay)
    def test_partial_snapshot_is_published_before_busy_clears(self):
        r=ThreadedReceiver(Mock(),Mock(),lambda:10.);observed=[]
        class Pending:
            def __bool__(self):observed.append(r._polling);return True
        class Raw:
            _pending=Pending()
            def poll(self):r.stop_event.set();return True
        r.raw=Raw();r._run()
        self.assertEqual(observed,[True]);self.assertTrue(r.partial);self.assertFalse(r._polling)
if __name__=='__main__':unittest.main()
