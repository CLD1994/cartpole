import unittest
from step_loss_events import StepLossProbe, StepLossEventMixin
from session import Session, Phase, Limits

class ProbeTests(unittest.TestCase):
    def test_baseline_duplicates_and_first_active_only(self):
        p=StepLossProbe(); p.expected_run_id=1
        p.receive('EVENT STEP_LOSS 1 1 100',1,False)
        p.receive('EVENT STEP_LOSS 1 1 100',2,True)
        self.assertIsNone(p.trigger)
        p.receive('EVENT STEP_LOSS 2 1 101',3,True)
        p.receive('EVENT STEP_LOSS 3 1 102',4,True)
        self.assertEqual(p.trigger['event_id'],2)
        self.assertEqual(p.trigger['host_received_clock_s'],3)
    def test_real_transport_run_query(self):
        from run_policy import ControlTransport
        class Port:
            def write(self, data): self.data=data; return len(data)
        port=Port(); t=ControlTransport(port)
        self.assertEqual(t.write(b'RUN?\n'),5)
        self.assertEqual(port.data,b'RUN?\n')
        with self.assertRaises(ValueError): t.write(b'ENABLE\n')

    def test_monitor_keeps_model_and_freezes_window(self):
        p=StepLossProbe(); p.expected_run_id=1; p.monitor_only=True; p.wait_started_at=0
        for n in range(500): p.record_sample({'n':n})
        p.receive('EVENT STEP_LOSS 1 1 100',1000,True)
        for n in range(3000): p.record_sample({'n':500+n})
        self.assertEqual(len(p.event_window),2400)
        self.assertEqual(p.event_window[0]['n'],100)
        self.assertEqual(p.event_window[-1]['n'],2499)
        p.receive('EVENT STEP_LOSS 2 1 101',1001,True)
        self.assertEqual(len(p.event_window),2400)
        self.assertEqual(p.trigger['event_id'],1)

    def test_invalid_event_formats(self):
        for line in ('EVENT STEP_LOSS 0 1 0','EVENT STEP_LOSS 1 0 0',
                     'EVENT STEP_LOSS 1 1 -1','EVENT STEP_LOSS 1 1 0 extra',
                     'EVENT X 1 1 0','EVENT STEP_LOSS +1 1 0'):
            with self.subTest(line=line), self.assertRaises(ValueError):
                StepLossProbe().receive(line,0,True)
    def test_older_device_time_is_not_new_trigger(self):
        p=StepLossProbe(); p.expected_run_id=1; p.receive('EVENT STEP_LOSS 1 1 100',0,False)
        p.receive('EVENT STEP_LOSS 2 2 99',1,True)
        self.assertIsNone(p.trigger)

class ProbeSession(StepLossEventMixin,Session): pass
class SessionEventTests(unittest.TestCase):
    def make_session(self):
        class Port:
            def write(self,b): return len(b)
        return ProbeSession(Port(),lambda:10,Limits(400,1250,9000,10000))
    def test_run_handshake_and_old_event_rejection(self):
        s=self.make_session(); s.run_query_pending=True
        s.receive('OK RUN 7')
        self.assertEqual(s.step_loss_probe.expected_run_id,7)
        self.assertEqual(s.run_reply,7)
        s.step_loss_probe.receive('EVENT STEP_LOSS 1 6 100',10,True)
        self.assertIsNone(s.step_loss_probe.trigger)

    def test_startup_record_and_non_event_delegation(self):
        s=self.make_session(); s.receive('MODE READY NONE')
        s.receive('EVENT STEP_LOSS 1 1 10')
        self.assertEqual(s.mode,('READY','NONE'))
        self.assertIsNone(s.step_loss_probe.trigger)
    def active_session(self):
        s=self.make_session()
        s.receive('MODE READY NONE')
        for key,value in {'CAL_STATE':2,'CAL_SAVED':1,'CAL_STORAGE':0,
                          'CAL_LENGTH_UM':853940,'VEL_MAX':1250,'ACC_MAX':9000,
                          'WDT_MS':100,'WDT_DEC':5000,'X_MARGIN_UM':10000}.items():
            s.receive(f'OK {key} {value}')
        s.receive('VEL_VALID 1 1'); s.receive('STATE 0 0 0 0'); s.start()
        return s
    def test_active_event_keeps_session_and_selects_low_speed(self):
        s=self.active_session(); s.step_loss_probe.expected_run_id=4; s.receive('EVENT STEP_LOSS 1 4 999')
        self.assertEqual(s.phase,Phase.ACTIVE)
        self.assertEqual(s.step_loss_probe.trigger['run_id'],4)
        self.assertEqual(s.command(.008),10)
    def test_fault_still_stops_after_event(self):
        s=self.active_session(); s.step_loss_probe.expected_run_id=4; s.receive('EVENT STEP_LOSS 1 4 999')
        s.receive('MODE FAULT POWER')
        self.assertEqual(s.failure,'firmware_mode')
        with self.assertRaises(RuntimeError): s.command(.008)
    def test_active_malformed_event_latches_failure(self):
        s=self.active_session()
        with self.assertRaises(RuntimeError): s.receive('EVENT STEP_LOSS 1 4')
        self.assertEqual(s.failure,'malformed_step_loss_event')
    def test_malformed_event_fails_session(self):
        s=self.make_session()
        with self.assertRaisesRegex(RuntimeError,'malformed_step_loss_event'):
            s.receive('EVENT STEP_LOSS broken')

if __name__=='__main__': unittest.main()

