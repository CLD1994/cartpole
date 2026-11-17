import unittest
from run_policy import BoundSession
from session import Limits, Phase
from test_session import Clock, Port

class PoleRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.port = Clock(), Port()
        self.s = BoundSession(self.port,self.clock,Limits(400,900,10000,10000),{})
        for k,v in {'CAL_STATE':2,'CAL_SAVED':1,'CAL_STORAGE':0,'CAL_LENGTH_UM':920531,'VEL_MAX':900,'ACC_MAX':10000,'WDT_MS':100,'X_MARGIN_UM':10000,'POLE_ZERO_URAD':0}.items():
            self.s.receive(f'OK {k} {v}')
        self.s.receive('MODE READY NONE');self.s.receive('STATE 3 0 100 0 20');self.s.start()

    def test_invalid_pole_waits_without_reusing_observation(self):
        self.s.receive('STATE 1 0 0 0 0')
        self.assertEqual(self.s.phase,Phase.ACTIVE)
        self.assertIsNone(self.s.failure)
        self.assertIsNone(self.s.observation)
        self.assertTrue(self.s.pole_waiting)
        self.assertEqual(self.port.sent,[])

    def test_pending_recovery_cannot_send_command(self):
        self.s.receive('STATE 1 0 0 0 0')
        with self.assertRaisesRegex(RuntimeError,'pole_recovery_pending'):self.s.command(.1)
        self.assertEqual(self.port.sent,[])

    def test_invalid_stream_longer_than_command_budget_does_not_end_session(self):
        for _ in range(30):
            self.clock.time+=.002;self.s.receive('STATE 1 0 0 0 0')
        self.s.tick()
        self.assertEqual(self.s.phase,Phase.ACTIVE)
        self.assertIsNone(self.s.failure)

    def test_silence_during_recovery_still_stops(self):
        self.s.receive('STATE 1 0 0 0 0');self.clock.time+=.101;self.s.tick()
        self.assertEqual(self.s.failure,'measurement_timeout')

    def test_fault_during_recovery_still_stops(self):
        self.s.receive('STATE 1 0 0 0 0');self.s.receive('MODE FAULT MT6835')
        self.assertEqual(self.s.failure,'firmware_mode')

    def test_invalid_cart_still_stops(self):
        self.s.receive('STATE 2 0 0 0 0')
        self.assertEqual(self.s.failure,'measurement_invalid')

    def test_recovery_requires_explicit_new_fresh_valid_observation(self):
        self.s.receive('STATE 1 0 0 0 0');self.clock.time+=.002
        self.s.receive('STATE 3 5 123 1 22')
        self.assertTrue(self.s.pole_waiting)
        self.assertEqual(self.s.observation.theta_urad,123)
        self.s.finish_pole_recovery(.004)
        self.assertFalse(self.s.pole_waiting)
        self.assertEqual(self.s.command(.1),90)

    def test_stale_recovery_not_permitted(self):
        self.s.receive('STATE 1 0 0 0 0');self.clock.time+=.002
        self.s.receive('STATE 3 0 0 0 0');self.clock.time+=.005
        self.assertFalse(self.s.finish_pole_recovery(.004))
        self.assertTrue(self.s.pole_waiting)
