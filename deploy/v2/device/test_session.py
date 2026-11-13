import unittest

from session import Limits, Phase, Session
from check import limits_from_profile


class Port:
    def __init__(self):
        self.sent = []
        self.fail = False
    def write(self, data):
        if self.fail:
            raise OSError('unplugged')
        self.sent.append(data)
        return len(data)


class Clock:
    def __init__(self):
        self.time = 10.
    def __call__(self):
        return self.time


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.port, self.clock = Port(), Clock()
        self.limits = Limits(500, 1250, 10000, 10000)
        self.s = Session(self.port, self.clock, self.limits)

    def ready(self):
        self.s.receive('MODE READY NONE')
        for key, value in {'CAL_STATE': 2, 'CAL_SAVED': 1, 'CAL_STORAGE': 0,
                           'CAL_LENGTH_UM': 853325, 'VEL_MAX': 1250, 'ACC_MAX': 10000,
                           'WDT_MS': 100, 'WDT_DEC': 5000, 'X_MARGIN_UM': 10000}.items():
            self.s.receive(f'OK {key} {value}')
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')

    def test_observe_only_queries(self):
        self.s.observe()
        self.assertEqual(self.port.sent, [b'QUERY\n', b'CALIB?\n', b'CONFIG?\n'])

    def test_unknown_validity_rejects_start(self):
        self.ready()
        self.s.observe()
        with self.assertRaises(RuntimeError): self.s.start()

    def test_left_reference_required(self):
        self.ready()
        self.s.receive('OK CAL_STATE 3')
        with self.assertRaisesRegex(RuntimeError, 'CAL_STATE'): self.s.start()

    def test_config_mismatch_blocks(self):
        self.ready()
        self.s.receive('OK ACC_MAX 5000')
        with self.assertRaisesRegex(RuntimeError, 'ACC_MAX'): self.s.start()

    def test_stale_preflight_blocks(self):
        self.ready()
        self.clock.time += 1.1
        with self.assertRaises(RuntimeError): self.s.start()

    def test_start_has_no_external_action(self):
        self.ready()
        self.s.start()
        self.assertEqual(self.port.sent, [])

    def test_maps_explicit_action(self):
        self.ready()
        self.s.start()
        self.assertEqual(self.s.command(.8), 1000)
        self.assertEqual(self.port.sent, [b'VEL 1000\n'])

    def test_invalidity_stops_and_latches(self):
        self.ready()
        self.s.start()
        self.s.receive('VEL_VALID 0 1')
        self.assertIsNone(self.s.observation)
        self.assertEqual(self.port.sent, [b'VEL 0\n'])
        self.ready()
        with self.assertRaises(RuntimeError): self.s.command(1)

    def test_warmup_may_recover_before_start(self):
        self.s.receive('VEL_VALID 0 0')
        self.ready()
        self.s.start()
        self.assertEqual(self.s.phase, Phase.ACTIVE)

    def test_validity_restore_needs_new_state(self):
        self.ready()
        self.s.receive('VEL_VALID 0 1')
        self.s.receive('VEL_VALID 1 1')
        self.assertIsNone(self.s.observation)

    def test_state_timeout_stops(self):
        self.ready()
        self.s.start()
        self.clock.time += .101
        self.s.tick()
        self.assertEqual(self.s.failure, 'measurement_timeout')

    def test_heartbeat_timeout_even_with_new_state(self):
        self.ready()
        self.s.start()
        self.clock.time += .301
        self.s.receive('STATE 0 0 0 0')
        self.s.tick()
        self.assertEqual(self.s.failure, 'measurement_timeout')

    def test_fault_mode_stops(self):
        self.ready()
        self.s.start()
        self.s.receive('MODE FAULT STEP_LOSS')
        self.assertEqual(self.s.failure, 'firmware_mode')

    def test_malformed_state_invalidates(self):
        self.ready()
        self.s.start()
        self.s.receive('STATE 1 MODE READY NONE')
        self.assertEqual(self.s.failure, 'malformed_STATE')
        self.assertIsNone(self.s.observation)

    def test_nan_action_stops(self):
        self.ready()
        self.s.start()
        with self.assertRaises(ValueError): self.s.command(float('nan'))
        self.assertEqual(self.port.sent, [b'VEL 0\n'])

    def test_transport_failure_latches(self):
        self.ready()
        self.s.start()
        self.port.fail = True
        with self.assertRaises(RuntimeError): self.s.command(1)
        self.assertEqual(self.s.failure, 'transport_write')

    def test_stop_needs_new_state_then_disable_ack(self):
        self.ready()
        self.s.start()
        self.s.stop()
        self.clock.time += .04
        self.s.tick()
        self.assertNotIn(b'DISABLE\n', self.port.sent)
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 100 0 0 0')
        self.clock.time += .04
        self.s.tick()
        self.assertIn(b'DISABLE\n', self.port.sent)
        self.assertFalse(self.s.disabled_confirmed)
        self.s.receive('MODE DISABLED NONE')
        self.assertTrue(self.s.disabled_confirmed)

    def test_stop_deadline_not_extended_by_busy(self):
        self.ready()
        self.s.start()
        self.s.stop()
        self.s.receive('ERR BUSY')
        self.clock.time += 1.1
        self.s.tick()
        self.assertEqual(self.s.failure, 'stop_unconfirmed')
        self.assertFalse(self.s.disabled_confirmed)

    def test_outward_soft_boundary_blocked(self):
        self.ready()
        self.s.start()
        self.s.receive('STATE 417000 0 0 0')
        self.assertEqual(self.s.command(1), 0)
        self.assertEqual(self.s.command(-1), -1250)

    def test_observation_si_keeps_firmware_frame(self):
        self.ready()
        self.s.receive('STATE 100000 1000000 200 -300')
        self.assertEqual(self.s.observation.si(), (.1, 1., .2, -.3))

    def test_clock_regression_stops(self):
        self.ready()
        self.s.start()
        self.clock.time -= 1
        with self.assertRaises(RuntimeError): self.s.tick()
        self.assertEqual(self.s.failure, 'clock_invalid')

    def test_changed_length_stops_active_session(self):
        self.ready()
        self.s.start()
        self.s.receive('OK CAL_LENGTH_UM 700000')
        self.assertEqual(self.s.failure, 'readback_changed')

    def test_stale_observation_cache_is_cleared(self):
        self.ready()
        self.clock.time += .101
        self.s.tick()
        self.assertIsNone(self.s.observation)

    def test_fault_recovery_can_disable_but_not_resume(self):
        self.ready()
        self.s.start()
        self.s.receive('VEL_VALID 0 1')
        self.clock.time += .04
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        self.clock.time += .04
        self.s.tick()
        self.s.receive('MODE DISABLED NONE')
        self.assertTrue(self.s.disabled_confirmed)
        self.assertEqual(self.s.failure, 'measurement_invalid')
        with self.assertRaises(RuntimeError): self.s.start()

    def test_metadata_expires_even_with_fresh_measurement(self):
        self.ready()
        self.s.start()
        self.clock.time += 1.01
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        self.s.tick()
        self.assertEqual(self.s.failure, 'metadata_unavailable')

    def test_refresh_does_not_send_configuration_or_clear_state(self):
        self.ready()
        self.s.start()
        before = self.s.observation
        self.s.refresh()
        self.assertEqual(self.s.observation, before)
        self.assertEqual(self.port.sent, [b'QUERY\n', b'CALIB?\n', b'CONFIG?\n'])

    def test_observer_clock_failure_never_sends_velocity(self):
        self.ready()
        self.clock.time -= 1
        with self.assertRaises(RuntimeError): self.s.tick()
        self.assertEqual(self.port.sent, [])

    def test_late_bytes_do_not_hide_expired_state(self):
        self.ready()
        self.s.start()
        self.clock.time += .4
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        with self.assertRaises(RuntimeError): self.s.command(1)
        self.assertEqual(self.s.failure, 'measurement_timeout')

    def test_late_disable_ack_does_not_pass_stop_deadline(self):
        self.ready()
        self.s.start()
        self.s.stop()
        self.clock.time += .04
        self.s.receive('STATE 0 0 0 0')
        self.clock.time += .04
        self.s.tick()
        self.assertIn(b'DISABLE\n', self.port.sent)
        self.clock.time += 1.1
        self.s.receive('MODE DISABLED NONE')
        self.assertFalse(self.s.disabled_confirmed)
        self.assertEqual(self.s.failure, 'stop_unconfirmed')

    def test_wrong_action_type_stops(self):
        self.ready()
        self.s.start()
        with self.assertRaises(ValueError): self.s.command(None)
        self.assertEqual(self.s.failure, 'invalid_action')
        self.assertEqual(self.s.phase, Phase.STOPPING)

    def test_nonscalar_action_stops(self):
        self.ready()
        self.s.start()
        with self.assertRaises(ValueError): self.s.command([1])
        self.assertEqual(self.s.failure, 'invalid_action')

    def test_external_abort_stops_without_private_access(self):
        self.ready()
        self.s.start()
        self.s.abort()
        self.assertEqual(self.s.failure, 'application_abort')
        self.assertEqual(self.port.sent, [b'VEL 0\n'])

    def test_observer_abort_is_read_only(self):
        self.s.abort()
        self.assertEqual(self.port.sent, [])
        self.assertEqual(self.s.phase, Phase.FINISHED)

    def test_fault_during_stopping_survives_disabled_ack(self):
        self.ready()
        self.s.start()
        self.s.stop()
        self.s.receive('MODE FAULT STEP_LOSS')
        self.clock.time += .04
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        self.clock.time += .04
        self.s.tick()
        self.s.receive('MODE DISABLED NONE')
        self.assertTrue(self.s.disabled_confirmed)
        self.assertEqual(self.s.failure, 'firmware_mode')

    def test_error_during_stopping_survives_disabled_ack(self):
        self.ready()
        self.s.start()
        self.s.stop()
        self.s.receive('ERR DISABLE')
        self.clock.time += .04
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        self.clock.time += .04
        self.s.tick()
        self.s.receive('MODE DISABLED NONE')
        self.assertTrue(self.s.disabled_confirmed)
        self.assertEqual(self.s.failure, 'firmware_error')

    def test_missing_calibration_value_revokes_previous_readback(self):
        self.ready()
        self.s.receive('OK CAL_STATE')
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        with self.assertRaisesRegex(RuntimeError, 'CAL_STATE'): self.s.start()

    def test_missing_config_value_revokes_previous_readback(self):
        self.ready()
        self.s.receive('OK ACC_MAX')
        self.s.receive('VEL_VALID 1 1')
        self.s.receive('STATE 0 0 0 0')
        with self.assertRaisesRegex(RuntimeError, 'ACC_MAX'): self.s.start()

    def test_missing_active_config_value_latches_failure(self):
        self.ready()
        self.s.start()
        self.s.receive('OK ACC_MAX')
        self.assertEqual(self.s.phase, Phase.STOPPING)
        self.assertEqual(self.s.failure, 'malformed_OK')

    def test_profile_adapter_uses_injected_profile(self):
        profile = {'training': {'control_hz': 450}, 'configured': {
            'target_speed_mm_s': 1000, 'acceleration_mm_s2': 8000, 'soft_limit_margin_um': 12000}}
        self.assertEqual(limits_from_profile(profile).speed_mm_s, 1000)


if __name__ == '__main__':
    unittest.main()
