import contextlib
import io
import json
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import check
from receiver import Receiver
from session import Session


class CheckTests(unittest.TestCase):
    def run_check(self, receive_error=None, preflight_reason=None, failure=None):
        session = Mock(spec=Session)
        session.preflight_reason.return_value = preflight_reason
        session.mode = ('READY', 'NONE')
        session.observation = None
        session.values = {}
        session.failure = failure
        receiver = Mock(spec=Receiver)
        receiver.poll.return_value = receive_error is None
        receiver.error = receive_error
        port = Mock(spec=['reset_input_buffer', 'write', 'read', 'in_waiting'])
        serial = Mock()
        serial.Serial.return_value.__enter__ = Mock(return_value=port)
        serial.Serial.return_value.__exit__ = Mock(return_value=False)
        profile = {'configured': {'target_speed_mm_s': 1250, 'acceleration_mm_s2': 10000,
                                  'soft_limit_margin_um': 10000},
                   'training': {'control_hz': 500}, 'measured': {'protocol_baud': 460800}}
        profile_path = Mock(spec=Path)
        profile_path.read_text.return_value = json.dumps(profile)
        output = io.StringIO()
        with patch.object(check, 'PROFILE', profile_path), \
             patch.object(check, 'Session', return_value=session), \
             patch.object(check, 'Receiver', return_value=receiver), \
             patch.object(check.time, 'monotonic', side_effect=[0, 0, 0, .002, .002]), \
             patch.object(check.time, 'sleep'), \
             patch.dict('sys.modules', {'serial': serial}), \
             patch('sys.argv', ['check', '--port', 'TEST', '--seconds', '.001']), \
             contextlib.redirect_stdout(output):
            code = check.main()
        return code, json.loads(output.getvalue())

    def test_receive_failure_returns_nonzero(self):
        code, report = self.run_check(receive_error='backlog')
        self.assertEqual(code, 1)
        self.assertEqual(report['receive_error'], 'backlog')
        self.assertFalse(report['motion_sent'])

    def test_unready_preflight_returns_nonzero(self):
        code, report = self.run_check(preflight_reason='CAL_STATE')
        self.assertEqual(code, 1)
        self.assertEqual(report['preflight_reason'], 'CAL_STATE')

    def test_latched_session_failure_returns_nonzero(self):
        code, report = self.run_check(failure='measurement_invalid')
        self.assertEqual(code, 1)
        self.assertEqual(report['session_failure'], 'measurement_invalid')

    def test_ready_check_returns_zero(self):
        code, report = self.run_check()
        self.assertEqual(code, 0)
        self.assertIsNone(report['preflight_reason'])
