import unittest
from unittest.mock import Mock
from receiver import Receiver, ReceiveLimits
from session import Session


class Source:
    def __init__(self):
        self.data = b''
    @property
    def in_waiting(self):
        return len(self.data)
    def read(self, size):
        data, self.data = self.data[:size], self.data[size:]
        return data


class ReceiverTests(unittest.TestCase):
    def setUp(self):
        self.source = Source()
        self.session = Mock(spec=Session)
        self.now = 1.
        self.receiver = Receiver(self.source, self.session, lambda: self.now)

    def test_fragment_then_multiple_lines(self):
        self.source.data = b'STATE 1'
        self.assertTrue(self.receiver.poll())
        self.session.receive.assert_not_called()
        self.source.data = b' 2 3 4\r\nVEL_VALID 1 1\n'
        self.now += .001
        self.assertTrue(self.receiver.poll())
        self.assertEqual([c.args[0] for c in self.session.receive.call_args_list],
                         ['STATE 1 2 3 4', 'VEL_VALID 1 1'])

    def test_bad_batch_never_delivered(self):
        self.source.data = b'STATE 1 2 3 4\n\xff\n'
        self.assertFalse(self.receiver.poll())
        self.session.receive.assert_not_called()
        self.session.abort.assert_called_once()

    def test_backlog(self):
        self.source.data = b'x'*1025
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'backlog')

    def test_overlong_partial(self):
        self.source.data = b'x'*257
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'line_overflow')

    def test_partial_deadline(self):
        self.source.data = b'STATE '
        self.receiver.poll()
        for _ in range(3):
            self.now += .009
            self.receiver.poll()
        self.assertEqual(self.receiver.error, 'partial_timeout')

    def test_poll_deadline_before_consuming(self):
        self.receiver.poll()
        self.now += .03
        self.source.data = b'STATE 1 2 3 4\n'
        self.assertFalse(self.receiver.poll())
        self.session.receive.assert_not_called()
        self.assertTrue(self.source.data)

    def test_read_exception(self):
        self.source.data = b'x'
        self.source.read = Mock(side_effect=OSError())
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'read_error')

    def test_late_read_rejected(self):
        self.source.data = b'STATE 1 2 3 4\n'
        original = self.source.read
        def slow(n):
            self.now += .011
            return original(n)
        self.source.read = slow
        self.assertFalse(self.receiver.poll())
        self.session.receive.assert_not_called()

    def test_failure_resynchronizes_on_next_newline(self):
        self.source.data = b'\xff'
        self.receiver.poll()
        self.source.data = b'STATE 1 2 3 4\nMODE DISABLED NONE\n'
        self.assertTrue(self.receiver.poll())
        self.session.receive.assert_called_once_with('MODE DISABLED NONE')
        self.assertEqual(self.receiver.error, 'invalid_ascii')

    def test_empty_poll_ticks(self):
        self.assertTrue(self.receiver.poll())
        self.assertTrue(self.session.tick.called)
        self.session.abort.assert_not_called()

    def test_invalid_limits(self):
        with self.assertRaises(ValueError):
            ReceiveLimits(max_read_s=float('nan'))

    def test_limits_reject_wrong_types_and_unbounded_allocation(self):
        for field, value in [('max_pending', 1048577), ('max_pending', True),
                             ('max_line', 1.5), ('max_poll_s', True),
                             ('max_read_s', '0.01'), ('partial_timeout_s', 0)]:
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    ReceiveLimits(**{field: value})

    def test_short_read_preserves_unfinished_line(self):
        self.source.data = b'STATE 1 2 3 4\n'
        original = self.source.read
        self.source.read = lambda n: original(min(n, 7))
        self.assertTrue(self.receiver.poll())
        self.session.receive.assert_not_called()
        self.assertTrue(self.receiver.poll())
        self.session.receive.assert_called_once_with('STATE 1 2 3 4')

    def test_read_crosses_partial_deadline(self):
        self.source.data = b'STATE '
        self.receiver.poll()
        self.now += .019
        self.source.data = b'1 2 3 4\n'
        original = self.source.read
        def slow(n):
            self.now += .009
            return original(n)
        self.source.read = slow
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'partial_timeout')
        self.session.receive.assert_not_called()

    def test_backlog_appearing_during_read_rejects_batch(self):
        self.source.data = b'STATE 1 2 3 4\n'
        original = self.source.read
        def refill(n):
            data = original(n)
            self.source.data = b'x'*1025
            return data
        self.source.read = refill
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'backlog')
        self.session.receive.assert_not_called()

    def test_empty_read_with_pending_bytes_rejected(self):
        self.source.data = b'x'
        self.source.read = Mock(return_value=b'')
        self.assertFalse(self.receiver.poll())
        self.assertEqual(self.receiver.error, 'read_contract')

    def test_session_fatal_error_propagates_without_read(self):
        self.source.read = Mock()
        self.session.tick.side_effect = RuntimeError('clock_invalid')
        with self.assertRaisesRegex(RuntimeError, 'clock_invalid'):
            self.receiver.poll()
        self.source.read.assert_not_called()
