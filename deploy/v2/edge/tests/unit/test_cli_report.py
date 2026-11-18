"""报告读取接口的纯内存契约测试，不运行 CLI 或串口。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).parents[2]))
from cli_report import (parse_state_window_summary, parse_state_output, ModeEvent, StepLossEvent,
                        ERROR_INVALID_STATE_WINDOW_REPORT)

REPORT = ('[edge] window 1.0 s: 500 STATE lines -> 500.0 Hz (observed)\n'
          '[edge] unknown lines: 0 (require 0)\n'
          '[edge] sampled x range [-123, 456] um\n'
          '[edge] sampled theta range [-42, 13] urad\n')


class StateReportTests(unittest.TestCase):
    def test_complete_report_extracts_values_and_units(self):
        report = parse_state_window_summary(REPORT)
        self.assertEqual(report.elapsed_s, 1.0)
        self.assertEqual(report.received_state_count, 500)
        self.assertIsInstance(report.received_state_count, int)
        self.assertEqual(report.observed_hz, 500.0)
        self.assertEqual(report.unknown_line_count, 0)
        self.assertEqual(report.cart_range_um, (-123, 456))
        self.assertEqual(report.pole_range_urad, (-42, 13))
        self.assertEqual(parse_state_window_summary(REPORT.replace('\n', '\r\n')), report)
        self.assertEqual(parse_state_window_summary(REPORT.rstrip('\n')), report)

    def test_unavailable_lanes_are_not_zero_measurements(self):
        text = REPORT.replace('[edge] sampled x range [-123, 456] um', '[edge] cart samples unavailable')
        text = text.replace('[edge] sampled theta range [-42, 13] urad', '[edge] pole samples unavailable')
        report = parse_state_window_summary(text)
        self.assertIsNone(report.cart_range_um)
        self.assertIsNone(report.pole_range_urad)

    def test_full_stdout_retains_only_defined_events_and_report(self):
        output = '[edge] MODE READY NONE\n[edge] STEP_LOSS event=12 run=34 device_us=5678\n' + REPORT
        parsed = parse_state_output(output)
        self.assertEqual(parsed.report, parse_state_window_summary(REPORT))
        self.assertEqual(parsed.events, (ModeEvent('READY', 'NONE'), StepLossEvent(12, 34, 5678)))
        self.assertEqual(parse_state_output(REPORT).events, ())

    def test_full_stdout_rejects_unknown_or_malformed_events(self):
        for event in ('[edge] MODE RUNNING NONE', '[edge] MODE READY OTHER',
                      '[edge] STEP_LOSS event=-1 run=2 device_us=3',
                      '[edge] STEP_LOSS event=1 run=2 device_us=9223372036854775808',
                      '[edge] STEP_LOSS event=1 run=2', '[edge] unrelated warning'):
            with self.subTest(event=event):
                with self.assertRaises(ValueError) as caught:
                    parse_state_output(event + '\n' + REPORT)
                self.assertEqual(str(caught.exception), ERROR_INVALID_STATE_WINDOW_REPORT)

    def test_invalid_reports_are_rejected(self):
        lines = REPORT.splitlines(keepends=True)
        bad = [None, '', REPORT.replace('\n', '\v'), REPORT + lines[0], ''.join(lines[:3]),
               ''.join([lines[0], lines[0], *lines[2:]]),
               ''.join([lines[1], lines[0], *lines[2:]]),
               REPORT.replace('500 STATE', '-1 STATE'), REPORT.replace('500 STATE', '1.5 STATE'),
               REPORT.replace('500.0 Hz', 'nan Hz'), REPORT.replace('1.0 s', 'inf s'),
               REPORT.replace('[-123, 456]', '[456, -123]'), REPORT.replace(' um', ' mm'),
               REPORT.replace('[-42, 13]', '[x, 13]'), REPORT + '\n', REPORT.replace('500.0 Hz', '9'*400+'.0 Hz')]
        for text in bad:
            with self.subTest(text=text):
                with self.assertRaises(ValueError) as caught:
                    parse_state_window_summary(text)
                self.assertEqual(str(caught.exception), ERROR_INVALID_STATE_WINDOW_REPORT)

if __name__ == '__main__': unittest.main()
