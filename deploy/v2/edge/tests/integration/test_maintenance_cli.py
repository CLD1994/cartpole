"""Exercise the maintenance CLI with a PTY rather than real hardware."""
import sys
import termios
import unittest
from pathlib import Path

import test_cli

if __name__ == "__main__":
    test_cli.BINARY = str(Path(sys.argv.pop(1)).resolve())

Firmware = test_cli.Firmware


class MaintenanceCommands(unittest.TestCase):
    def test_enable_ready_reports_success_without_sending_enable(self):
        with Firmware(mode="READY", acknowledge_enable=False) as fw:
            result = fw.run("enable", "--timeout", "0.3")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(fw.commands, ["QUERY"])
            self.assertIn("已处于 READY", result.stdout)

    def test_enable_fault_does_not_send_enable(self):
        with Firmware(mode="FAULT", acknowledge_enable=False) as fw:
            result = fw.run("enable", "--timeout", "0.3")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(fw.commands, ["QUERY"])
            self.assertIn("FAULT", result.stderr)
            self.assertNotIn("TIMEOUT", result.stderr)

    def test_enable_readiness_cannot_hide_a_fault_in_the_query_response(self):
        class FaultThenReady(Firmware):
            def send(self, line):
                if line == "MODE READY NONE":
                    # Deliver both device frames in one write to test the whole response batch.
                    return super().send("MODE FAULT MT6835\nMODE READY NONE")
                return super().send(line)

        with FaultThenReady(mode="READY") as fw:
            result = fw.run("enable", "--timeout", "0.3")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(fw.commands, ["QUERY"])
            self.assertIn("ENABLE: FAULT", result.stderr)
            self.assertNotIn("[OK]", result.stdout)

    def test_enable_disabled_requires_both_measurement_lanes(self):
        with Firmware(mode="DISABLED", flags=1) as fw:
            result = fw.run("enable", "--timeout", "0.3")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(fw.commands, ["QUERY"])
            self.assertIn("STATE", result.stderr)

    def test_enable_disabled_sends_once_after_preflight(self):
        with Firmware(mode="DISABLED") as fw:
            result = fw.run("enable", "--timeout", "0.3")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(fw.commands, ["QUERY", "ENABLE"])

    def test_query_emits_one_mode_line(self):
        with Firmware(mode="READY") as fw:
            result = fw.run("query")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.splitlines(), ["[MODE] READY NONE"])

    def test_enable_unconfirmed_is_not_automatically_replayed(self):
        with Firmware(mode="DISABLED", acknowledge_enable=False) as fw:
            result = fw.run("enable", "--timeout", "0.2")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(fw.commands, ["QUERY", "ENABLE"])
            self.assertIn("执行未确认，不要自动重试", result.stderr)

    def test_mark_reports_typed_values_from_the_matched_reply(self):
        with Firmware(mode="DISABLED") as fw:
            left = fw.run("mark-left")
            right = fw.run("mark-right")
            self.assertEqual(left.returncode, 0, left.stderr)
            self.assertEqual(right.returncode, 0, right.stderr)
            self.assertEqual(fw.commands, ["MARK LEFT", "MARK RIGHT"])
            self.assertIn("[OK] MARK_LEFT = -447422 um", left.stdout)
            self.assertIn("[OK] MARK_RIGHT = 894845 um", right.stdout)

    def test_dump_diagnoses_missing_setting(self):
        with Firmware(missing_setting="STEP_LOSS_UM") as fw:
            result = fw.run("dump")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(fw.commands, ["CONFIG?"])
            self.assertIn("STEP_LOSS_UM", result.stderr)

    def test_open_explicitly_clears_stop_bits_and_software_flow_control(self):
        with Firmware() as fw:
            attrs = termios.tcgetattr(fw.slave)
            attrs[0] |= termios.IXOFF | termios.IXANY
            attrs[2] |= termios.CSTOPB
            termios.tcsetattr(fw.slave, termios.TCSANOW, attrs)
            result = fw.run("query")
            self.assertEqual(result.returncode, 0, result.stderr)
            attrs = termios.tcgetattr(fw.slave)
            self.assertEqual(attrs[0] & (termios.IXOFF | termios.IXANY), 0)
            self.assertEqual(attrs[2] & termios.CSTOPB, 0)


if __name__ == "__main__":
    unittest.main()
