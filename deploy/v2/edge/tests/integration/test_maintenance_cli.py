"""Exercise the maintenance CLI with a PTY rather than real hardware."""
import os
import select
import subprocess
import sys
import time
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

    def test_query_waits_for_a_complete_startup_state_even_with_no_valid_sensor_lanes(self):
        fw = Firmware(stream=False)
        try:
            with subprocess.Popen([test_cli.BINARY, "--port", fw.path, "query"],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as child:
                # Wait until the program has configured the PTY, then let its initial flush finish.
                deadline = time.monotonic() + 1
                while not termios.tcgetattr(fw.slave)[2] & termios.CLOCAL:
                    self.assertLess(time.monotonic(), deadline, "serial port not configured")
                    time.sleep(.001)
                self.assertFalse(select.select([fw.master], [], [], .003)[0],
                                 "QUERY preceded any startup framing evidence")
                # This plausible frame still lacked a known leading line boundary.
                os.write(fw.master, b"STATE 3 1 2 3 4\n")
                self.assertFalse(select.select([fw.master], [], [], .003)[0],
                                 "the ambiguous first line was accepted as startup sync")
                os.write(fw.master, b"STATE 0 0 0 0 0\n")
                self.assertTrue(select.select([fw.master], [], [], .2)[0])
                self.assertEqual(os.read(fw.master, 64), b"QUERY\n")
                os.write(fw.master, b"MODE DISABLED NONE\n")
                stdout, stderr = child.communicate(timeout=2)
                self.assertEqual(child.returncode, 0, stderr)
                self.assertEqual(stdout.splitlines(), ["[MODE] DISABLED NONE"])
        finally:
            os.close(fw.master)
            os.close(fw.slave)

    def test_query_fallback_drops_incomplete_startup_garbage(self):
        fw = Firmware(stream=False)
        try:
            with subprocess.Popen([test_cli.BINARY, "--port", fw.path, "query"],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as child:
                deadline = time.monotonic() + 1
                while not termios.tcgetattr(fw.slave)[2] & termios.CLOCAL:
                    self.assertLess(time.monotonic(), deadline, "serial port not configured")
                    time.sleep(.001)
                self.assertFalse(select.select([fw.master], [], [], .003)[0])
                # The initial hardware capture contained binary bytes and no LF before the reply.
                os.write(fw.master, bytes.fromhex("4c4c26d31352a9929aa2aaaa0282028252fe"))
                self.assertTrue(select.select([fw.master], [], [], .3)[0],
                                "missing STATE must not block QUERY")
                self.assertEqual(os.read(fw.master, 64), b"QUERY\n")
                os.write(fw.master, b"junkMODE READY NONE\nMODE DISABLED NONE\n")
                stdout, stderr = child.communicate(timeout=2)
                self.assertEqual(child.returncode, 0, stderr)
                self.assertEqual(stdout.splitlines(), ["[MODE] DISABLED NONE"])
        finally:
            os.close(fw.master)
            os.close(fw.slave)

    def test_queries_and_disable_still_get_a_reply_without_telemetry(self):
        for command, request in [("query", "QUERY"), ("disable", "DISABLE")]:
            with self.subTest(command=command), Firmware(stream=False) as fw:
                result = fw.run(command)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(fw.commands, [request])

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
