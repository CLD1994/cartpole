"""真实 CLI 与串口的行程测试；PTY 不连接实机。"""

import os
from pathlib import Path
import shutil
import subprocess
import sys
from threading import Event
import unittest

import test_cli


class MovingFirmware(test_cli.Firmware):
    def __init__(self, flags=3):
        super().__init__(flags=flags)
        self.zero_received = Event()

    def send(self, line):
        if line.startswith("STATE ") and self.driving.is_set():
            # 一收到非零命令就报告进入制动区；收到零速后继续输出静止帧。
            if "VEL 0" in self.commands:
                self.zero_received.set()
            velocity = 0 if self.zero_received.is_set() else 600
            line = f"STATE {self.flags} 410000 0 {velocity} 0"
        super().send(line)


class TravelCommandTests(unittest.TestCase):
    def test_planning_is_read_only_even_in_disabled_mode(self):
        with test_cli.Firmware(mode="DISABLED") as fw:
            result = fw.run("vel", "600", "--plan")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(fw.commands, ["CALIB?", "CONFIG?", "QUERY"])
        self.assertIn("417.422 mm", result.stdout)

    def test_position_triggers_zero_without_disabling_driver(self):
        with MovingFirmware() as fw:
            result = fw.run("vel", "600", "--max-seconds", "1")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        nonzero = fw.commands.index("VEL 600")
        zero = fw.commands.index("VEL 0")
        self.assertEqual(nonzero + 1, zero)
        self.assertNotIn("ENABLE", fw.commands)
        self.assertNotIn("DISABLE", fw.commands)
        self.assertIn("410.000 mm", result.stdout)
        self.assertIn("收到停稳证据", result.stdout)

    def test_travel_uses_cart_measurement_without_pole_measurement(self):
        with MovingFirmware(flags=1) as fw:
            result = fw.run("vel", "600", "--max-seconds", "1")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("VEL 600", fw.commands)
        self.assertIn("VEL 0", fw.commands)

    @unittest.skipUnless(shutil.which("stdbuf"), "需要 stdbuf 禁用子进程的 stdout 缓冲")
    def test_full_output_pipe_does_not_delay_zero_request(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, "rb") as reader, os.fdopen(write_fd, "wb") as writer:
            os.set_blocking(write_fd, False)
            filled = 0
            try:
                while True:
                    filled += os.write(write_fd, b"x" * 4096)
            except BlockingIOError:
                pass
            os.set_blocking(write_fd, True)
            self.assertGreater(filled, 0)
            with MovingFirmware() as fw:
                args = ["stdbuf", "-o0", test_cli.BINARY, "--port", fw.path,
                        "vel", "600", "--max-seconds", "1"]
                child = subprocess.Popen(args, stdout=writer, stderr=subprocess.PIPE)
                try:
                    self.assertTrue(fw.zero_received.wait(2), fw.commands)
                    writer.close()
                    self.assertEqual(reader.read(filled), b"x" * filled)
                    _, stderr = child.communicate(timeout=4)
                    self.assertEqual(child.returncode, 0, stderr.decode())
                    self.assertIn("收到停稳证据".encode(), reader.read())
                finally:
                    if child.poll() is None:
                        child.kill()
                    child.communicate(timeout=4)

    def test_legacy_watchdog_parameter_rejects_plan(self):
        with test_cli.Firmware() as fw:
            fw.settings["WDT_DEC"] = 1000
            result = fw.run("vel", "600", "--plan")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(fw.commands, ["CALIB?", "CONFIG?", "QUERY"])


if __name__ == "__main__":
    test_cli.BINARY = str(Path(sys.argv.pop(1)).resolve())
    unittest.main()
