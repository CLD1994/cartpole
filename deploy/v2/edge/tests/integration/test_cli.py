"""真实 CLI/会话/串口/协议协作；PTY 代替固件，NPU 为链接级替身。

不打开实际设备。针对参数索引、MODE 回执、无效 lane 显示、预检写入边界和停机确认。
"""
import hashlib
import json
import os
from pathlib import Path
import pty
import queue
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

sys.path.insert(0, str(Path(__file__).parents[2]))
from cli_report import parse_state_output, ModeEvent, StepLossEvent

BINARY = os.environ.get("EDGE_TEST_BINARY")
if __name__ == "__main__":
    BINARY = str(Path(sys.argv.pop(1)).resolve())


class Firmware:
    def __init__(self, flags=3, stream=True, mode="READY", velocity=900, acknowledge_stop=True, acknowledge_enable=True):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self.slave)
        self.flags, self.stream, self.mode = flags, stream, mode
        self.velocity, self.acknowledge_stop = velocity, acknowledge_stop
        self.acknowledge_enable = acknowledge_enable
        self.commands = []
        self.end = threading.Event()
        self.driving = threading.Event()
        self.thread = threading.Thread(target=self.serve)
        self.error = None
        self.injections = queue.Queue()
        self.theta = 0

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.end.set()
        self.thread.join(2)
        if self.thread.is_alive():
            raise RuntimeError("PTY peer did not exit")
        os.close(self.master)
        os.close(self.slave)
        if self.error:
            raise self.error

    def send(self, line):
        data = (line + "\n").encode()
        while data and not self.end.is_set():
            if select.select([], [self.master], [], .01)[1]:
                try:
                    data = data[os.write(self.master, data):]
                except BlockingIOError:
                    pass

    def serve(self):
        pending = b""
        try:
            while not self.end.is_set():
                if select.select([self.master], [], [], .002)[0]:
                    pending += os.read(self.master, 4096)
                    while b"\n" in pending:
                        raw, pending = pending.split(b"\n", 1)
                        command = raw.decode().strip()
                        self.commands.append(command)
                        if command == "ENABLE":
                            if self.acknowledge_enable:
                                self.mode = "READY"
                                self.send("MODE READY NONE")
                        elif command == "QUERY":
                            self.send(f"MODE {self.mode} NONE")
                        elif command == "DISABLE":
                            if self.acknowledge_stop:
                                self.mode = "DISABLED"
                                self.send("MODE DISABLED NONE")
                        elif command == "CALIB?":
                            for key, value in [("CAL_STATE", 2), ("CAL_LENGTH_UM", 894845), ("CAL_SAVED", 1), ("CAL_STORAGE", 0)]:
                                self.send(f"OK {key} {value}")
                        elif command == "CONFIG?":
                            for key, value in [("VEL_MAX", self.velocity), ("ACC_MAX", 10000), ("WDT_MS", 100), ("X_MARGIN_UM", 10000)]:
                                self.send(f"OK {key} {value}")
                        elif command.startswith("VEL ") and float(command.split()[1]) != 0:
                            self.driving.set()
                while not self.injections.empty():
                    self.send(self.injections.get_nowait())
                if self.stream:
                    self.send(f"STATE {self.flags} {123 if self.flags & 1 else 0} {self.theta if self.flags & 2 else 0} 0 0")
        except BaseException as exc:
            self.error = exc

    def run(self, *args):
        return subprocess.run([BINARY, "--port", self.path, *args], capture_output=True, text=True, timeout=5)


def package(directory):
    model = b"test model, not an OM file"
    Path(directory, "policy.om").write_bytes(model)
    manifest = Path(directory, "edge-package.json")
    manifest.write_text(json.dumps(dict(schema_version=2,model_cpu=None,model_npu="policy.om",
        sha256_cpu=None,sha256_npu=hashlib.sha256(model).hexdigest(),control_hz=400,
        action_scale_mm_s=900,velocity_limit_mm_s=900,acceleration_mm_s2=10000,
        angle_sign=-1,pause_deg=20,resume_deg=18)))
    return str(manifest)


@unittest.skipUnless(BINARY, "需 make test-integration 构建 PTY 测试应用，或设置 EDGE_TEST_BINARY")
class CommandTests(unittest.TestCase):
    def test_console_preserves_multiple_commands_from_one_stdin_write(self):
        with Firmware() as fw:
            result=subprocess.run([BINARY,"--port",fw.path,"console"],input="query\nstate\nquery\nquit\n",
                                  capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(fw.commands[:fw.commands.index("VEL 0")].count("QUERY"),3)
            self.assertIn("cart x=",result.stdout)
            self.assertIn("DISABLE",fw.commands)

    def test_console_rejects_overlong_line_without_losing_next_command(self):
        with Firmware() as fw:
            result=subprocess.run([BINARY,"--port",fw.path,"console"],input="vel 10"+" "*300+"\nquery\nquit\n",
                                  capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn("? line too long", result.stdout)
            self.assertEqual(fw.commands[:fw.commands.index("VEL 0")].count("QUERY"),2)
            self.assertFalse(any(c.startswith("VEL ") and float(c.split()[1])!=0 for c in fw.commands))

    def test_backend_must_be_explicit_and_supported(self):
        for backend in (None, "bad"):
            with self.subTest(backend=backend), tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
                args = ["run", "--package", package(tmp)]
                if backend is not None:
                    args += ["--backend", backend]
                result = fw.run(*args)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("run requires --backend cpu|npu", result.stderr)
                self.assertEqual(fw.commands, [])

    def test_missing_cpu_model_does_not_fall_back_to_npu(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            result = fw.run("run", "--backend", "cpu", "--package", package(tmp))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("no cpu model; no fallback", result.stderr)
            self.assertEqual(fw.commands, [])

    def start_run(self, fw, tmp):
        process = subprocess.Popen([BINARY, "--port", fw.path, "run", "--backend", "npu", "--package", package(tmp)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(lambda: self.finish(process))
        self.assertTrue(fw.driving.wait(3), "策略未开始发送动作")
        return process

    @staticmethod
    def finish(process):
        if process.poll() is None:
            process.kill()
        process.communicate()

    def test_invalid_cart_cannot_be_hidden_by_valid_frame_in_same_batch(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = self.start_run(fw, tmp)
            fw.injections.put("STATE 2 0 0 0 0\nSTATE 3 123 0 0 0")
            stdout, stderr = process.communicate(timeout=3)
            self.assertNotEqual(process.returncode, 0, stdout + stderr)
            self.assertIn("DISABLE", fw.commands)

    def test_disabled_cannot_be_hidden_by_ready_in_same_batch(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = self.start_run(fw, tmp)
            fw.injections.put("MODE DISABLED NONE\nMODE READY NONE")
            stdout, stderr = process.communicate(timeout=3)
            self.assertNotEqual(process.returncode, 0, stdout + stderr)
            self.assertIn("DISABLE", fw.commands)

    def test_angle_pause_survives_same_batch_return_to_hysteresis_band(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = self.start_run(fw, tmp)
            fw.theta = 331613
            fw.injections.put("STATE 3 123 366520 0 0\nSTATE 3 123 331613 0 0")
            time.sleep(.05)
            fw.driving.clear()
            self.assertFalse(fw.driving.wait(.06), "19度尚未恢复接管却发送非零动作")
            self.assertIsNone(process.poll())
            fw.theta = 0
            self.assertTrue(fw.driving.wait(.5), "回到直立后没有恢复控制")
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stdout + stderr)

    def test_scheduler_gap_cannot_be_hidden_by_buffered_frames(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = self.start_run(fw, tmp)
            try:
                process.send_signal(signal.SIGSTOP)
                time.sleep(.13)
                fw.driving.clear()
            finally:
                process.send_signal(signal.SIGCONT)
            stdout, stderr = process.communicate(timeout=3)
            self.assertNotEqual(process.returncode, 0, stdout + stderr)
            self.assertFalse(fw.driving.is_set(), "调度长停顿后使用积压帧继续控制")

    def test_stream_gap_cannot_be_hidden_by_recovered_frame(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = self.start_run(fw, tmp)
            fw.stream = False
            time.sleep(.13)
            fw.driving.clear()
            fw.stream = True
            stdout, stderr = process.communicate(timeout=3)
            self.assertNotEqual(process.returncode, 0, stdout + stderr)
            self.assertFalse(fw.driving.is_set(), "长时间断流恢复后继续发出了非零动作")

    def test_enable_accepts_mode_without_ok_reply(self):
        with Firmware(mode="DISABLED") as fw:
            result = fw.run("enable")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("ENABLE", fw.commands)

    def test_query_protocol_error_is_not_reported_as_timeout(self):
        with Firmware(mode="INVALID_MODE") as fw:
            result = fw.run("query")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("QUERY: PROTOCOL_ERROR", result.stderr)
            self.assertNotIn("timed out", result.stderr.lower())
            self.assertNotIn("TIMEOUT", result.stderr)

    def test_enable_cancelled_while_waiting_reports_cancellation(self):
        with Firmware(mode="DISABLED", acknowledge_enable=False) as fw:
            process = subprocess.Popen([BINARY, "--port", fw.path, "enable"],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 1
                while "ENABLE" not in fw.commands and time.monotonic() < deadline:
                    time.sleep(.002)
                self.assertIn("ENABLE", fw.commands)
                process.send_signal(signal.SIGINT)
                stdout, stderr = process.communicate(timeout=3)
                self.assertNotEqual(process.returncode, 0, stdout + stderr)
                self.assertIn("ENABLE: CANCELLED", stderr)
                self.assertNotIn("TIMEOUT", stderr)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def test_timeout_reports_unconfirmed_and_no_automatic_retry(self):
        with Firmware(mode="DISABLED", acknowledge_enable=False) as fw:
            result = fw.run("enable")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("ENABLE: TIMEOUT", result.stderr)
            self.assertIn("execution unconfirmed", result.stderr)
            self.assertIn("do not automatically retry", result.stderr)
            self.assertEqual(fw.commands.count("ENABLE"), 1)

    def test_velocity_invalid_cart_is_not_mislabeled_as_bad_velocity(self):
        with Firmware(flags=2) as fw:
            result = fw.run("vel", "100", "--hold", "0.01")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("cart measurement invalid", result.stderr)
            self.assertNotIn("invalid velocity", result.stderr)
            self.assertNotIn("VEL 100", fw.commands)

    def test_state_duration_accepts_argument_and_observes_lines(self):
        with Firmware() as fw:
            result = fw.run("state", "--secs", "0.06")
            self.assertEqual(result.returncode, 0, result.stderr)
            report = parse_state_output(result.stdout).report
            self.assertGreater(report.received_state_count, 0)
            self.assertGreater(report.observed_hz, 0)
            self.assertEqual(report.unknown_line_count, 0)
            self.assertEqual(report.cart_range_um, (123, 123))
            self.assertEqual(report.pole_range_urad, (0, 0))
            self.assertNotIn("usage:", result.stderr)

    def test_state_sampling_accepts_real_async_mode_and_step_loss_events(self):
        with Firmware() as fw:
            process = subprocess.Popen([BINARY, '--port', fw.path, 'state', '--secs', '0.3'],
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.addCleanup(lambda: self.finish(process))
            # 在采样区间重复送达，避免用启动时序猜测 UART 是否已打开。
            for _ in range(5):
                time.sleep(.03)
                fw.injections.put('MODE READY NONE\nEVENT STEP_LOSS 12 34 5678')
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            parsed = parse_state_output(stdout)
            self.assertIn(ModeEvent('READY', 'NONE'), parsed.events)
            self.assertIn(StepLossEvent(12, 34, 5678), parsed.events)
            self.assertGreater(parsed.report.received_state_count, 0)
            self.assertEqual(parsed.report.cart_range_um, (123, 123))
            self.assertFalse(any(command.startswith('VEL ') for command in fw.commands))

    def test_state_pole_invalid_not_displayed_as_valid_zero_angle(self):
        with Firmware(flags=1) as fw:
            result = fw.run("state", "--secs", "0.06")
            self.assertEqual(result.returncode, 0, result.stderr)
            report = parse_state_output(result.stdout).report
            self.assertEqual(report.cart_range_um, (123, 123))
            self.assertIsNone(report.pole_range_urad)
            self.assertIn("pole samples unavailable", result.stdout)
            self.assertNotIn("sampled theta range", result.stdout)

    def test_state_without_stream_fails(self):
        with Firmware(stream=False) as fw:
            result = fw.run("state", "--secs", "0.01")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("STATE: TIMEOUT", result.stderr)
            self.assertFalse(any(c.startswith("VEL ") for c in fw.commands))

    def test_invalid_command_does_not_open_port(self):
        result = subprocess.run([BINARY, "--port", "/definitely/not/a/device", "bogus"], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("usage:", result.stderr)
        self.assertNotIn("cannot open", result.stderr)

    def test_vel_exit_confirms_disabled(self):
        with Firmware() as fw:
            result = fw.run("vel", "10", "--hold", "0.03")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("VEL 0", fw.commands)
            self.assertIn("DISABLE", fw.commands)
            self.assertEqual(fw.mode, "DISABLED")
            moving = [i for i, c in enumerate(fw.commands) if c.startswith("VEL ") and float(c.split()[1]) != 0]
            self.assertTrue(moving)
            zero = fw.commands.index("VEL 0", moving[-1] + 1)
            self.assertLess(zero, fw.commands.index("DISABLE"))

    def test_vel_exit_missing_disabled_is_failure(self):
        with Firmware(acknowledge_stop=False) as fw:
            result = fw.run("vel", "10", "--hold", "0.03")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("DISABLED not confirmed", result.stderr)
            self.assertIn("DISABLE", fw.commands)

    def test_run_mismatch_emits_no_velocity_or_configuration(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware(velocity=1250) as fw:
            result = fw.run("run", "--backend", "npu", "--package", package(tmp))
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("CONFIG?", fw.commands)
            self.assertFalse(any(c.startswith(("VEL ", "CONFIG ")) for c in fw.commands), fw.commands)

    def test_run_drives_without_enable_or_auto_center_and_confirms_stop(self):
        with tempfile.TemporaryDirectory() as tmp, Firmware() as fw:
            process = subprocess.Popen([BINARY, "--port", fw.path, "run", "--backend", "npu", "--package", package(tmp)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                drove = fw.driving.wait(3)
                process.send_signal(signal.SIGINT)
                stdout, stderr = process.communicate(timeout=4)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
            self.assertTrue(drove, stderr)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertFalse(any(c.startswith("CONFIG ") or c in ("ENABLE", "RESET", "MARK LEFT", "MARK RIGHT") for c in fw.commands), fw.commands)
            moving = [c for c in fw.commands if c.startswith("VEL ") and float(c.split()[1]) != 0]
            self.assertTrue(moving)
            self.assertTrue(all(float(c.split()[1]) == 225 for c in moving), moving)
            self.assertIn("DISABLE", fw.commands)
            self.assertEqual(fw.mode, "DISABLED")


if __name__ == "__main__":
    unittest.main()
