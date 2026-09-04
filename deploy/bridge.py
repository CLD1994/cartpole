#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Real-hardware bridge for the trained cartpole policy.

Runs the exported ONNX policy at 60 Hz: reads the cart encoder + pole pot from the
STM32 over USB serial, builds the observation ``[cart_pos, pole_angle, cart_vel,
pole_vel]`` in the same units/sign as training, runs the policy, and sends a signed
PWM command back.

Usage:
    python bridge.py --port COM3 --onnx exported/policy.onnx
    python bridge.py --port COM3 --manual        # keyboard jog (no policy)

The calibration constants below come from `calibrate_pot.py` and the hardware specs.
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
import time

import numpy as np
import serial

# ---- fill in from calibration / hardware ----
# NO x4 quadrature factor: the encoder interface counts 1 per PPR line. Verified by ruler
# against raw counts: full travel = ~3142 counts over ~0.363 m usable (41 cm rail minus cart).
METERS_PER_COUNT = math.pi * 0.0382 / (20 * 52)  # cart, 1.154e-4 m/count (2GT-60T, gearbox 20, PPR 52)
RAD_PER_COUNT = -0.0020010  # pole, rad per ADC count (from calibrate_pot.py)
POLE_ZERO_ADC = 163  # ADC reading when the pole is vertical (from calibrate_pot.py)
PWM_PER_ACTION = 1000  # map action [-1, 1] -> pwm [-1000, 1000]
CONTROL_HZ = 60.0
SERIAL_TIMEOUT = 0.05
STATE_TIMEOUT = 0.2  # s; stop the motor if no state line arrives for this long
TRACK_HALF_LENGTH = 0.181  # m; 可用行程 36.3 cm（轨道 41 cm 减小车宽度）的一半


def parse_state(line: str) -> tuple[int, int] | None:
    """Parse an ``S <cart_count> <pole_adc>`` line, or None."""
    parts = line.split()
    if len(parts) == 3 and parts[0] == "S":
        try:
            return int(parts[1]), int(parts[2])
        except ValueError:
            return None
    return None


def read_states(serial_port: serial.Serial, buf: bytearray) -> list[tuple[int, int]]:
    """非阻塞读取所有可用字节，解析完整行；不完整尾部留在 buf 等下次拼接。

    返回本帧新解析出的状态列表（最后一项最新）。缺 ``\\n`` 时绝不阻塞，
    避免 200Hz 连续流下 ``while in_waiting: readline()`` 永不退出的问题。
    """
    if serial_port.in_waiting:
        buf.extend(serial_port.read(serial_port.in_waiting))
    if len(buf) > 256:  # 无 \n 时防无限增长：只丢旧数据，保留最后一条不完整行
        nl = buf.rfind(b"\n")
        if nl >= 0:
            del buf[: nl + 1]
        else:
            buf.clear()
    states: list[tuple[int, int]] = []
    while True:
        idx = buf.find(b"\n")
        if idx < 0:
            break
        st = parse_state(buf[:idx].decode(errors="ignore").strip())
        del buf[: idx + 1]
        if st is not None:
            states.append(st)
    return states


def send_pwm(serial_port: serial.Serial, pwm: float) -> None:
    serial_port.write(f"P {int(round(max(-1000.0, min(1000.0, pwm))))}\n".encode())


def _stop_motor(serial_port: serial.Serial) -> None:
    """Best-effort motor stop; never raises (used in finally blocks)."""
    with contextlib.suppress(Exception):
        send_pwm(serial_port, 0)


def _clear_line() -> None:
    """Clear the current terminal line."""
    sys.stdout.write("\r" + " " * 90 + "\r")
    sys.stdout.flush()


def manual_jog(serial_port: serial.Serial) -> None:
    """Keyboard jog: w = +, s = -, space = stop, q = quit."""
    is_windows = sys.platform.startswith("win")

    # Print the help in cooked mode (before entering raw mode) so the newline
    # is translated properly.
    print("[bridge] manual jog: w=+  s=-  space=stop  q=quit")

    if is_windows:
        import msvcrt
    else:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)

    buf = bytearray()
    try:
        while True:
            # ---- 1) drain serial buffer (non-blocking bulk read) ----
            states = read_states(serial_port, buf)
            if states:
                st = states[-1]  # 取最新一帧
                _clear_line()
                pole_rad = (st[1] - POLE_ZERO_ADC) * RAD_PER_COUNT
                sys.stdout.write(
                    f"cart={st[0]:>6} ({st[0] * METERS_PER_COUNT:+.3f} m)"
                    f"  pole_adc={st[1]:>5} ({pole_rad:+.3f} rad)"
                )
                sys.stdout.flush()

            # ---- 2) non-blocking keyboard check ----
            key = None
            if is_windows:
                import msvcrt

                if msvcrt.kbhit():
                    raw = msvcrt.getch()
                    try:
                        key = raw.decode("ascii", errors="ignore")
                    except UnicodeDecodeError:
                        key = None
            else:
                import select

                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)

            # ---- 3) handle key ----
            if key:
                if key == "w":
                    send_pwm(serial_port, 400)
                elif key == "s":
                    send_pwm(serial_port, -400)
                elif key == " ":
                    send_pwm(serial_port, 0)
                elif key in ("q", "\x03"):  # q or Ctrl+C
                    break

            time.sleep(0.01)  # 10 ms poll to keep CPU usage low
    finally:
        if not is_windows:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        _clear_line()
        _stop_motor(serial_port)  # always stop, even on exception/disconnect


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="COM port of the STM32")
    ap.add_argument("--onnx", default="exported/policy.onnx")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--manual", action="store_true", help="keyboard jog, no policy")
    args = ap.parse_args()

    if RAD_PER_COUNT == 0.0015 and POLE_ZERO_ADC == 2048:
        print("[bridge] WARNING: RAD_PER_COUNT/POLE_ZERO_ADC are placeholders — run calibrate_pot.py first")

    serial_port = serial.Serial(args.port, args.baud, timeout=SERIAL_TIMEOUT)
    serial_port.reset_input_buffer()

    if args.manual:
        try:
            manual_jog(serial_port)
        finally:
            _stop_motor(serial_port)
            serial_port.close()
        return

    # ---- policy mode ----
    try:
        import onnxruntime as ort

        sess = ort.InferenceSession(args.onnx)
        in_name = sess.get_inputs()[0].name
        print(
            f"[bridge] onnx inputs: {[i.name for i in sess.get_inputs()]}, "
            f"outputs: {[o.name for o in sess.get_outputs()]}"
        )
    except Exception as e:
        serial_port.close()
        sys.stderr.write(f"[bridge] failed to load ONNX policy: {e}\n")
        sys.exit(1)

    # Running state – explicitly initialise everything
    cart_pos = 0.0
    pole_angle = 0.0
    cart_vel = 0.0
    pole_vel = 0.0
    cart_zero = 0.0
    prev_cart_pos = 0.0
    prev_pole_angle = 0.0
    have_state = False  # gate: don't drive until the first valid state arrives
    first = True  # cart position zeroing on the first state
    t_prev = time.perf_counter()
    last_state_t = time.perf_counter()
    limit_tripped = False  # warn once when the cart reaches the track end

    print(f"[bridge] running policy at {CONTROL_HZ} Hz. Ctrl+C to stop.")
    buf = bytearray()
    try:
        while True:
            # 1) read the latest streamed state (non-blocking bulk read, keep newest)
            states = read_states(serial_port, buf)
            if states:
                cart_count, pole_adc = states[-1]
                if first:
                    cart_zero = cart_count * METERS_PER_COUNT
                    first = False
                cart_pos = cart_count * METERS_PER_COUNT - cart_zero
                pole_angle = (pole_adc - POLE_ZERO_ADC) * RAD_PER_COUNT
                last_state_t = time.perf_counter()

            now = time.perf_counter()

            # 2) wait for the first valid state before driving
            if not have_state:
                if not first:
                    # first state just arrived: seed prev, skip this control step
                    have_state = True
                    prev_cart_pos, prev_pole_angle = cart_pos, pole_angle
                    t_prev = now
                time.sleep(1.0 / CONTROL_HZ)
                continue

            # 3) state watchdog: MCU stopped streaming / link issue -> stop & wait
            if now - last_state_t > STATE_TIMEOUT:
                _stop_motor(serial_port)
                have_state = False  # re-seed prev when state resumes (no spike)
                time.sleep(1.0 / CONTROL_HZ)
                continue

            # 4) velocities (state-to-state over actual elapsed time; skip frames with
            #    no new state so a missed frame doesn't read 0 then double-count)
            if states:
                dt = max(now - t_prev, 1e-3)
                cart_vel = (cart_pos - prev_cart_pos) / dt
                pole_vel = (pole_angle - prev_pole_angle) / dt
                prev_cart_pos, prev_pole_angle = cart_pos, pole_angle
                t_prev = now

            # 5) policy inference
            obs = np.array([[cart_pos, pole_angle, cart_vel, pole_vel]], dtype=np.float32)
            action = float(np.asarray(sess.run(None, {in_name: obs})).reshape(-1)[0])
            action = max(-1.0, min(1.0, action))

            # 6) track-end safety: block outward push past the limit (allow recovery inward)
            if cart_pos > TRACK_HALF_LENGTH and action > 0.0 or cart_pos < -TRACK_HALF_LENGTH and action < 0.0:
                action = 0.0
            if abs(cart_pos) > TRACK_HALF_LENGTH and not limit_tripped:
                limit_tripped = True
                print(
                    f"[bridge] WARNING: cart at track end "
                    f"(|pos|={abs(cart_pos):.3f} m > {TRACK_HALF_LENGTH} m); blocking outward motion"
                )

            # 7) action -> PWM
            send_pwm(serial_port, action * PWM_PER_ACTION)

            # 8) keep the loop at 60 Hz
            sleep = (1.0 / CONTROL_HZ) - (time.perf_counter() - now)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as e:
        sys.stderr.write(f"\n[bridge] serial error: {e}\n")
    finally:
        _stop_motor(serial_port)
        serial_port.close()
        print("\n[bridge] stopped, motor off.")


if __name__ == "__main__":
    main()
