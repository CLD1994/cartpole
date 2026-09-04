#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Two-point calibration for the pole potentiometer.

Reads the live ADC stream from the STM32, records the ADC at two known pole angles,
and prints the constants to paste into `bridge.py`. Averages many samples at each
point (pot/ADC jitter) and drains the serial buffer first so it never reads a stale
value left over from while you were positioning the pole.

Usage:
    python calibrate_pot.py --port COM3
    python calibrate_pot.py --port COM3 --zero-adc 1850  # skip auto-measuring the vertical ADC

Procedure:
    1. Hold the pole VERTICAL, press ENTER            (records ADC at 0 deg)
    2. Tilt the pole to a large known angle (30-60 deg recommended), type it in
       degrees, press ENTER                           (records ADC at that angle)
    3. Verify the sign: tilt the pole TOP toward +x (the direction the cart moves
       on +action); the printed angle must be positive, otherwise the script flips
       the sign for you.
    4. Paste the printed RAD_PER_COUNT / POLE_ZERO_ADC into bridge.py
"""

import argparse
import contextlib
import math
import sys
import time
import unicodedata

import serial


def sample_adc(serial_port: serial.Serial, n: int = 50, timeout_s: float = 3.0) -> int:
    """Drain the buffer, then return the average of `n` fresh ``S <count> <adc>`` samples.

    Raises:
        RuntimeError: no valid stream line within `timeout_s` (firmware not running,
        wrong port, or USB link down).
    """
    serial_port.reset_input_buffer()  # discard stale data from while you were positioning
    serial_port.timeout = timeout_s
    values: list[int] = []
    deadline = time.time() + timeout_s
    while len(values) < n and time.time() < deadline:
        line = serial_port.readline()
        if not line:
            continue
        parts = line.decode(errors="ignore").strip().split()
        if len(parts) == 3 and parts[0] == "S":
            with contextlib.suppress(ValueError):
                values.append(int(parts[2]))
    if not values:
        raise RuntimeError("no 'S <count> <adc>' stream received — is the firmware running, and is --port correct?")
    return round(sum(values) / len(values))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="COM port of the STM32")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--zero-adc",
        type=int,
        default=None,
        help="Manually specify the ADC reading at vertical (0 deg); skips step 1 auto-sampling.",
    )
    args = ap.parse_args()

    serial_port = serial.Serial(args.port, args.baud, timeout=0.1)

    print("== Cartpole pole-pot calibration ==")
    print("(make sure the STM32 firmware is streaming 'S <count> <adc>')")

    try:
        if args.zero_adc is not None:
            adc_0 = args.zero_adc
            print(f"1. Using manually-specified adc at 0 deg = {adc_0}")
        else:
            input("1. Hold the pole VERTICAL, then press ENTER...")
            adc_0 = sample_adc(serial_port)
            print(f"   adc at 0 deg = {adc_0}")

        while True:
            deg_s = input("2. Tilt the pole to a large known angle (30-60 deg recommended) and enter it in degrees: ")
            # 全角数字（中文输入法）→ 半角，再去掉首尾空白
            deg_s = unicodedata.normalize("NFKC", deg_s).strip()
            try:
                deg = float(deg_s)
            except ValueError:
                print(f"   couldn't read a number from {deg_s!r} — type just the number, e.g. 45")
                continue
            if abs(deg) < 5.0:
                print("   angle too small for good accuracy — tilt more (>= ~20 deg)")
                continue
            break
        adc_a = sample_adc(serial_port)
        print(f"   adc at {deg:g} deg = {adc_a}")

        span = adc_a - adc_0
        if span == 0:
            sys.exit("ADC did not change between the two positions — is the pot wired to the pole?")
        rad_per_count = math.radians(deg) / span

        # ---- live sign check ----
        print("\n== Verify direction ==")
        print("Tilt the pole TOP toward +x (the direction the cart moves on +action).")
        input("Hold it there, then press ENTER...")
        adc_t = sample_adc(serial_port)
        ang = (adc_t - adc_0) * rad_per_count
        print(f"   live angle = {ang:+.3f} rad ({math.degrees(ang):+.1f} deg)")
        if ang < 0.0:
            rad_per_count = -rad_per_count
            print("   came out NEGATIVE -> flipping RAD_PER_COUNT sign.")

        print("\n== Paste these into bridge.py ==")
        print(f"RAD_PER_COUNT = {rad_per_count:.7f}   # rad per ADC count")
        print(f"POLE_ZERO_ADC = {adc_0}              # ADC when pole is vertical")
    except RuntimeError as e:
        sys.exit(f"[calibrate] {e}")
    finally:
        serial_port.close()


if __name__ == "__main__":
    main()
