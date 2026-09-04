#!/usr/bin/env python3
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""PWM -> force calibration for the cartpole rig.

Sends constant PWM commands and records the 200 Hz encoder stream, then fits
the three parameters the simulation needs:

    m * x'' = G * pwm - b * sign(v) - c * v

  * G     force gain [N per PWM unit]      -> sim action scale = G * 1000
  * b     Coulomb friction [N]             -> sim deadzone (max of both directions)
  * c     viscous damping [N*s/m]          -> sim joint damping range
  * tau   velocity rise time constant [s]  -> sim motor lag range
  * stiction threshold [PWM -> N]          -> cross-check for the deadzone

Procedure per run: start at rest near one end, constant PWM until the cart
approaches the other end, then P 0. Both directions, several PWM levels.

Prerequisites (read before running):
  1. Place the cart at the TRACK CENTER - the script zeros its position there
     and verifies that +PWM drives the encoder count up (aborts on sign
     mismatch). The usable travel range is then measured by driving to both
     ends, so no assumption about the track length is needed.
  2. Pole hanging straight down and settled (a hanging pole adds almost no
     effective horizontal mass; with the pole removed the math is identical).
  3. Nothing on the track; the cart WILL traverse the full rail repeatedly.

Usage:
    python calibrate_force.py --port COM3
    python calibrate_force.py --port COM3 --mass 0.15 --pwms 150,250,400,600,800
    python calibrate_force.py --reanalyze force_calib_raw.csv --mass 0.15
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import math
import sys
import time

import serial

METERS_PER_COUNT = math.pi * 0.0382 / (20 * 52)  # 1.154e-4 m/count (same as bridge.py; no x4 quadrature)
SAMPLE_HZ = 200.0  # firmware stream rate
PROBE_STALL_S = 1.5  # stalled this long at max jog PWM = end of travel
END_MARGIN_M = 0.008  # keep this far from the physical/firmware ends during runs
START_FRAC = 0.18  # settle this fraction of the span away from the start end
MOTION_ONSET_M = 1.5e-3  # motion detected once the cart moves this far from rest
STOP_HOLD_S = 0.5  # run ends when the cart has been this slow for this long after moving
JOG_PWM = 120  # initial positioning speed
MAX_JOG_PWM = 300  # positioning escalation cap (positioning is not a measurement)
STALL_TIME_S = 1.0  # "stalled" = less than 1 mm of progress in this window
STALL_EPS_M = 1.0e-3
RUN_TIMEOUT_S = 4.0
SETTLE_TIMEOUT_S = 8.0
MOVE_EPS_M = 2.0e-3  # "the cart moved" threshold for the stiction sweep
MIN_RUN_DISP_M = 0.02  # a run that traveled less than this is discarded (no breakaway)
SIGN_CHECK_PWM = 300  # preflight pulse level for the PWM/encoder sign check


# ---------------------------------------------------------------------------- serial I/O
# (same line protocol helpers as bridge.py)


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
    """Non-blocking bulk read; return all complete lines parsed this call."""
    if serial_port.in_waiting:
        buf.extend(serial_port.read(serial_port.in_waiting))
    if len(buf) > 256:
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
    with contextlib.suppress(Exception):
        send_pwm(serial_port, 0)


# ---------------------------------------------------------------------------- rig helpers


def read_for(serial_port: serial.Serial, buf: bytearray, duration_s: float) -> list[tuple[float, int]]:
    """Collect ``(t, count)`` samples for the given duration [s]."""
    out: list[tuple[float, int]] = []
    t_end = time.perf_counter() + duration_s
    while time.perf_counter() < t_end:
        for count, _adc in read_states(serial_port, buf):
            out.append((time.perf_counter(), count))
        time.sleep(0.002)
    return out


def wait_settled(serial_port: serial.Serial, buf: bytearray, timeout_s: float = SETTLE_TIMEOUT_S) -> bool:
    """True once the cart stops moving (20 ms window below threshold), or False on timeout."""
    t0 = time.perf_counter()
    recent: list[tuple[float, float]] = []
    while time.perf_counter() - t0 < timeout_s:
        for count, _adc in read_states(serial_port, buf):
            recent.append((time.perf_counter(), count * METERS_PER_COUNT))
            if len(recent) > 12:
                recent = recent[-12:]
        if len(recent) >= 6:
            (ta, pa), (tb, pb) = recent[0], recent[-1]
            if tb - ta > 0.05 and abs(pb - pa) < 1.0e-3:
                return True
        time.sleep(0.005)
    return False


def count_to_pos(count: int, offset_counts: int) -> float:
    """Cart position [m] relative to the software zero (measured track center)."""
    return (count - offset_counts) * METERS_PER_COUNT


def preflight(serial_port: serial.Serial, buf: bytearray) -> int:
    """Software-zero at the current position and verify the PWM/encoder sign convention.

    The cart must be sitting at the TRACK CENTER when this runs. A short +/- pulse pair
    confirms that +PWM drives the encoder count up; a mismatch (DIR pin, encoder wiring,
    or sign convention) would otherwise silently mirror the whole calibration and slam
    the cart against the end stops.

    Returns:
        The encoder count at the track center (software zero).
    """
    # 1) software zero: average 0.5 s of the stream at the current position
    samples = read_for(serial_port, buf, 0.5)
    if not samples:
        sys.exit("[calib] preflight: no state stream - check the port and the firmware")
    offset = round(sum(c for _, c in samples) / len(samples))

    # 2) sign check: +PWM must move the cart toward +x (count up), and vice versa
    for expected in (+1, -1):
        c0 = None
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 0.3 and c0 is None:
            states = read_states(serial_port, buf)
            if states:
                c0 = states[-1][0]
            time.sleep(0.005)
        if c0 is None:
            sys.exit("[calib] preflight: no state stream during the sign check")
        moved = 0
        t0 = time.perf_counter()
        send_pwm(serial_port, expected * SIGN_CHECK_PWM)
        while time.perf_counter() - t0 < 0.6:
            for st in read_states(serial_port, buf):
                moved = st[0] - c0
            if abs(moved) * METERS_PER_COUNT > 3.0e-3:
                break
            time.sleep(0.002)
        send_pwm(serial_port, 0)
        wait_settled(serial_port, buf, timeout_s=3.0)
        if abs(moved) * METERS_PER_COUNT <= 3.0e-3:
            sys.exit(f"[calib] preflight: PWM {expected * SIGN_CHECK_PWM} did not move the cart - drive problem?")
        if (moved > 0) != (expected > 0):
            sys.exit(
                f"[calib] preflight: SIGN MISMATCH - PWM {expected * SIGN_CHECK_PWM:+d} moved the encoder "
                f"{moved:+d} counts. +PWM must drive the cart toward +x (count up). Fix the DIR pin / "
                "encoder wiring, or recalibrate the sign convention (see deploy/protocol.md)."
            )
    return offset


def latest_pos(serial_port: serial.Serial, buf: bytearray, offset_counts: int, timeout_s: float = 0.2) -> float | None:
    """Poll the stream until a fresh state arrives; return the cart position [m], or None."""
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout_s:
        states = read_states(serial_port, buf)
        if states:
            return count_to_pos(states[-1][0], offset_counts)
        time.sleep(0.005)
    return None


def probe_travel(serial_port: serial.Serial, buf: bytearray, offset_counts: int) -> tuple[float, float]:
    """Drive to each physical end (escalating PWM on stall); return (min_pos, max_pos).

    The usable travel is measured, not assumed: a stall at MAX_JOG_PWM held for
    PROBE_STALL_S is declared the end of travel (sticky spots that even the maximum
    jog force cannot pass are conservatively treated as ends).
    """
    limits = []
    for direction in (+1, -1):
        pwm_mag = JOG_PWM
        extreme = 0.0
        last_pos: float | None = None
        last_progress_t = time.perf_counter()
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 20.0:
            states = read_states(serial_port, buf)
            if states:
                pos = count_to_pos(states[-1][0], offset_counts)
                extreme = max(extreme, direction * pos)
                if last_pos is None or abs(pos - last_pos) > STALL_EPS_M:
                    last_pos = pos
                    last_progress_t = time.perf_counter()
                elif time.perf_counter() - last_progress_t > PROBE_STALL_S:
                    if pwm_mag < MAX_JOG_PWM:
                        pwm_mag += 30
                        last_progress_t = time.perf_counter()
                        print(f"[calib] probe dir {direction:+d}: stalled at {pos:.3f} m - raising PWM to {pwm_mag}")
                    else:
                        break  # stalled at maximum jog force: this is the travel limit
                send_pwm(serial_port, direction * pwm_mag)
            time.sleep(0.01)
        send_pwm(serial_port, 0)
        wait_settled(serial_port, buf, timeout_s=4.0)
        pos = latest_pos(serial_port, buf, offset_counts)
        if pos is not None:
            extreme = max(extreme, direction * pos)
        limits.append(direction * extreme)  # +1 -> max_pos, -1 -> min_pos
    return limits[1], limits[0]


def goto_pos(serial_port: serial.Serial, buf: bytearray, offset_counts: int, target_pos: float) -> float | None:
    """Bring the cart to ~``target_pos`` from wherever it is; return the settled position.

    Positioning is not part of the measurement, so the drive force escalates when the
    cart stalls (rail/belt friction is position-dependent) in steps of 30 up to
    MAX_JOG_PWM, and drops back to JOG_PWM for the final 2 cm approach. Bidirectional:
    the cart may already be beyond the target (e.g. left at an end by the probe).

    Returns None if the target was not reached - the caller must skip the run.
    """
    pwm_mag = JOG_PWM
    last_pos: float | None = None
    last_progress_t = time.perf_counter()
    reached = False
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 15.0:
        states = read_states(serial_port, buf)
        if states:
            pos = count_to_pos(states[-1][0], offset_counts)
            err = target_pos - pos
            if abs(err) < 0.004:
                reached = True
                break
            cmd = (JOG_PWM if abs(err) < 0.02 else pwm_mag) * (1 if err > 0 else -1)
            if last_pos is None or abs(pos - last_pos) > STALL_EPS_M:
                last_pos = pos
                last_progress_t = time.perf_counter()
            elif time.perf_counter() - last_progress_t > STALL_TIME_S and pwm_mag < MAX_JOG_PWM:
                pwm_mag += 30
                last_progress_t = time.perf_counter()
                print(f"[calib] positioning stalled at {pos:.3f} m - raising jog PWM to {pwm_mag}")
            send_pwm(serial_port, cmd)
        time.sleep(0.01)
    send_pwm(serial_port, 0)
    if not wait_settled(serial_port, buf):
        print("[calib] WARNING: cart did not settle cleanly before the run")
    pos = latest_pos(serial_port, buf, offset_counts)
    if not reached or pos is None or abs(pos - target_pos) > 0.010:
        print(f"[calib] positioning FAILED (target {target_pos:.3f} m, settled at {pos} m) - skipping this run")
        return None
    return pos


def run_constant_pwm(
    serial_port: serial.Serial,
    buf: bytearray,
    pwm_signed: int,
    offset_counts: int,
    bounds: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """From rest, apply a constant PWM until leaving ``bounds`` or timeout; return (t_rel, pos_m).

    Returns None if the cart did not travel at least MIN_RUN_DISP_M (PWM below the
    breakaway threshold, or stalled) - such a run has no useful dynamics.
    """
    samples: list[tuple[float, float]] = []
    t_start = time.perf_counter()
    send_pwm(serial_port, pwm_signed)
    last_x: float | None = None
    last_move_t = t_start
    try:
        while time.perf_counter() - t_start < RUN_TIMEOUT_S:
            for count, _adc in read_states(serial_port, buf):
                t = time.perf_counter() - t_start
                pos = count_to_pos(count, offset_counts)
                samples.append((t, pos))
                if pos < bounds[0] or pos > bounds[1]:
                    raise StopIteration
                # natural end: the cart has moved, then stopped (hit the end region or
                # the firmware clamp) - sampling a parked cart only adds flat data
                if last_x is None or abs(pos - last_x) > 5.0e-4:
                    last_x = pos
                    last_move_t = time.perf_counter()
                elif time.perf_counter() - last_move_t > STOP_HOLD_S and abs(pos - samples[0][1]) > MIN_RUN_DISP_M:
                    raise StopIteration
            time.sleep(0.002)
    except StopIteration:
        pass
    finally:
        send_pwm(serial_port, 0)
        wait_settled(serial_port, buf, timeout_s=3.0)
    if len(samples) < 20 or abs(samples[-1][1] - samples[0][1]) < MIN_RUN_DISP_M:
        return None
    return samples


def stiction_sweep(serial_port: serial.Serial, buf: bytearray, direction: int) -> int | None:
    """Ramp PWM upward until the cart breaks away; return the breakaway PWM value."""
    for pwm in range(60, 421, 30):
        states = read_states(serial_port, buf)  # drain
        if not states:
            time.sleep(0.05)
            continue
        pos0 = states[-1][0] * METERS_PER_COUNT
        send_pwm(serial_port, direction * pwm)
        moved_max = 0.0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 0.45:
            for st in read_states(serial_port, buf):
                moved_max = max(moved_max, abs(st[0] * METERS_PER_COUNT - pos0))
            if moved_max > MOVE_EPS_M:
                break
            time.sleep(0.002)
        send_pwm(serial_port, 0)
        wait_settled(serial_port, buf, timeout_s=2.0)
        if moved_max > MOVE_EPS_M:
            return pwm
    return None


# ---------------------------------------------------------------------------- fitting (pure python)


def moving_segment(samples: list[tuple[float, float]]) -> tuple[list[tuple[float, float]], float] | None:
    """Extract the moving part of a run, aligned to the motion onset.

    The command reaches the motor only after the serial link, the firmware poll, and
    breakaway (~tens of ms); a fit over the raw timeline misreads that dead time as a
    large negative initial velocity. This returns samples from the motion onset until
    the cart has slowed below 15% of its peak speed (dropping any parked-at-the-end
    tail), with time rebased to zero at the onset.

    Returns:
        (segment, delay_s) where delay_s is the command-to-motion latency, or None if
        no clean motion segment exists.
    """
    if len(samples) < 20:
        return None
    x0 = samples[0][1]
    i0 = next((i for i, (_, x) in enumerate(samples) if abs(x - x0) > MOTION_ONSET_M), None)
    if i0 is None or i0 > len(samples) - 6:
        return None
    _, vs = velocity_series(samples)
    peak = max(abs(v) for v in vs[i0:])
    if peak < 0.02:
        return None
    i1 = i0
    for i in range(len(vs) - 1, i0, -1):
        if abs(vs[i]) > 0.15 * peak:
            i1 = i
            break
    seg = samples[i0 : i1 + 1]
    if len(seg) < 6:
        return None
    t0 = seg[0][0]
    return [(t - t0, x) for t, x in seg], t0


def fit_first_order(samples: list[tuple[float, float]], window_s: float = 0.8) -> tuple[float, float, float] | None:
    """Fit ``x(t) = x0 + v_ss*((t-d) - T*(1 - exp(-(t-d)/T)))`` (0 motion before ``d``).

    Grid-searches the command-to-motion delay ``d`` [s] and the velocity time constant
    ``T`` [s]; ``x0`` and ``v_ss`` are closed-form least squares per (d, T). Returns
    (delay, T, v_ss), or None if no positive-velocity fit is found. Only the first
    ``window_s`` seconds are used, so a parked-at-the-end tail cannot pollute the fit.

    The initial acceleration follows exactly: a(0) = v_ss / T = F_net / m.
    """
    ts = [t for t, _ in samples if t <= window_s]
    xs = [x for t, x in samples if t <= window_s]
    if len(ts) < 10:
        return None
    n = len(ts)
    best: tuple[float, float, float, float] | None = None
    d = 0.0
    while d <= 0.30:
        t_param = 0.004
        while t_param <= 0.6:
            us = [max(0.0, (t - d) - t_param * (1.0 - math.exp(-(max(t - d, 0.0)) / t_param))) for t in ts]
            su = sum(us)
            suu = sum(u * u for u in us)
            det = n * suu - su * su
            if det > 1e-12:
                sx = sum(xs)
                sxu = sum(x * u for x, u in zip(xs, us))
                x0 = (suu * sx - su * sxu) / det
                vss = (n * sxu - su * sx) / det
                if vss > 0.01:
                    sse = sum((x - x0 - vss * u) ** 2 for x, u in zip(xs, us))
                    if best is None or sse < best[3]:
                        best = (d, t_param, vss, sse)
            t_param *= 1.08
        d += 0.005
    if best is None:
        return None
    return best[0], best[1], best[2]


def linreg(xs: list[float], ys: list[float]) -> tuple[float, float, float]:
    """Least-squares y = slope*x + intercept; return (slope, intercept, r2)."""
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    det = n * sxx - sx * sx
    slope = (n * sxy - sx * sy) / det
    intercept = (sy - slope * sx) / n
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - sy / n) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return slope, intercept, r2


def fit_initial_accel(samples: list[tuple[float, float]], window_s: float = 0.15) -> tuple[float, float]:
    """Fit x = v0*t + (a/2)*t^2 over the first window; return (a, v0).

    Cross-check only: the velocity rise decays within a few motor time constants, so a
    long-window quadratic underestimates the true initial acceleration. The regression
    uses a0 = v_ss / tau from the exponential fit instead.
    """
    ts = [t for t, _ in samples if t <= window_s]
    xs = [x for t, x in samples if t <= window_s]
    if len(ts) < 6:
        return 0.0, 0.0
    # normal equations for basis {t, t^2}
    s11 = sum(t * t for t in ts)
    s12 = sum(t**3 for t in ts)
    s22 = sum(t**4 for t in ts)
    b1 = sum(t * x for t, x in zip(ts, xs))
    b2 = sum(t * t * x for t, x in zip(ts, xs))
    det = s11 * s22 - s12 * s12
    v0 = (s22 * b1 - s12 * b2) / det
    half_a = (s11 * b2 - s12 * b1) / det
    return 2.0 * half_a, v0


def steady_velocity(samples: list[tuple[float, float]]) -> tuple[float, float]:
    """Slope of x(t) over the last 40% of the run; return (v_ss, r2)."""
    if not samples:
        return 0.0, 0.0
    t_end = samples[-1][0]
    win = [(t, x) for t, x in samples if t >= 0.6 * t_end]
    if len(win) < 6:
        return 0.0, 0.0
    slope, _ic, r2 = linreg([t for t, _ in win], [x for _, x in win])
    return slope, r2


def velocity_series(samples: list[tuple[float, float]]) -> tuple[list[float], list[float]]:
    """Smoothed central-difference velocity; return (ts, vs)."""
    ts = [t for t, _ in samples]
    xs = [x for _, x in samples]
    raw = [0.0] * len(xs)
    for i in range(1, len(xs) - 1):
        raw[i] = (xs[i + 1] - xs[i - 1]) / (ts[i + 1] - ts[i - 1] + 1e-9)
    vs = []
    for i in range(len(raw)):
        lo, hi = max(0, i - 2), min(len(raw), i + 3)
        vs.append(sum(raw[lo:hi]) / (hi - lo))
    return ts, vs


def fit_tau(samples: list[tuple[float, float]]) -> tuple[float, float]:
    """Fit v(t) = v_ss*(1 - exp(-t/T)); return (T, v_ss)."""
    ts, vs = velocity_series(samples)
    if len(ts) < 10:
        return 0.0, 0.0
    best = (None, 0.0, float("inf"))
    t = 0.004
    while t < 0.5:
        us = [1.0 - math.exp(-x / t) for x in ts]
        denom = sum(u * u for u in us)
        if denom > 1e-9:
            vss = sum(v * u for v, u in zip(vs, us)) / denom
            sse = sum((v - vss * u) ** 2 for v, u in zip(vs, us))
            if vss > 0 and sse < best[2]:
                best = (t, vss, sse)
        t *= 1.06
    return (best[0], best[1]) if best[0] is not None else (0.0, 0.0)


# ---------------------------------------------------------------------------- analysis


def analyze(runs: list[dict], mass: float, stiction: dict[int, int | None]) -> None:
    """Regress per-run results into G, b, c, tau and print suggested sim parameters."""
    print("\n== Per-run results ==")
    print(f"{'dir':>4} {'pwm':>5} {'a0 [m/s2]':>10} {'v0 [m/s]':>9} {'v_ss [m/s]':>10} {'r2':>6} {'tau [s]':>8}")
    for r in runs:
        print(
            f"{r['dir']:>+4d} {r['pwm']:>5d} {r['a0']:>10.3f} {r['v0']:>9.3f} "
            f"{r['vss']:>10.3f} {r['vss_r2']:>6.3f} {r['tau']:>8.3f}"
        )

    print("\n== Fits (per direction) ==")
    gains: dict[int, tuple[float, float]] = {}  # dir -> (G, b)
    for d in (+1, -1):
        rs = [r for r in runs if r["dir"] == d and r["a0"] != 0.0]
        if len(rs) < 2:
            print(f"dir {d:+d}: not enough runs with a valid acceleration fit")
            continue
        # m*a0 = G*pwm - b  ->  a0 = (G/m)*pwm - b/m
        slope, ic, r2 = linreg([r["pwm"] for r in rs], [r["a0"] for r in rs])
        g = mass * slope
        b = -mass * ic
        gains[d] = (g, b)
        # v_ss = (G*pwm - b)/c  ->  c = G / slope2
        slope2, _ic2, r22 = linreg([r["pwm"] for r in rs], [r["vss"] for r in rs])
        c_from_vss = g / slope2 if slope2 > 0 else 0.0
        # independent per-run damping estimate
        cs = [(g * r["pwm"] - b) / r["vss"] for r in rs if r["vss"] > 0.05]
        taus = [r["tau"] for r in rs if r["tau"] > 0]
        print(f"dir {d:+d}: G = {g:.4f} N/pwm-unit (accel fit r2={r2:.3f}), Coulomb b = {b:.2f} N")
        if cs and taus:
            print(
                f"        damping: c(v_ss fit) = {c_from_vss:.1f}, c(per-run) = {min(cs):.1f}..{max(cs):.1f}"
                f" N*s/m (v_ss fit r2={r22:.3f}), tau = {min(taus):.3f}..{max(taus):.3f} s"
            )
        else:
            print("        insufficient steady-state data for damping/tau")

    print("\n== Suggested sim parameters (vs current) ==")
    delays = [r["delay"] for r in runs if r.get("delay")]
    if delays:
        print(
            f"actuation latency: {min(delays) * 1000.0:.0f}..{max(delays) * 1000.0:.0f} ms "
            f"(serial + firmware poll + breakaway; feeds the sim delay model)"
        )
    if len(gains) == 2:
        gs = [abs(g) for g, _ in gains.values()]
        bs = [abs(b) for _, b in gains.values()]
        cs_all = []
        taus_all = []
        for d, (g, b) in gains.items():
            for r in runs:
                if r["dir"] == d and r["vss"] > 0.05 and r["tau"] > 0:
                    cs_all.append(abs(g * r["pwm"] - b) / r["vss"])
                    taus_all.append(r["tau"])
        scale = max(gs) * 1000.0
        print(f"scale        = {scale:.1f} N          (current: 10.0)")
        print(f"deadzone     = {max(bs):.2f} N          (current: 1.0)")
        if cs_all:
            print(f"damping_range= ({0.75 * min(cs_all):.1f}, {1.25 * max(cs_all):.1f})  (current: (5.0, 15.0))")
        if taus_all:
            print(f"tau_range    = ({0.8 * min(taus_all):.3f}, {1.25 * max(taus_all):.3f})   (current: (0.02, 0.04))")
        for d, st in stiction.items():
            if st is not None:
                print(f"stiction(dir {d:+d}): breaks away at PWM {st}  -> ~{abs(gains[d][0]) * st:.2f} N")
            else:
                print(f"stiction(dir {d:+d}): no breakaway up to PWM 420 - check the drive!")


# ---------------------------------------------------------------------------- main


def collect(serial_port: serial.Serial, args: argparse.Namespace) -> None:
    buf = bytearray()
    pwms = [int(p) for p in args.pwms.split(",")]
    runs: list[dict] = []
    stiction: dict[int, int | None] = {}

    input(
        "\nReady? The cart will sweep the full rail in both directions.\n"
        "Place the cart at the TRACK CENTER now (the script re-zeros here),\n"
        "pole hanging and settled. A sign check pulses the motor briefly.\n"
        "Press ENTER to start (Ctrl+C aborts, motor is stopped)..."
    )

    try:
        print("\n== Preflight: software zero + PWM/encoder sign check ==")
        offset = preflight(serial_port, buf)
        print(f"software zero: count {offset}; sign convention OK")

        print("\n== Probing usable travel range (drives to both ends) ==")
        pos_min, pos_max = probe_travel(serial_port, buf, offset)
        span = pos_max - pos_min
        print(
            f"usable travel: [{pos_min:.3f}, {pos_max:.3f}] m "
            f"(span {span * 1000.0:.0f} mm, center {(pos_min + pos_max) / 2.0:+.3f} m)"
        )
        if span < 0.10:
            print(
                f"[calib] WARNING: only {span * 1000.0:.0f} mm of travel - high PWM levels may cross it\n"
                "before reaching steady state; prefer lower --pwms values"
            )
        bounds = (pos_min + END_MARGIN_M, pos_max - END_MARGIN_M)

        print("\n== Stiction sweep (from track center each direction) ==")
        center = (pos_min + pos_max) / 2.0
        for d in (+1, -1):
            # sweep from the center: near an end the cart has no room to move the
            # detection distance and the sweep falsely reports no breakaway
            goto_pos(serial_port, buf, offset, center)
            stiction[d] = stiction_sweep(serial_port, buf, d)
            print(f"dir {d:+d}: breakaway PWM = {stiction[d]}")

        for d in (+1, -1):
            # start 18% of the span away from the end opposite to the run direction
            target = pos_min + START_FRAC * span if d == +1 else pos_max - START_FRAC * span
            # a PWM level at/below the breakaway threshold produces no motion: the fit
            # would silently return garbage and corrupt the regression - skip it
            gate = (stiction[d] or 0) + 20
            skipped = [p for p in pwms if p <= gate]
            if skipped:
                print(f"dir {d:+d}: skipping PWM {skipped} (at/below breakaway {stiction[d]} + 20)")
            for pwm in pwms:
                if pwm <= gate:
                    continue
                if goto_pos(serial_port, buf, offset, target) is None:
                    continue  # positioning failed (already reported); skip the run
                samples = run_constant_pwm(serial_port, buf, d * pwm, offset, bounds)
                if samples is None:
                    print(f"[calib] dir {d:+d} pwm {pwm}: no useful motion - discarded")
                    continue
                # normalize to positive displacement from rest for the first-order fit
                sign = 1.0 if samples[-1][1] >= samples[0][1] else -1.0
                norm = [(t, sign * (x - samples[0][1])) for t, x in samples]
                fit = fit_first_order(norm)
                if fit is None:
                    print(f"[calib] dir {d:+d} pwm {pwm}: no first-order fit - discarded")
                    continue
                delay_s, tau, vss_fit = fit
                a0 = vss_fit / tau
                # cross-checks on the detected motion segment
                seg = moving_segment(norm)
                if seg is not None:
                    segment, _ = seg
                    vss_x, r2 = steady_velocity(segment)
                    a0_quad, v0 = fit_initial_accel(segment)
                else:
                    vss_x, r2, a0_quad, v0 = 0.0, 0.0, 0.0, 0.0
                runs.append(
                    {
                        "dir": d,
                        "pwm": pwm,
                        "a0": a0,
                        "a0_quad": a0_quad,
                        "v0": v0,
                        "vss": vss_fit,
                        "vss_x": vss_x,
                        "vss_r2": r2,
                        "tau": tau,
                        "delay": delay_s,
                    }
                )
                print(
                    f"[calib] dir {d:+d} pwm {pwm}: a0={a0:.2f} (quad {a0_quad:.2f}) "
                    f"vss={vss_fit:.2f} (slope {vss_x:.2f}) tau={tau:.3f} delay={delay_s * 1000.0:.0f}ms"
                )
    finally:
        _stop_motor(serial_port)

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["dir", "pwm", "a0", "a0_quad", "v0", "vss", "vss_x", "vss_r2", "tau", "delay"])
        for r in runs:
            w.writerow(
                [
                    r["dir"],
                    r["pwm"],
                    r["a0"],
                    r["a0_quad"],
                    r["v0"],
                    r["vss"],
                    r["vss_x"],
                    r["vss_r2"],
                    r["tau"],
                    r["delay"],
                ]
            )
    print(f"\n[calib] summary written to {args.out}")
    analyze(runs, args.mass, stiction)


def reanalyze(path: str, mass: float) -> None:
    """Re-run the fits from a previously saved summary CSV (stiction unknown)."""
    with open(path, newline="", encoding="utf-8") as f:
        runs = [
            {
                "dir": int(r["dir"]),
                "pwm": int(r["pwm"]),
                "a0": float(r["a0"]),
                "v0": float(r["v0"]),
                "vss": float(r["vss"]),
                "vss_r2": float(r["vss_r2"]),
                "tau": float(r["tau"]),
                "delay": float(r.get("delay", 0.0)),
                "vss_x": 0.0,
            }
            for r in csv.DictReader(f)
        ]
    analyze(runs, mass, {})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM port of the STM32 (skip for --reanalyze)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mass", type=float, default=0.15, help="moving mass [kg]: cart only (pole hanging adds ~0)")
    ap.add_argument("--pwms", default="150,250,400,600,800", help="comma-separated PWM levels to test")
    ap.add_argument("--out", default="force_calib_raw.csv", help="output summary CSV path")
    ap.add_argument("--reanalyze", help="re-run fits from a saved summary CSV, no hardware needed")
    args = ap.parse_args()

    if args.reanalyze:
        reanalyze(args.reanalyze, args.mass)
        return
    if not args.port:
        sys.exit("either --port or --reanalyze is required")

    serial_port = serial.Serial(args.port, args.baud, timeout=0.05)
    serial_port.reset_input_buffer()
    try:
        collect(serial_port, args)
    finally:
        _stop_motor(serial_port)
        serial_port.close()
        print("[calib] done, motor off.")


if __name__ == "__main__":
    main()
