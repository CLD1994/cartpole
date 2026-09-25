#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PC 端策略闭环验证器（C610 上线前的实机快验）。

在 PC 上跑导出的 ONNX 策略，经串口直连 ESP32-S3 固件，完整复现
deploy/v2/edge（C610 端侧 C 程序）的控制环：同一套会话建立、门控、
观测组装与动作映射。验证的是「导出模型 + 单位/符号映射 + 协议闭环」
这条链；int8 量化误差属于 C610 侧的后续环节。

用法：
    py pc_runner.py --port COM6 --onnx C:\\path\\to\\policy.onnx
    py pc_runner.py --port COM6 --onnx policy.onnx --dry-run      # 不发 VEL，核对单位与符号
    py pc_runner.py --port COM6 --onnx policy.onnx --duration 30 # 30 s 后自动停

前置条件（COM 口独占，须先于本程序用 host 工具完成）：
    固件已标定并 ENABLE（MODE READY）。若在 FAULT：先 reset 再 enable；
    若在 DISABLED：mark-left -> 推右端 -> mark-right -> enable。

依赖：pyserial、onnxruntime（含 numpy）。ONNX 由训练机
scripts/rsl_rl/play.py 跑一次 v2 任务导出（checkpoint 同目录 exported/
policy.onnx，normalizer 已烘焙进图，输入顺序 [x, θ, ẋ, θ̇]）。

安全语义与 edge/main.c 逐条对齐；常量不在此处另立权威——默认值从
edge/config.h 与 edge/rig.c 的 CFG_FW_PARAMS 表解析（失败即拒绝启动），
CLI 可临时覆盖。固件侧另有看门狗、软限位、STEP_LOSS 兜底。

部署边界符号映射（2026-09-25 实机定案，详见 sim2real-spec）：训练侧摆角
正方向与实机相反（仅角量），观测送入模型前对 θ、θ̇ 取反，x、ẋ 与动作
不变。默认开启；--no-frame-fix 仅在训练侧改为实机同向约定并重训后使用。
"""

from __future__ import annotations

import argparse
import contextlib
import math
import re
import sys
import time
from pathlib import Path

import serial

BAUD = 460800
LINE_BUF_CAP = 512  # 无换行的残留字节超过此数视为链路垃圾，丢弃

EDGE_DIR = Path(__file__).resolve().parent / "edge"

# config.h 中参与本程序的数值宏（加载器按此清单校验完整性）
_CONFIG_KEYS = (
    "CONTROL_HZ",
    "STATE_TIMEOUT_S",
    "V_MAX_MPS",
    "TRACK_LIMIT_M",
    "ENGAGE_ANGLE_RAD",
)


# ================================================================
# 线协议解析（纯函数）
# ================================================================

def parse_state_line(line: str) -> tuple[int, int, int, int] | None:
    """STATE <x_um> <theta_urad> <vx_mm_s> <vth_mrad_s>；非法返回 None。"""
    parts = line.split()
    if len(parts) != 5 or parts[0] != "STATE":
        return None
    try:
        return (
            int(parts[1]),
            int(parts[2]),
            int(parts[3]),
            int(parts[4]),
        )
    except ValueError:
        return None


def parse_mode_line(line: str) -> tuple[str, str] | None:
    """MODE <mode> <reason>；非法返回 None。"""
    parts = line.split()
    if len(parts) != 3 or parts[0] != "MODE":
        return None
    return parts[1], parts[2]


def parse_reply_line(line: str):
    """OK <NAME> [value] / ERR <NAME>；非法返回 None。

    回执名单单 token（如 OK MARK_LEFT）与带值 CONFIG 回执（如
    OK VEL_MAX 1000）都是合法形态。
    """
    parts = line.split()
    if parts and parts[0] == "OK" and len(parts) in (2, 3):
        value = None
        if len(parts) == 3:
            try:
                value = int(parts[2])
            except ValueError:
                value = None
        return ("OK", parts[1], value)
    if len(parts) == 2 and parts[0] == "ERR":
        return ("ERR", parts[1], None)
    return None


class LineSplitter:
    """字节流切行：跨 feed 持久缓冲（半行不丢），\\r\\n 兼容。

    无换行残留超过 LINE_BUF_CAP 视为链路垃圾，整段丢弃——STATE 行
    约 50 字节且 5 ms 一 pump，健康链路永远不会触顶。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[str]:
        self._buf.extend(data)
        lines: list[str] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                break
            raw = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            text = raw.decode("utf-8", errors="ignore").strip()
            if text:
                lines.append(text)
        if len(self._buf) > LINE_BUF_CAP:
            self._buf.clear()
        return lines


# ================================================================
# 观测组装与驱动门控（纯函数，与 edge/main.c 对齐）
# ================================================================

def build_obs(x_um: int, theta_urad: int, vx_mm_s: int, vth_mrad_s: int):
    """STATE 整数遥测 -> SI 观测 [x m, θ rad, ẋ m/s, θ̇ rad/s]。"""
    return (
        x_um * 1e-6,
        theta_urad * 1e-6,
        vx_mm_s * 1e-3,
        vth_mrad_s * 1e-3,
    )


def to_policy_frame(obs):
    """实机观测 -> 策略坐标系：θ、θ̇ 取反，x、ẋ 与动作不变。

    训练侧摆角正方向与实机约定相反（摆顶向 −x 为正），线位置/线速度
    与动作符号一致。判别依据：桌面刚体仿真七候选映射实验中本映射唯一
    全存活（其余 ~1 s 内倒下），实机闭环已排除恒等映射。详见
    sim2real-spec「部署边界符号映射」。"""
    return (obs[0], -obs[1], obs[2], -obs[3])


def gate_reason(
    mode_known: bool,
    mode: str | None,
    state_age_s: float | None,
    th_rad: float,
    engage_rad: float,
    state_timeout_s: float,
) -> str | None:
    """驱动门控（次序与 edge/main.c 一致）：mode -> stale -> engage。

    返回 None 表示允许驱动；否则返回原因码 "mode"/"stale"/"engage"。
    state_age_s 为 None 表示从未收到 STATE；负值按 stale 处理。
    """
    if not mode_known or mode != "READY":
        return "mode"
    if state_age_s is None or state_age_s < 0.0 or state_age_s > state_timeout_s:
        return "stale"
    if abs(th_rad) > engage_rad:
        return "engage"
    return None


def lround_int(x: float) -> int:
    """C lround 语义：四舍五入、远离零。"""
    return int(math.floor(x + 0.5)) if x >= 0 else int(math.ceil(x - 0.5))


def map_action(action: float, x_m: float, v_max_mps: float, track_limit_m: float) -> int:
    """动作映射：钳位 ±1 -> ×V_MAX -> 轨道端头只封向外 -> 四舍五入整数 mm/s。

    非有限输入是策略侧致命错误，抛 ValueError（调用方走统一停机路径）。
    """
    if not math.isfinite(action):
        raise ValueError("policy action 非有限值")
    a = min(1.0, max(-1.0, action))
    v = a * v_max_mps * 1000.0
    if abs(x_m) > track_limit_m and (x_m > 0.0) == (v > 0.0):
        v = 0.0
    return lround_int(v)


# ================================================================
# edge 常量加载（单一权威：edge/config.h 与 edge/rig.c）
# ================================================================

def parse_config_h(text: str) -> dict[str, float]:
    """从 config.h 文本解析本程序用到的 CFG_ 数值宏；缺项即报错。"""
    found: dict[str, float] = {}
    for m in re.finditer(r"#define\s+CFG_(\w+)\s+(-?\d+(?:\.\d+)?)", text):
        if m.group(1) in _CONFIG_KEYS:
            found[m.group(1)] = float(m.group(2))
    missing = [k for k in _CONFIG_KEYS if k not in found]
    if missing:
        raise RuntimeError("config.h 解析缺失宏: %s" % ", ".join(missing))
    return found


def parse_fw_params(text: str) -> list[tuple[str, int]]:
    """从 rig.c 文本解析 CFG_FW_PARAMS 表（保序；表缺失即报错）。"""
    start = text.find("CFG_FW_PARAMS[")
    if start < 0:
        raise RuntimeError("rig.c 中未找到 CFG_FW_PARAMS 表")
    end = text.find("};", start)
    if end < 0:
        raise RuntimeError("rig.c 中 CFG_FW_PARAMS 表未闭合")
    table = text[start:end]
    params = re.findall(r'\{\s*"([A-Z_0-9]+)"\s*,\s*(-?\d+)\s*\}', table)
    if not params:
        raise RuntimeError("CFG_FW_PARAMS 表解析为空")
    return [(name, int(value)) for name, value in params]


def load_edge_defaults(edge_dir: Path) -> tuple[dict[str, float], list[tuple[str, int]]]:
    config_h = edge_dir / "config.h"
    rig_c = edge_dir / "rig.c"
    for path in (config_h, rig_c):
        if not path.is_file():
            raise RuntimeError("缺少权威常量来源 %s（edge 目录不完整？）" % path)
    cfg = parse_config_h(config_h.read_text(encoding="utf-8"))
    params = parse_fw_params(rig_c.read_text(encoding="utf-8"))
    return cfg, params


# ================================================================
# 串口会话（协议层的薄封装，实机集成层）
# ================================================================

class Rig:
    """ESP32-S3 线协议会话。所有读取经 pump() 非阻塞推进。"""

    def __init__(self, port: str) -> None:
        self._ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.0)
        self._splitter = LineSplitter()
        # 开串口对齐：只丢到首个 \n，其后字节进入切行器（A8 丢行教训）
        self._ser.reset_input_buffer()
        with self._timeout(0.5):
            self._ser.read_until(b"\n")
        self.last_state: tuple[int, int, int, int] | None = None
        self.state_t: float | None = None
        self.mode_known = False
        self.mode: str | None = None
        self.reason: str | None = None
        self.last_reply = None

    @contextlib.contextmanager
    def _timeout(self, seconds: float):
        old = self._ser.timeout
        self._ser.timeout = seconds
        try:
            yield
        finally:
            self._ser.timeout = old

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._ser.close()

    def pump(self) -> None:
        if not self._ser.is_open:
            return
        if self._ser.in_waiting:
            for line in self._splitter.feed(self._ser.read(self._ser.in_waiting)):
                self._handle(line)

    def _handle(self, line: str) -> None:
        state = parse_state_line(line)
        if state is not None:
            self.last_state = state
            self.state_t = time.perf_counter()
            return
        mode = parse_mode_line(line)
        if mode is not None:
            self.mode_known = True
            self.mode, self.reason = mode
            return
        reply = parse_reply_line(line)
        if reply is not None:
            self.last_reply = reply

    def send_line(self, text: str) -> None:
        self._ser.write((text + "\n").encode("ascii"))

    def send_vel(self, v_mm_s: float) -> None:
        if not math.isfinite(v_mm_s) or abs(v_mm_s) > 1.0e9:
            return  # 与固件侧 proto_format_vel 拒绝语义一致：不发送
        self.send_line("VEL %d" % lround_int(v_mm_s))

    def state_age_s(self) -> float | None:
        if self.state_t is None:
            return None
        return time.perf_counter() - self.state_t

    def sync_mode(self, timeout_s: float) -> bool:
        self.last_reply = None
        self.send_line("QUERY")
        t0 = time.perf_counter()
        while not self.mode_known and time.perf_counter() - t0 < timeout_s:
            self.pump()
            time.sleep(0.005)
        return self.mode_known

    def wait_state(self, timeout_s: float) -> bool:
        t0 = time.perf_counter()
        while self.last_state is None and time.perf_counter() - t0 < timeout_s:
            self.pump()
            time.sleep(0.002)
        return self.last_state is not None

    def push_params(self, params: list[tuple[str, int]]) -> list[str]:
        """逐条 CONFIG 推送（间隔 10 ms）；返回被拒（ERR）的参数名列表。"""
        rejected: list[str] = []
        for name, value in params:
            self.last_reply = None
            self.send_line("CONFIG %s %d" % (name, value))
            time.sleep(0.01)
            self.pump()
            if self.last_reply is not None and self.last_reply[0] == "ERR":
                rejected.append(name)
        return rejected


# ================================================================
# ONNX 策略（onnxruntime 惰性导入：--help 与单测不依赖该包）
# ================================================================

class PolicyOnnx:
    def __init__(self, model_path: str) -> None:
        try:
            import numpy as np
            import onnxruntime as ort
        except ImportError as exc:
            raise RuntimeError(
                "需要 onnxruntime（py -m pip install onnxruntime）"
            ) from exc
        path = Path(model_path)
        if not path.is_file():
            raise RuntimeError("模型文件不存在: %s" % path)
        self._np = np
        try:
            self._sess = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"]
            )
        except Exception as exc:
            if "External data path" in str(exc):
                raise RuntimeError(
                    "模型权重在伴生外部数据 %s.data 中，主文件只含图结构；"
                    "请把训练机 exported/ 目录内的 .data 伴生文件/目录连同"
                    " policy.onnx 一起拷贝并保持同名相邻" % path.name
                ) from exc
            raise RuntimeError("onnxruntime 加载失败: %s" % exc) from exc
        inputs = self._sess.get_inputs()
        outputs = self._sess.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError(
                "期望单输入单输出的策略图，实际 %d 输入 / %d 输出"
                % (len(inputs), len(outputs))
            )
        self._in_name = inputs[0].name
        self._out_name = outputs[0].name

    def forward(self, obs) -> float:
        x = self._np.asarray(obs, dtype=self._np.float32).reshape(1, -1)
        y = self._sess.run([self._out_name], {self._in_name: x})[0]
        action = float(self._np.asarray(y).flatten()[0])
        if not math.isfinite(action):
            raise ValueError("policy 输出非有限值（推理库契约：致命错误）")
        return action


# ================================================================
# 控制环与节拍
# ================================================================

@contextlib.contextmanager
def windows_high_res_timer():
    """Windows 定时器粒度 1 ms（默认 ~15.6 ms 无法支撑 5 ms 周期）。"""
    if sys.platform != "win32":
        yield
        return
    import ctypes

    winmm = ctypes.windll.winmm
    winmm.timeBeginPeriod(1)
    try:
        yield
    finally:
        winmm.timeEndPeriod(1)


def sleep_until(deadline: float) -> None:
    """睡到绝对时刻；末段自旋保证节拍精度。"""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return
        if remaining > 0.002:
            time.sleep(remaining - 0.0015)


def run_loop(rig: Rig, policy: PolicyOnnx, args) -> dict:
    period = 1.0 / args.hz
    next_t = time.perf_counter() + period
    t0 = time.perf_counter()
    stats = {
        "ticks": 0,
        "gate_mode": 0,
        "gate_stale": 0,
        "gate_engage": 0,
        "infer": 0,
        "x_min_m": None,
        "x_max_m": None,
        "th_min_rad": None,
        "th_max_rad": None,
        "v_min_mm_s": 0,
        "v_max_mm_s": 0,
    }
    print(
        "[pc] policy loop at %.0f Hz%s. Ctrl+C to stop."
        % (args.hz, " (dry-run: 不发 VEL)" if args.dry_run else "")
    )
    print("[pc] 部署边界符号映射(θ,θ̇ 取反): %s" % ("开" if args.frame_fix else "关"))
    last_mode, last_reason = rig.mode, rig.reason
    try:
        while True:
            rig.pump()
            if rig.mode_known and (rig.mode, rig.reason) != (last_mode, last_reason):
                print(
                    "[pc] 固件模式变化: %s %s -> %s %s"
                    % (last_mode, last_reason or "NONE", rig.mode, rig.reason or "NONE")
                )
                last_mode, last_reason = rig.mode, rig.reason
            tel = rig.last_state
            age = rig.state_age_s()
            reason = None
            if tel is not None:
                obs = build_obs(*tel)
                stats["x_min_m"] = (
                    obs[0] if stats["x_min_m"] is None else min(stats["x_min_m"], obs[0])
                )
                stats["x_max_m"] = (
                    obs[0] if stats["x_max_m"] is None else max(stats["x_max_m"], obs[0])
                )
                stats["th_min_rad"] = (
                    obs[1]
                    if stats["th_min_rad"] is None
                    else min(stats["th_min_rad"], obs[1])
                )
                stats["th_max_rad"] = (
                    obs[1]
                    if stats["th_max_rad"] is None
                    else max(stats["th_max_rad"], obs[1])
                )
                reason = gate_reason(
                    rig.mode_known,
                    rig.mode,
                    age,
                    obs[1],
                    args.engage_rad,
                    args.state_timeout_s,
                )
                if reason is not None:
                    stats["gate_" + reason] += 1

            v_mm_s = 0
            action = None
            if tel is not None:
                # 模型接口走策略坐标系（θ/θ̇ 取反），门控/统计/显示保持实机坐标系
                model_obs = to_policy_frame(obs) if args.frame_fix else obs
                action = policy.forward(model_obs)
                stats["infer"] += 1
                # dry-run 下计算「本应发送」的指令仅供核对显示；被门控
                # 拦截的正常模式置零。两种情况都不在这里发送。
                if args.dry_run or reason is None:
                    v_mm_s = map_action(
                        action, obs[0], args.v_max_mps, args.track_limit_m
                    )
                    stats["v_min_mm_s"] = min(stats["v_min_mm_s"], v_mm_s)
                    stats["v_max_mm_s"] = max(stats["v_max_mm_s"], v_mm_s)
            else:
                stats["gate_stale"] += 1

            if not args.dry_run:
                rig.send_vel(v_mm_s)

            stats["ticks"] += 1
            if stats["ticks"] % args.print_every == 0 and tel is not None:
                label = reason if reason is not None else ("dry" if args.dry_run else "drive")
                print(
                    "[pc] t=%5.1fs x=%+.3fm θ=%+6.2f° θ̇=%+5.2frad/s a=%+.3f v=%+4dmm/s (%s)"
                    % (
                        time.perf_counter() - t0,
                        obs[0],
                        math.degrees(obs[1]),
                        obs[3],
                        action if action is not None else 0.0,
                        v_mm_s,
                        label,
                    )
                )

            if args.duration is not None and time.perf_counter() - t0 >= args.duration:
                break
            sleep_until(next_t)
            next_t += period
            if time.perf_counter() > next_t:
                # 落拍过多则重锚，避免追赶式连发
                next_t = time.perf_counter() + period
    except KeyboardInterrupt:
        print("\n[pc] Ctrl+C，停机退出")
    stats["elapsed_s"] = time.perf_counter() - t0
    return stats


def print_stats(stats: dict) -> None:
    print(
        "[pc] 统计: %.1f s, %d 拍 (实际 %.1f Hz), 推理 %d 次" %
        (stats["elapsed_s"], stats["ticks"], stats["ticks"] / stats["elapsed_s"],
         stats["infer"])
    )
    print(
        "[pc] 门控拦截: mode=%d stale=%d engage=%d"
        % (stats["gate_mode"], stats["gate_stale"], stats["gate_engage"])
    )
    if stats["x_min_m"] is not None:
        print(
            "[pc] x ∈ [%.3f, %.3f] m, θ ∈ [%+.2f°, %+.2f°], v ∈ [%+d, %+d] mm/s"
            % (
                stats["x_min_m"],
                stats["x_max_m"],
                math.degrees(stats["th_min_rad"]),
                math.degrees(stats["th_max_rad"]),
                stats["v_min_mm_s"],
                stats["v_max_mm_s"],
            )
        )


# ================================================================
# CLI
# ================================================================

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="PC 端 ONNX 策略闭环验证器（镜像 deploy/v2/edge 控制环）"
    )
    parser.add_argument("--port", required=True, help="串口（如 COM6）")
    parser.add_argument(
        "--onnx", required=True, help="policy.onnx 路径（play.py 导出，任意位置）"
    )
    parser.add_argument("--hz", type=float, default=None, help="控制频率（默认取 edge 配置）")
    parser.add_argument("--v-max-mps", type=float, default=None)
    parser.add_argument("--engage-rad", type=float, default=None)
    parser.add_argument("--track-limit-m", type=float, default=None)
    parser.add_argument("--state-timeout-s", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None, help="运行秒数（默认直到 Ctrl+C）")
    parser.add_argument(
        "--dry-run", action="store_true", help="只组装观测跑推理打印动作，不发 VEL"
    )
    parser.add_argument(
        "--no-frame-fix",
        action="store_true",
        help="关闭部署边界符号映射（仅在训练侧已改为实机同向约定时使用）",
    )
    parser.add_argument("--print-every", type=int, default=40, help="状态行间隔拍数")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        cfg, params = load_edge_defaults(EDGE_DIR)
    except RuntimeError as exc:
        print("[pc] 常量加载失败: %s" % exc, file=sys.stderr)
        return 1
    args.hz = args.hz if args.hz is not None else cfg["CONTROL_HZ"]
    args.state_timeout_s = (
        args.state_timeout_s if args.state_timeout_s is not None
        else cfg["STATE_TIMEOUT_S"]
    )
    args.v_max_mps = (
        args.v_max_mps if args.v_max_mps is not None else cfg["V_MAX_MPS"]
    )
    args.track_limit_m = (
        args.track_limit_m if args.track_limit_m is not None else cfg["TRACK_LIMIT_M"]
    )
    args.engage_rad = (
        args.engage_rad if args.engage_rad is not None
        else cfg["ENGAGE_ANGLE_RAD"]
    )
    args.frame_fix = not args.no_frame_fix
    print(
        "[pc] 配置: %.0f Hz, V_MAX=%.2f m/s, engage=%.3f rad (%.1f°), "
        "track=±%.3f m, state_timeout=%.2f s, frame_fix=%s"
        % (
            args.hz,
            args.v_max_mps,
            args.engage_rad,
            math.degrees(args.engage_rad),
            args.track_limit_m,
            args.state_timeout_s,
            "开" if args.frame_fix else "关",
        )
    )
    print("[pc] 会话参数推送集（%d 项，来自 edge/rig.c）" % len(params))

    try:
        policy = PolicyOnnx(args.onnx)
    except RuntimeError as exc:
        print("[pc] 模型加载失败: %s" % exc, file=sys.stderr)
        return 1

    try:
        rig = Rig(args.port)
    except serial.SerialException as exc:
        print("[pc] 串口打开失败: %s" % exc, file=sys.stderr)
        return 1

    exit_code = 0
    stats: dict = {}
    try:
        if not rig.sync_mode(1.0):
            print("[pc] QUERY 无应答（固件未运行？）", file=sys.stderr)
            return 1
        print("[pc] 固件模式: %s %s" % (rig.mode, rig.reason or ""))

        rejected = rig.push_params(params)
        if rejected:
            print("[pc] 参数被拒: %s" % ", ".join(rejected), file=sys.stderr)

        if rig.mode != "READY" and not args.dry_run:
            print(
                "[pc] 固件在 %s，未就绪。先完成（host 工具，COM 口独占须在本程序外）：\n"
                "     FAULT -> py tools/cartpole_host.py --port %s reset 再 enable；\n"
                "     DISABLED -> mark-left -> 推右端 -> mark-right -> enable"
                % (rig.mode, args.port),
                file=sys.stderr,
            )
            return 1
        if not rig.wait_state(args.state_timeout_s if not args.dry_run else 2.0):
            print("[pc] 无 STATE 流", file=sys.stderr)
            return 1

        with windows_high_res_timer():
            stats = run_loop(rig, policy, args)
    except (ValueError, RuntimeError) as exc:
        print("[pc] 致命错误（已停机）: %s" % exc, file=sys.stderr)
        exit_code = 1
    finally:
        if not args.dry_run:
            for _ in range(3):
                with contextlib.suppress(Exception):
                    rig.send_vel(0.0)
                    time.sleep(0.02)
        rig.close()
        if stats:
            print_stats(stats)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
