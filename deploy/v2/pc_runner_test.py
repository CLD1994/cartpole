# -*- coding: utf-8 -*-
"""pc_runner 纯逻辑单元测试（标准库 unittest，不触碰串口与真实文件）。

被测对象是 deploy/v2/pc_runner.py 中具有独立行为契约的函数与类：
行解析、字节流切行、观测组装、驱动门控决策表、动作映射与端头封锁、
edge 常量加载器。串口会话与控制环属于实机集成，不在本层覆盖。
"""

import math
import unittest

import pc_runner


class ParseStateLineTest(unittest.TestCase):
    def test_valid_with_negatives(self):
        self.assertEqual(
            pc_runner.parse_state_line("STATE -422968 3141593 -12 4567"),
            (-422968, 3141593, -12, 4567),
        )

    def test_reject_wrong_token(self):
        self.assertIsNone(pc_runner.parse_state_line("MODE READY NONE"))

    def test_reject_wrong_field_count(self):
        self.assertIsNone(pc_runner.parse_state_line("STATE 1 2 3"))
        self.assertIsNone(pc_runner.parse_state_line("STATE 1 2 3 4 5"))

    def test_reject_non_integer(self):
        self.assertIsNone(pc_runner.parse_state_line("STATE 1 2 x 4"))

    def test_empty(self):
        self.assertIsNone(pc_runner.parse_state_line(""))


class ParseModeLineTest(unittest.TestCase):
    def test_ready_none(self):
        self.assertEqual(
            pc_runner.parse_mode_line("MODE READY NONE"), ("READY", "NONE")
        )

    def test_fault_with_reason(self):
        self.assertEqual(
            pc_runner.parse_mode_line("MODE FAULT STEP_LOSS"),
            ("FAULT", "STEP_LOSS"),
        )

    def test_reject_wrong_arity(self):
        self.assertIsNone(pc_runner.parse_mode_line("MODE READY"))
        self.assertIsNone(pc_runner.parse_mode_line("MODE READY NONE EXTRA"))

    def test_reject_wrong_token(self):
        self.assertIsNone(pc_runner.parse_mode_line("STATE 1 2 3 4"))


class ParseReplyLineTest(unittest.TestCase):
    def test_ok_with_value(self):
        self.assertEqual(
            pc_runner.parse_reply_line("OK VEL_MAX 1000"),
            ("OK", "VEL_MAX", 1000),
        )

    def test_ok_bare_name(self):
        self.assertEqual(
            pc_runner.parse_reply_line("OK MARK_LEFT"), ("OK", "MARK_LEFT", None)
        )

    def test_err(self):
        self.assertEqual(
            pc_runner.parse_reply_line("ERR ENABLE"), ("ERR", "ENABLE", None)
        )

    def test_reject_junk(self):
        self.assertIsNone(pc_runner.parse_reply_line("OK"))
        self.assertIsNone(pc_runner.parse_reply_line("BSomething"))


class LineSplitterTest(unittest.TestCase):
    def test_line_split_across_feeds(self):
        sp = pc_runner.LineSplitter()
        self.assertEqual(sp.feed(b"STATE 1 2"), [])
        self.assertEqual(sp.feed(b" 3 4\n"), ["STATE 1 2 3 4"])

    def test_multiple_lines_in_one_feed(self):
        sp = pc_runner.LineSplitter()
        self.assertEqual(sp.feed(b"A\nB\n"), ["A", "B"])

    def test_crlf_stripped(self):
        sp = pc_runner.LineSplitter()
        self.assertEqual(sp.feed(b"x\r\ny\n"), ["x", "y"])

    def test_garbage_without_newline_is_capped(self):
        sp = pc_runner.LineSplitter()
        sp.feed(b"\xff" * 600)  # 超过缓冲上限且无换行：丢弃
        self.assertEqual(sp.feed(b"GOOD 1 2 3 4\n"), ["GOOD 1 2 3 4"])


class BuildObsTest(unittest.TestCase):
    def test_unit_conversion(self):
        obs = pc_runner.build_obs(-422968, 3141593, -123, 4567)
        self.assertAlmostEqual(obs[0], -0.422968, places=12)
        self.assertAlmostEqual(obs[1], 3.141593, places=12)
        self.assertAlmostEqual(obs[2], -0.123, places=12)
        self.assertAlmostEqual(obs[3], 4.567, places=12)

    def test_zeros(self):
        self.assertEqual(pc_runner.build_obs(0, 0, 0, 0), (0.0, 0.0, 0.0, 0.0))


class GateReasonTest(unittest.TestCase):
    """门控决策表：次序 mode -> stale -> engage，命中即返回原因码。"""

    ENGAGE = 0.524
    TIMEOUT = 0.2

    def reason(self, mode_known, mode, state_age, th_rad):
        return pc_runner.gate_reason(
            mode_known, mode, state_age, th_rad, self.ENGAGE, self.TIMEOUT
        )

    def test_drive_allowed(self):
        self.assertIsNone(
            self.reason(True, "READY", 0.005, 0.1)
        )

    def test_mode_unknown_or_not_ready(self):
        self.assertEqual(self.reason(False, None, 0.005, 0.1), "mode")
        self.assertEqual(self.reason(True, "DISABLED", 0.005, 0.1), "mode")
        self.assertEqual(self.reason(True, "FAULT", 0.005, 0.1), "mode")

    def test_state_stale_or_missing(self):
        self.assertEqual(self.reason(True, "READY", None, 0.1), "stale")
        self.assertEqual(self.reason(True, "READY", -0.001, 0.1), "stale")
        self.assertEqual(self.reason(True, "READY", 0.201, 0.1), "stale")

    def test_engage_angle_strictly_greater(self):
        self.assertEqual(self.reason(True, "READY", 0.005, 0.525), "engage")
        self.assertEqual(self.reason(True, "READY", 0.005, -0.525), "engage")
        self.assertIsNone(self.reason(True, "READY", 0.005, 0.524))

    def test_mode_takes_precedence_over_stale(self):
        self.assertEqual(self.reason(True, "DISABLED", 5.0, 0.1), "mode")

    def test_stale_takes_precedence_over_engage(self):
        self.assertEqual(self.reason(True, "READY", 5.0, 2.0), "stale")


class MapActionTest(unittest.TestCase):
    """动作映射：钳位 -> 速度尺度 -> 轨道端头只封向外，输出四舍五入整数 mm/s。"""

    V_MAX = 0.5
    TRACK = 0.260

    def mapped(self, action, x_m):
        return pc_runner.map_action(action, x_m, self.V_MAX, self.TRACK)

    def test_clip_then_scale(self):
        self.assertEqual(self.mapped(1.0, 0.0), 500)
        self.assertEqual(self.mapped(1.7, 0.0), 500)
        self.assertEqual(self.mapped(-2.0, 0.0), -500)
        self.assertEqual(self.mapped(0.5, 0.0), 250)
        self.assertEqual(self.mapped(0.0, 0.0), 0)

    def test_rounding_matches_lround(self):
        # 0.4992 * 500 = 249.6 -> 250（C 侧 lround 语义）
        self.assertEqual(self.mapped(0.4992, 0.0), 250)
        self.assertEqual(self.mapped(-0.4992, 0.0), -250)

    def test_track_end_blocks_outward_only(self):
        self.assertEqual(self.mapped(1.0, 0.270), 0)     # 右端外 + 向外
        self.assertEqual(self.mapped(-1.0, 0.270), -500)  # 右端外 + 向内放行
        self.assertEqual(self.mapped(-1.0, -0.270), 0)    # 左端外 + 向外
        self.assertEqual(self.mapped(1.0, -0.270), 500)   # 左端外 + 向内放行

    def test_inside_track_not_blocked(self):
        self.assertEqual(self.mapped(1.0, 0.250), 500)

    def test_nonfinite_is_fatal(self):
        with self.assertRaises(ValueError):
            self.mapped(math.nan, 0.0)
        with self.assertRaises(ValueError):
            self.mapped(math.inf, 0.0)


_CONFIG_H_FIXTURE = """\
/* 端侧控制程序常量（新装置 v2）。*/
#ifndef EDGE_CONFIG_H
#define EDGE_CONFIG_H

#define CFG_SERIAL_PORT     "/dev/ttyAMA1"
#define CFG_BAUD            460800

#define CFG_CONTROL_HZ      200.0
#define CFG_STATE_TIMEOUT_S 0.2

#define CFG_V_MAX_MPS       0.5       /* 训练随机化上界 */
#define CFG_TRACK_LIMIT_M   0.260
#define CFG_ENGAGE_ANGLE_RAD 0.524    /* 策略接管门限 */

struct fw_param {
    const char *name;
    long value;
};

#define CFG_FW_PARAM_COUNT  6
extern const struct fw_param CFG_FW_PARAMS[CFG_FW_PARAM_COUNT];

#endif /* EDGE_CONFIG_H */
"""

_RIG_C_FIXTURE = """\
#include "config.h"

const struct fw_param CFG_FW_PARAMS[CFG_FW_PARAM_COUNT] = {
    {"VEL_MAX", 1000},
    {"ACC_MAX", 5000},
    {"WDT_MS", 100},
    {"WDT_DEC", 5000},
    /* 台位标定量（上板验收：θ₀ 由下垂读数+180° 反推） */
    {"POLE_ZERO_URAD", 1655229},
    {"POLE_INV", 1},
};

static volatile sig_atomic_t g_stop;
"""


class FrameFixTest(unittest.TestCase):
    """部署边界符号映射：θ、θ̇ 取反，x、ẋ 与动作不变
    （桌面刚体仿真判别实验定案，见 sim2real-spec 符号映射）。"""

    def test_theta_channels_negated_others_kept(self):
        obs = (0.123, -3.14, 0.5, -1.2)
        self.assertEqual(
            pc_runner.to_policy_frame(obs), (0.123, 3.14, 0.5, 1.2)
        )

    def test_involution(self):
        obs = (1.0, -2.0, 3.0, -4.0)
        self.assertEqual(
            pc_runner.to_policy_frame(pc_runner.to_policy_frame(obs)), obs
        )


class EdgeConstantLoaderTest(unittest.TestCase):
    """加载器输入是文本；对真实文件的解析由运行时自校验，不属单元层。"""

    def test_parse_config_h(self):
        cfg = pc_runner.parse_config_h(_CONFIG_H_FIXTURE)
        self.assertEqual(cfg["CONTROL_HZ"], 200.0)
        self.assertAlmostEqual(cfg["STATE_TIMEOUT_S"], 0.2)
        self.assertAlmostEqual(cfg["V_MAX_MPS"], 0.5)
        self.assertAlmostEqual(cfg["TRACK_LIMIT_M"], 0.260)
        self.assertAlmostEqual(cfg["ENGAGE_ANGLE_RAD"], 0.524)
        # 只取 CFG_ 数值宏，字符串宏与其他定义不进入结果
        self.assertNotIn("SERIAL_PORT", cfg)
        self.assertNotIn("BAUD", cfg)

    def test_parse_config_h_missing_key_is_error(self):
        text = "#define CFG_CONTROL_HZ 200.0\n"
        with self.assertRaisesRegex(RuntimeError, "CONTROL_HZ|缺失|missing"):
            pc_runner.parse_config_h(text)

    def test_parse_fw_params_ordered(self):
        params = pc_runner.parse_fw_params(_RIG_C_FIXTURE)
        self.assertEqual(
            params,
            [
                ("VEL_MAX", 1000),
                ("ACC_MAX", 5000),
                ("WDT_MS", 100),
                ("WDT_DEC", 5000),
                ("POLE_ZERO_URAD", 1655229),
                ("POLE_INV", 1),
            ],
        )

    def test_parse_fw_params_negative_value(self):
        text = (
            "const struct fw_param CFG_FW_PARAMS[1] = {\n"
            '    {"X_LIMIT_UM", -3000},\n'
            "};\n"
        )
        self.assertEqual(pc_runner.parse_fw_params(text), [("X_LIMIT_UM", -3000)])

    def test_parse_fw_params_no_table_is_error(self):
        with self.assertRaisesRegex(RuntimeError, "CFG_FW_PARAMS"):
            pc_runner.parse_fw_params("int main(void) { return 0; }\n")


if __name__ == "__main__":
    unittest.main()
