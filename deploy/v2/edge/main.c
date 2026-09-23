/* 端侧控制程序（新装置 v2）：200 Hz 读遥测 -> 组观测 -> NPU 推理 -> 发速度命令。
 *
 * 职责与安全语义（与 deploy/v2/protocol.md、sim2real-spec.md 对齐）：
 *   - 驱动门控：仅当最后已知固件模式为 READY 且 STATE 流新鲜（< CFG_STATE_TIMEOUT_S）时驱动；
 *   - 摆杆门限：|theta| 超过 CFG_ENGAGE_ANGLE_RAD 时不驱动（人手扶到直立附近才接管）；
 *   - 轨道端头：|x| 超过 CFG_TRACK_LIMIT_M 时只封锁向外的速度，保留回程；
 *   - 退出路径统一：SIGINT/SIGTERM/致命错误 -> VEL 0 -> 释放资源。
 *   固件侧另有独立的看门狗与软限位兜底（见协议）。
 */

#include "config.h"
#include "policy.h"
#include "proto.h"
#include "uart.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

const struct fw_param CFG_FW_PARAMS[CFG_FW_PARAM_COUNT] = {
    {"VEL_MAX", 1000},
    {"ACC_MAX", 5000},
    {"WDT_MS", 100},
    {"WDT_DEC", 5000},
    {"HOME_VEL", 50},
    {"HOME_TIMEOUT", 30000},
    {"X_LIMIT", 260000},
};

/* ---- 运行状态 ---- */
static volatile sig_atomic_t g_stop = 0;
static uart_t g_uart;
static proto_rx_t g_rx;

/* 遥测快照（STATE 行更新，主循环消费最新值） */
static int64_t g_x_um, g_theta_urad, g_vx_mm_s, g_vth_mrad_s;
static double g_state_t = -1.0; /* 最近 STATE 时刻 [s, monotonic] */

/* 控制面：最后已知固件模式（MODE 事件与 QUERY 应答维护） */
static fw_mode_t g_mode = FW_DISABLED;
static bool g_mode_known = false;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void sleep_to(double next)
{
    struct timespec ts;
    ts.tv_sec = (time_t)next;
    ts.tv_nsec = (long)((next - (double)ts.tv_sec) * 1e9);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int send_line(const char *buf, size_t n)
{
    return uart_write_all(&g_uart, (const uint8_t *)buf, n);
}

static int send_vel(double v_mm_s)
{
    char buf[64];
    size_t n = proto_format_vel(buf, v_mm_s);
    return n == 0 ? -1 : send_line(buf, n);
}

static int send_simple(const char *cmd)
{
    char buf[64];
    size_t n = proto_format_simple(buf, cmd);
    return n == 0 ? -1 : send_line(buf, n);
}

/* ---- 行分发 ---- */
static void handle_line(const char *line, size_t len, void *user)
{
    (void)len;
    (void)user;
    proto_msg_t m;
    if (proto_parse_line(line, &m) == PROTO_UNKNOWN) {
        return; /* 非法行静默丢弃 */
    }
    switch (m.type) {
    case PROTO_STATE:
        g_x_um = m.x_um;
        g_theta_urad = m.theta_urad;
        g_vx_mm_s = m.vx_mm_s;
        g_vth_mrad_s = m.vth_mrad_s;
        g_state_t = now_s();
        break;
    case PROTO_MODE: {
        fw_mode_t prev = g_mode;
        g_mode = m.mode;
        g_mode_known = true;
        if (m.mode != prev || m.mode == FW_FAULT) {
            printf("[edge] MODE %s %s\n", proto_mode_name(m.mode), proto_reason_name(m.reason));
        }
        break;
    }
    case PROTO_OK:
        printf("[edge] config OK %s = %ld\n", m.param, (long)m.value);
        break;
    case PROTO_ERR:
        printf("[edge] config ERR %s (state rejects CONFIG?)\n", m.param);
        break;
    default:
        break;
    }
}

static void pump_serial(void)
{
    uint8_t chunk[512];
    for (;;) {
        int n = uart_read(&g_uart, chunk, sizeof(chunk));
        if (n <= 0) {
            return;
        }
        proto_rx_feed(&g_rx, chunk, (size_t)n, handle_line, NULL);
    }
}

/* ---- 启动流程 ---- */

/* 等 MODE 已知（发 QUERY 后）。成功返回 0。 */
static int sync_mode(double timeout_s)
{
    char buf[64];
    size_t n = proto_format_simple(buf, "QUERY");
    send_line(buf, n);
    double t0 = now_s();
    while (!g_mode_known && now_s() - t0 < timeout_s) {
        pump_serial();
        struct timespec ts = {0, 5 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    return g_mode_known ? 0 : -1;
}

static int push_config(void)
{
    char buf[64];
    for (int i = 0; i < CFG_FW_PARAM_COUNT; i++) {
        size_t n = proto_format_config(buf, CFG_FW_PARAMS[i].name, CFG_FW_PARAMS[i].value);
        if (send_line(buf, n) != 0) {
            return -1;
        }
        struct timespec ts = {0, 10 * 1000 * 1000};
        nanosleep(&ts, NULL); /* 固件逐条回执，间隔 10 ms 防淹没 */
        pump_serial();
    }
    return 0;
}

/* 发 HOME 并等 READY；超时返回 -1。 */
static int wait_ready(double timeout_s)
{
    if (send_simple("HOME") != 0) {
        return -1;
    }
    double t0 = now_s();
    while (!g_stop && now_s() - t0 < timeout_s) {
        pump_serial();
        if (g_mode_known && g_mode == FW_READY) {
            return 0;
        }
        if (g_mode_known && g_mode == FW_FAULT) {
            return -1;
        }
        struct timespec ts = {0, 5 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    return -1;
}

/* 回零完成后把小车从中点移回行程中点（协议操作流程第 5 步）。 */
static int goto_center(double timeout_s)
{
    double t0 = now_s();
    int warned = 0;
    while (!g_stop && now_s() - t0 < timeout_s) {
        pump_serial();
        if (g_state_t < 0.0 || now_s() - g_state_t > CFG_STATE_TIMEOUT_S) {
            send_vel(0.0);
            continue;
        }
        if (g_mode != FW_READY) {
            return -1;
        }
        double x_m = (double)g_x_um * 1e-6;
        if (fabs(x_m) < 0.005) {
            send_vel(0.0);
            return 0;
        }
        double v = (x_m > 0.0 ? -1.0 : 1.0) * CFG_CENTER_VEL_MMS;
        if (fabs(x_m) > CFG_TRACK_LIMIT_M && !warned) {
            warned = 1;
            printf("[edge] WARNING: cart beyond soft limit (%.3f m), moving inward\n", x_m);
        }
        send_vel(v);
        struct timespec ts = {0, 5 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }
    send_vel(0.0);
    return -1;
}

/* ---- 控制环 ----
 * 返回 0：SIGINT/SIGTERM 正常退出；返回 -1：致命错误（推理句柄失效），
 * 调用方走统一清理路径（VEL 0 -> 释放资源）。 */
static int control_loop(void)
{
    const double period = 1.0 / CFG_CONTROL_HZ;
    double next = now_s() + period;
    int state_stale_logged = 0;
    int limit_logged = 0;

    printf("[edge] policy loop at %.0f Hz. Ctrl+C to stop.\n", CFG_CONTROL_HZ);
    while (!g_stop) {
        pump_serial();

        double t = now_s();
        double x_m = (double)g_x_um * 1e-6;
        double th_rad = (double)g_theta_urad * 1e-6;

        /* 门控 1：固件必须处于 READY（否则固件本就不执行 VEL，这里只是不发并提示一次） */
        if (!g_mode_known || g_mode != FW_READY) {
            state_stale_logged = 0;
            sleep_to(next);
            next += period;
            continue;
        }
        /* 门控 2：STATE 流必须新鲜 */
        if (g_state_t < 0.0 || t - g_state_t > CFG_STATE_TIMEOUT_S) {
            send_vel(0.0);
            if (!state_stale_logged) {
                state_stale_logged = 1;
                printf("[edge] WARNING: STATE stream stale, motor stopped\n");
            }
            sleep_to(next);
            next += period;
            continue;
        }
        state_stale_logged = 0;

        /* 门控 3：摆杆在接管角以内（人手扶到直立附近），否则不驱动 */
        if (fabs(th_rad) > CFG_ENGAGE_ANGLE_RAD) {
            send_vel(0.0);
            sleep_to(next);
            next += period;
            continue;
        }

        /* 观测（顺序/单位与训练一致）与推理 */
        float obs[4] = {
            (float)x_m,
            (float)th_rad,
            (float)((double)g_vx_mm_s * 1e-3),
            (float)((double)g_vth_mrad_s * 1e-3),
        };
        float action;
        if (policy_forward(obs, &action) != 0) {
            /* 库契约：句柄已失效，不得继续推理；停机走统一退出路径 */
            fprintf(stderr, "[edge] policy inference failed, stopping\n");
            return -1;
        }
        if (action > 1.0f) {
            action = 1.0f;
        }
        if (action < -1.0f) {
            action = -1.0f;
        }
        double v_mm_s = (double)action * CFG_V_MAX_MPS * 1000.0;

        /* 轨道端头：只封向外，保留向内纠偏 */
        if (fabs(x_m) > CFG_TRACK_LIMIT_M && (x_m > 0.0) == (v_mm_s > 0.0)) {
            v_mm_s = 0.0;
            if (!limit_logged) {
                limit_logged = 1;
                printf("[edge] WARNING: cart at track end (%.3f m), blocking outward motion\n", x_m);
            }
        } else {
            limit_logged = 0;
        }

        send_vel(v_mm_s);
        sleep_to(next);
        next += period;
    }
    return 0;
}

static void print_usage(const char *prog)
{
    fprintf(stderr, "usage: %s [--port /dev/ttyXXXX] [--model path.om] [--no-home]\n", prog);
}

int main(int argc, char **argv)
{
    const char *port = CFG_SERIAL_PORT;
    const char *model = "policy.om";
    int do_home = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = argv[++i];
        } else if (strcmp(argv[i], "--model") == 0 && i + 1 < argc) {
            model = argv[++i];
        } else if (strcmp(argv[i], "--no-home") == 0) {
            do_home = 0;
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (uart_open(&g_uart, port, CFG_BAUD) != 0) {
        fprintf(stderr, "[edge] cannot open %s\n", port);
        return 1;
    }
    proto_rx_init(&g_rx);

    int rc = 1;
    if (policy_init(model) != 0) {
        fprintf(stderr, "[edge] policy init failed\n");
        goto cleanup;
    }
    if (sync_mode(1.0) != 0) {
        fprintf(stderr, "[edge] no MODE reply (QUERY timed out) - firmware not running?\n");
        goto cleanup;
    }
    printf("[edge] firmware mode: %s\n", proto_mode_name(g_mode));

    if (push_config() != 0) {
        fprintf(stderr, "[edge] config push failed (serial write error)\n");
        goto cleanup;
    }

    if (g_mode == FW_DISABLED && do_home) {
        printf("[edge] homing...\n");
        if (wait_ready(35.0) != 0) {
            fprintf(stderr, "[edge] homing failed (see MODE events above)\n");
            goto cleanup;
        }
        if (goto_center(15.0) != 0) {
            fprintf(stderr, "[edge] move-to-center failed\n");
            goto cleanup;
        }
        printf("[edge] ready at track center; lift the pole near upright to engage the policy\n");
    } else if (g_mode != FW_READY) {
        fprintf(stderr, "[edge] firmware in %s; start from DISABLED (power-cycle or RESET) or pass --no-home\n",
                proto_mode_name(g_mode));
        goto cleanup;
    }

    rc = control_loop() == 0 ? 0 : 1;

cleanup:
    if (g_uart.fd >= 0) {
        send_vel(0.0); /* 统一退出路径：停机在先 */
    }
    policy_deinit();
    uart_close(&g_uart);
    printf("[edge] stopped, motor off.\n");
    return rc;
}
