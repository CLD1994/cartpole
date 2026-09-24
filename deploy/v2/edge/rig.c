/* 台位会话实现：行为从 main.c 原地迁入，语义不变。
 * 协议契约见 deploy/v2/protocol.md。
 */

#include "rig.h"

#include <signal.h>
#include <stdio.h>
#include <time.h>

#include "uart.h"

const struct fw_param CFG_FW_PARAMS[CFG_FW_PARAM_COUNT] = {
    {"VEL_MAX", 1000},
    {"ACC_MAX", 5000},
    {"WDT_MS", 100},
    {"WDT_DEC", 5000},
    {"HOME_VEL", 50},
    {"HOME_TIMEOUT", 30000},
    {"X_LIMIT", 260000},
};

/* ---- 会话状态 ---- */
static volatile sig_atomic_t g_stop;
static uart_t g_uart;
static proto_rx_t g_rx;

/* 遥测快照（STATE 行更新，各形态消费最新值） */
static int64_t g_x_um, g_theta_urad, g_vx_mm_s, g_vth_mrad_s;
static double g_state_t = -1.0; /* 最近 STATE 时刻 [s, monotonic] */

/* 控制面：最后已知固件模式（MODE 事件与 QUERY 应答维护） */
static fw_mode_t g_mode = FW_DISABLED;
static fw_reason_t g_mode_reason = FW_REASON_NONE;
static int g_mode_known;

/* 最近一条 OK/ERR 应答（CONFIG 回执等待用） */
static int g_reply_err;
static unsigned g_reply_seq;

double rig_now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

void rig_request_stop(void)
{
    g_stop = 1;
}

int rig_stop_requested(void)
{
    return g_stop != 0;
}

static void nap_ms(int ms)
{
    struct timespec ts = {0, (long)ms * 1000 * 1000};
    nanosleep(&ts, NULL);
}

static int send_line(const char *buf, size_t n)
{
    return uart_write_all(&g_uart, (const uint8_t *)buf, n);
}

int rig_send_vel(double v_mm_s)
{
    char buf[64];
    size_t n = proto_format_vel(buf, v_mm_s);
    return n == 0 ? -1 : send_line(buf, n);
}

int rig_send_simple(const char *cmd)
{
    char buf[64];
    size_t n = proto_format_simple(buf, cmd);
    return n == 0 ? -1 : send_line(buf, n);
}

int rig_send_config(const char *name, long value)
{
    char buf[64];
    size_t n = proto_format_config(buf, name, value);
    return n == 0 ? -1 : send_line(buf, n);
}

int rig_open(const char *port, int baud)
{
    if (uart_open(&g_uart, port, baud) != 0) {
        return -1;
    }
    proto_rx_init(&g_rx);
    return 0;
}

void rig_close(void)
{
    uart_close(&g_uart);
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
        g_state_t = rig_now_s();
        break;
    case PROTO_MODE: {
        fw_mode_t prev = g_mode;
        g_mode = m.mode;
        g_mode_reason = m.reason;
        g_mode_known = 1;
        if (m.mode != prev || m.mode == FW_FAULT) {
            printf("[edge] MODE %s %s\n", proto_mode_name(m.mode), proto_reason_name(m.reason));
        }
        break;
    }
    case PROTO_OK:
        g_reply_err = 0;
        g_reply_seq++;
        printf("[edge] config OK %s = %ld\n", m.param, (long)m.value);
        break;
    case PROTO_ERR:
        g_reply_err = 1;
        g_reply_seq++;
        printf("[edge] config ERR %s (state rejects CONFIG?)\n", m.param);
        break;
    default:
        break;
    }
}

void rig_pump(void)
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

/* ---- 等待与运动流程 ---- */

int rig_sync_mode(double timeout_s)
{
    if (rig_send_simple("QUERY") != 0) {
        return -1;
    }
    double t0 = rig_now_s();
    while (!g_mode_known && rig_now_s() - t0 < timeout_s) {
        rig_pump();
        nap_ms(5);
    }
    return g_mode_known ? 0 : -1;
}

int rig_wait_state(double timeout_s)
{
    double t0 = rig_now_s();
    while (g_state_t < 0.0 && rig_now_s() - t0 < timeout_s) {
        rig_pump();
        nap_ms(2);
    }
    return g_state_t >= 0.0 ? 0 : -1;
}

int rig_push_params(void)
{
    char buf[64];
    for (int i = 0; i < CFG_FW_PARAM_COUNT; i++) {
        size_t n = proto_format_config(buf, CFG_FW_PARAMS[i].name, CFG_FW_PARAMS[i].value);
        if (send_line(buf, n) != 0) {
            return -1;
        }
        nap_ms(10); /* 固件逐条回执，间隔 10 ms 防淹没 */
        rig_pump();
    }
    return 0;
}

int rig_wait_mode(fw_mode_t target, double timeout_s)
{
    double t0 = rig_now_s();
    while (!g_stop && rig_now_s() - t0 < timeout_s) {
        rig_pump();
        if (g_mode_known) {
            if (g_mode == target) {
                return 0;
            }
            if (g_mode == FW_FAULT && target != FW_FAULT) {
                return -1; /* 等待目标模式期间故障 */
            }
        }
        nap_ms(5);
    }
    return -1;
}

int rig_wait_ready(double timeout_s)
{
    if (rig_send_simple("HOME") != 0) {
        return -1;
    }
    return rig_wait_mode(FW_READY, timeout_s);
}

/* 回零完成后把小车从中点移回行程中点（协议操作流程第 5 步）。 */
int rig_goto_center(double timeout_s)
{
    double t0 = rig_now_s();
    int warned = 0;
    while (!g_stop && rig_now_s() - t0 < timeout_s) {
        rig_pump();
        if (g_state_t < 0.0 || rig_now_s() - g_state_t > CFG_STATE_TIMEOUT_S) {
            rig_send_vel(0.0);
            continue;
        }
        if (!g_mode_known || g_mode != FW_READY) {
            return -1;
        }
        double x_m = (double)g_x_um * 1e-6;
        if (x_m < 0.005 && x_m > -0.005) {
            rig_send_vel(0.0);
            return 0;
        }
        double v = (x_m > 0.0 ? -1.0 : 1.0) * CFG_CENTER_VEL_MMS;
        if ((x_m > CFG_TRACK_LIMIT_M || x_m < -CFG_TRACK_LIMIT_M) && !warned) {
            warned = 1;
            printf("[edge] WARNING: cart beyond soft limit (%.3f m), moving inward\n", x_m);
        }
        rig_send_vel(v);
        nap_ms(5);
    }
    rig_send_vel(0.0);
    return -1;
}

int rig_wait_reply(double timeout_s)
{
    unsigned seq0 = g_reply_seq;
    double t0 = rig_now_s();
    while (g_reply_seq == seq0 && rig_now_s() - t0 < timeout_s) {
        rig_pump();
        nap_ms(5);
    }
    return g_reply_seq != seq0 ? g_reply_err : -1;
}

/* ---- 快照访问 ---- */

fw_mode_t rig_mode(int *known, fw_reason_t *reason)
{
    if (known != NULL) {
        *known = g_mode_known;
    }
    if (reason != NULL) {
        *reason = g_mode_reason;
    }
    return g_mode;
}

void rig_telemetry(int64_t *x_um, int64_t *theta_urad, int64_t *vx_mm_s,
                   int64_t *vth_mrad_s)
{
    if (x_um != NULL) {
        *x_um = g_x_um;
    }
    if (theta_urad != NULL) {
        *theta_urad = g_theta_urad;
    }
    if (vx_mm_s != NULL) {
        *vx_mm_s = g_vx_mm_s;
    }
    if (vth_mrad_s != NULL) {
        *vth_mrad_s = g_vth_mrad_s;
    }
}

double rig_state_age_s(void)
{
    return g_state_t < 0.0 ? -1.0 : rig_now_s() - g_state_t;
}

void rig_print_state(void)
{
    int64_t x, th, vx, vth;
    rig_telemetry(&x, &th, &vx, &vth);
    int known;
    fw_mode_t m = rig_mode(&known, NULL);
    double age = rig_state_age_s();
    if (age < 0.0) {
        printf("no STATE received yet\n");
        return;
    }
    printf("x=%+lld um  theta=%+lld urad  vx=%+lld mm/s  vth=%+lld mrad/s  age=%.0f ms  mode=%s\n",
           (long long)x, (long long)th, (long long)vx, (long long)vth, age * 1000.0,
           known ? proto_mode_name(m) : "unknown");
}
