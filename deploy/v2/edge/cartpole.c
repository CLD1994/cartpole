/* 串口会话：明确区分有效测量、模式、请求回执及传输错误。 */
#include "cartpole.h"
#include "uart.h"
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop;
static uart_t g_uart = {.fd = -1};
static proto_rx_t g_rx;
static int g_io_error, g_fault_seen;
static int g_need_frame_boundary;
static int64_t g_x_um, g_theta_urad, g_vx_mm_s, g_vth_mrad_s;
static unsigned g_flags;
static uint64_t g_state_seq, g_mode_seq;
static double g_state_t = -1.;
static long g_state_count, g_unknown_count;
static fw_mode_t g_mode = FW_DISABLED;
static fw_reason_t g_reason = FW_REASON_NONE;
static int g_mode_known;
static char g_wait_name[16];
static int g_reply_seen, g_reply_err;
static int64_t g_reply_value;
struct cached_value {
    char name[16];
    int64_t value;
};
static struct cached_value g_values[24];
static size_t g_value_count;
static void (*g_update_hook)(void *);
static void *g_update_context;

void cartpole_set_update_hook(void (*hook)(void *), void *context)
{
    g_update_hook = hook;
    g_update_context = context;
}
double cartpole_state_received_at_s(void)
{
    return g_state_t;
}

static void nap_ms(int ms)
{
    struct timespec t = {0, (long)ms * 1000000};
    nanosleep(&t, NULL);
}
double cartpole_now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
void cartpole_request_stop(void)
{
    g_stop = 1;
}
int cartpole_stop_requested(void)
{
    return g_stop != 0;
}
int cartpole_io_error(void)
{
    return g_io_error;
}
int cartpole_fault_seen(void)
{
    return g_fault_seen;
}
int cartpole_valid_flags(void)
{
    return (int)g_flags;
}
uint64_t cartpole_state_sequence(void)
{
    return g_state_seq;
}

static int send_line(const char *s, size_t n)
{
    /* 之前可能只有命令前缀进入内核，必须先结束旧行，不能直接拼接停止命令。 */
    if (g_need_frame_boundary) {
        if (uart_write_all(&g_uart, (const uint8_t *)"\n", 1) != 0)
            return -1;
        g_need_frame_boundary = 0;
    }
    if (!n || uart_write_all(&g_uart, (const uint8_t *)s, n) != 0) {
        g_io_error = 1;
        g_need_frame_boundary = 1;
        return -1;
    }
    return 0;
}
int cartpole_try_send_vel(double v, double deadline_s)
{
    char line[64];
    size_t n = proto_format_vel(line, v);
    if (!n || !isfinite(deadline_s) || g_stop || g_io_error || !g_mode_known ||
        g_mode != FW_READY || g_reason != FW_REASON_NONE || !(g_flags & 1))
        return -1;
    double now = cartpole_now_s();
    if (now >= deadline_s)
        return 1;
    if (g_state_t < 0 || now - g_state_t > CFG_STATE_TIMEOUT_S)
        return -1;
    int result = uart_try_write_all(&g_uart, (const uint8_t *)line, n);
    if (result < 0) {
        g_io_error = 1;
        g_need_frame_boundary = 1;
        return -1;
    }
    if (result == 0 && cartpole_now_s() >= deadline_s)
        return 2; /* 已完整写入，不能撤销；调用方终止控制并执行停止流程。 */
    return result;
}
static void await_name(const char *name)
{
    snprintf(g_wait_name, sizeof g_wait_name, "%s", name);
    g_reply_seen = 0;
    g_reply_err = 0;
    g_reply_value = 0;
}
int cartpole_send_vel(double v)
{
    char s[64];
    /* 零速请求允许在未知模式或失效测量后用于停止；非零请求必须有新鲜小车测量。 */
    if (v != 0. && (g_stop || g_io_error || !g_mode_known || g_mode != FW_READY ||
                    g_reason != FW_REASON_NONE || !(g_flags & 1) || cartpole_state_age_s() < 0 ||
                    cartpole_state_age_s() > CFG_STATE_TIMEOUT_S))
        return -1;
    return send_line(s, proto_format_vel(s, v));
}
int cartpole_send_simple(const char *cmd)
{
    char s[64];
    if (!strcmp(cmd, "MARK LEFT"))
        await_name("MARK_LEFT");
    else if (!strcmp(cmd, "MARK RIGHT"))
        await_name("MARK_RIGHT");
    else if (!strcmp(cmd, "ENABLE"))
        await_name("ENABLE");
    else if (!strcmp(cmd, "RESET"))
        await_name("RESET");
    else if (!strcmp(cmd, "DISABLE"))
        await_name("DISABLE");
    return send_line(s, proto_format_simple(s, cmd));
}
int cartpole_send_config(const char *name, long value)
{
    char s[64];
    await_name(name);
    return send_line(s, proto_format_config(s, name, value));
}
int cartpole_open(const char *port, int baud)
{
    if (g_uart.fd >= 0)
        uart_close(&g_uart);
    g_stop = 0;
    g_io_error = 0;
    g_need_frame_boundary = 0;
    g_fault_seen = 0;
    g_flags = 0;
    g_state_seq = g_mode_seq = 0;
    g_state_t = -1.;
    g_state_count = g_unknown_count = 0;
    g_mode = FW_DISABLED;
    g_reason = FW_REASON_NONE;
    g_mode_known = 0;
    g_x_um = g_theta_urad = g_vx_mm_s = g_vth_mrad_s = 0;
    g_value_count = 0;
    g_update_hook = NULL;
    g_update_context = NULL;
    await_name("");
    proto_rx_init(&g_rx);
    return uart_open(&g_uart, port, baud);
}
void cartpole_close(void)
{
    uart_close(&g_uart);
}
static void cache_value(const char *name, int64_t value)
{
    size_t i = 0;
    for (; i < g_value_count; i++)
        if (!strcmp(g_values[i].name, name))
            break;
    if (i == g_value_count) {
        if (i >= sizeof g_values / sizeof g_values[0])
            return;
        g_value_count++;
    }
    snprintf(g_values[i].name, sizeof g_values[i].name, "%s", name);
    g_values[i].value = value;
}
int cartpole_get_config(const char *name, int64_t *value, double timeout_s)
{
    (void)timeout_s; /* 只读最近一次 refresh/configure 的回执缓存，不触发隐式I/O。 */
    for (size_t i = 0; i < g_value_count; i++)
        if (!strcmp(name, g_values[i].name)) {
            if (value)
                *value = g_values[i].value;
            return 0;
        }
    return -1;
}
static void handle_line(const char *line, size_t len, void *user)
{
    (void)len;
    (void)user;
    proto_msg_t m;
    if (proto_parse_line(line, &m) == PROTO_UNKNOWN) {
        g_unknown_count++;
        if (!strncmp(line, "STATE", 5) || !strncmp(line, "MODE", 4) || !strcmp(line, "!OVERLONG"))
            g_io_error = 2;
        return;
    }
    switch (m.type) {
    case PROTO_STATE:
        g_flags = m.valid_flags;
        g_x_um = m.x_um;
        g_theta_urad = m.theta_urad;
        g_vx_mm_s = m.vx_mm_s;
        g_vth_mrad_s = m.vth_mrad_s;
        g_state_t = cartpole_now_s();
        g_state_seq++;
        g_state_count++;
        break;
    case PROTO_MODE:
        if (!g_mode_known || g_mode != m.mode || g_reason != m.reason)
            printf("[edge] MODE %s %s\n", proto_mode_name(m.mode), proto_reason_name(m.reason));
        g_mode = m.mode;
        g_reason = m.reason;
        g_mode_known = 1;
        g_mode_seq++;
        if (m.mode == FW_FAULT)
            g_fault_seen = 1;
        break;
    case PROTO_OK:
    case PROTO_ERR:
        if (m.type == PROTO_OK)
            cache_value(m.param, m.value);
        if (!g_reply_seen && g_wait_name[0] && !strcmp(g_wait_name, m.param)) {
            g_reply_seen = 1;
            g_reply_err = m.type == PROTO_ERR;
            g_reply_value = m.value;
        }
        if (m.type == PROTO_OK)
            printf("[edge] OK %s %lld\n", m.param, (long long)m.value);
        else
            printf("[edge] ERR %s\n", m.param);
        break;
    case PROTO_STEP_LOSS:
        printf("[edge] STEP_LOSS event=%lld run=%lld device_us=%lld (record only)\n",
               (long long)m.event_id, (long long)m.run_id, (long long)m.device_us);
        break;
    default:
        break;
    }
    if (g_update_hook && (m.type == PROTO_STATE || m.type == PROTO_MODE || m.type == PROTO_OK))
        g_update_hook(g_update_context);
}
static int pump(int cleanup)
{
    uint8_t buf[512];
    for (size_t bytes = 0; bytes < 4096;) {
        int n = uart_read(&g_uart, buf, sizeof buf);
        if (n < 0) {
            g_io_error = 1;
            return -1;
        }
        if (n == 0)
            return !cleanup && g_io_error ? -1 : 0;
        bytes += (size_t)n;
        proto_rx_feed(&g_rx, buf, (size_t)n, handle_line, NULL);
        if (!cleanup && g_io_error)
            return -1;
    }
    return 1;
}
int cartpole_pump(void)
{
    return pump(0);
}
/* 发送控制面请求前读到空边界，避免用之前积压的回执满足新请求。 */
static int drain(double deadline)
{
    int r;
    do {
        r = cartpole_pump();
        if (r <= 0)
            return r;
        if (cartpole_now_s() >= deadline)
            return -1;
    } while (!g_stop);
    return -1;
}
int cartpole_sync_mode(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (drain(end) != 0)
        return -1;
    uint64_t seq = g_mode_seq;
    if (cartpole_send_simple("QUERY") != 0)
        return -1;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0)
            return -1;
        if (g_mode_seq != seq)
            return 0;
        nap_ms(1);
    }
    return -1;
}
int cartpole_wait_state(double timeout_s)
{
    uint64_t seq = g_state_seq;
    double end = cartpole_now_s() + timeout_s;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0)
            return -1;
        if (g_state_seq != seq)
            return 0;
        nap_ms(1);
    }
    return -1;
}
int cartpole_wait_reply(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0)
            return -1;
        if (g_reply_seen)
            return g_reply_err ? 1 : 0;
        nap_ms(1);
    }
    return -1;
}
int cartpole_configure(const char *name, long value, double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (drain(end) != 0 || cartpole_send_config(name, value) != 0)
        return -1;
    if (cartpole_wait_reply(end - cartpole_now_s()) != 0 || g_reply_value != value)
        return -1;
    return 0;
}
int cartpole_refresh_metadata(double timeout_s)
{
    static const char *required[] = {"CAL_STATE", "CAL_LENGTH_UM", "CAL_SAVED", "CAL_STORAGE",
                                     "VEL_MAX",   "ACC_MAX",       "WDT_MS",    "X_MARGIN_UM"};
    double end = cartpole_now_s() + timeout_s;
    if (drain(end) != 0)
        return -1;
    g_value_count = 0;
    if (cartpole_send_simple("CALIB?") != 0 || cartpole_send_simple("CONFIG?") != 0)
        return -1;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0)
            return -1;
        size_t i;
        for (i = 0; i < sizeof required / sizeof required[0]; i++)
            if (cartpole_get_config(required[i], NULL, 0) != 0)
                break;
        if (i == sizeof required / sizeof required[0])
            return 0;
        nap_ms(1);
    }
    return -1;
}
int cartpole_wait_mode(fw_mode_t target, double timeout_s)
{
    uint64_t seq = g_mode_seq;
    double end = cartpole_now_s() + timeout_s;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0)
            return -1;
        if (g_reply_seen && g_reply_err)
            return -1;
        if (g_mode_seq != seq) {
            if (g_mode == target && g_reason == FW_REASON_NONE)
                return 0;
            if (g_mode == FW_FAULT)
                return -1;
        }
        nap_ms(1);
    }
    return -1;
}
int cartpole_enable(double timeout_s)
{
    if (drain(cartpole_now_s() + timeout_s) != 0 || cartpole_send_simple("ENABLE") != 0)
        return -1;
    return cartpole_wait_mode(FW_READY, timeout_s);
}
int cartpole_stop_and_confirm(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    /* 清理读边界不受停止标记或此前协议错误锁存阻挡。旧MODE不能证明本次停止。 */
    int drained;
    do {
        drained = pump(1);
    } while (drained == 1 && cartpole_now_s() < end);
    if (drained != 0)
        return -1;
    uint64_t mode_seq = g_mode_seq, state_seq = g_state_seq;
    double next_send = 0.;
    int zero_observed = 0;
    while (cartpole_now_s() < end) {
        if (cartpole_now_s() >= next_send) {
            (void)cartpole_send_vel(0.);
            if (zero_observed)
                (void)cartpole_send_simple("DISABLE");
            (void)cartpole_send_simple("QUERY");
            next_send = cartpole_now_s() + .02;
        }
        if (pump(1) < 0)
            return -1;
        if (g_mode_seq != mode_seq) {
            if (g_mode == FW_DISABLED && g_reason == FW_REASON_NONE)
                return 0;
            if (g_mode == FW_FAULT)
                return 1;
        }
        if (g_state_seq != state_seq && (g_flags & 1) && g_vx_mm_s == 0 && !zero_observed) {
            zero_observed = 1;
            next_send = 0.;
        }
        nap_ms(1);
    }
    return -1;
}
int cartpole_goto_center(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (cartpole_sync_mode(1.) != 0)
        return -1;
    while (!g_stop && cartpole_now_s() < end) {
        int p = cartpole_pump();
        if (p < 0)
            break;
        if (p > 0) {
            nap_ms(1);
            continue;
        }
        if (!g_mode_known || g_mode != FW_READY || !(g_flags & 1) || cartpole_state_age_s() < 0 ||
            cartpole_state_age_s() > CFG_STATE_TIMEOUT_S)
            break;
        if (g_x_um > -5000 && g_x_um < 5000) {
            if (cartpole_send_vel(0.) != 0)
                return -1;
            /* 持续零速并等待小车速度回报，不能把请求发送等同停稳。 */
            uint64_t seq = g_state_seq;
            while (!g_stop && cartpole_now_s() < end) {
                if (cartpole_pump() < 0)
                    return -1;
                if (g_mode != FW_READY || !(g_flags & 1))
                    return -1;
                if (g_state_seq != seq && g_vx_mm_s == 0)
                    return 0;
                if (cartpole_send_vel(0.) != 0)
                    return -1;
                nap_ms(5);
            }
            return -1;
        }
        if (cartpole_send_vel(g_x_um > 0 ? -CFG_CENTER_VEL_MMS : CFG_CENTER_VEL_MMS) != 0)
            break;
        nap_ms(5);
    }
    (void)cartpole_send_vel(0.);
    return -1;
}
fw_mode_t cartpole_mode(int *known, fw_reason_t *reason)
{
    if (known)
        *known = g_mode_known;
    if (reason)
        *reason = g_reason;
    return g_mode;
}
void cartpole_telemetry(int64_t *x, int64_t *th, int64_t *vx, int64_t *w)
{
    if (x)
        *x = g_x_um;
    if (th)
        *th = g_theta_urad;
    if (vx)
        *vx = g_vx_mm_s;
    if (w)
        *w = g_vth_mrad_s;
}
double cartpole_state_age_s(void)
{
    return g_state_t < 0 ? -1. : cartpole_now_s() - g_state_t;
}
void cartpole_print_state(void)
{
    if (g_state_t < 0) {
        puts("[edge] no STATE received");
        return;
    }
    printf("STATE flags=%u x=%lld um theta=%lld urad vx=%lld mm/s omega=%lld mrad/s age=%.1f ms\n",
           g_flags, (long long)g_x_um, (long long)g_theta_urad, (long long)g_vx_mm_s,
           (long long)g_vth_mrad_s, cartpole_state_age_s() * 1000.);
}
void cartpole_window_reset(void)
{
    g_state_count = g_unknown_count = 0;
}
long cartpole_state_count(void)
{
    return g_state_count;
}
long cartpole_unknown_count(void)
{
    return g_unknown_count;
}
