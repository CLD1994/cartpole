/* 串口会话：明确区分有效测量、模式、请求回执及传输错误。 */
#include "cartpole.h"
#include "config.h"
#include "uart.h"
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop;
static uart_t g_uart = {.fd = -1};
static proto_rx_t g_rx;
static cartpole_snapshot_t g_snapshot = {.state_received_at_s = -1.};
static int g_need_frame_boundary;
static uint64_t g_mode_seq;
static char g_wait_name[16];
static int g_reply_seen, g_reply_err;
static int64_t g_reply_value;
struct cached_value {
    char name[16];
    int64_t value;
};
static struct cached_value g_values[24];
static size_t g_value_count;
static cartpole_observer_t g_update_hook;
static void *g_update_context;

void cartpole_set_observer(cartpole_observer_t hook, void *context)
{
    g_update_hook = hook;
    g_update_context = context;
}
cartpole_snapshot_t cartpole_snapshot(void)
{
    return g_snapshot;
}
double cartpole_snapshot_age(cartpole_snapshot_t s)
{
    return s.state_received_at_s < 0 ? -1. : cartpole_now_s() - s.state_received_at_s;
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
static int write_until(const uint8_t *data, size_t n, double end, int cleanup)
{
    while ((cleanup || !g_stop) && cartpole_now_s() < end) {
        int result = uart_try_write_all(&g_uart, data, n);
        if (result == 0)
            return cartpole_now_s() < end ? 0 : -1;
        if (result != 1)
            return -1;
        nap_ms(1);
    }
    return -1;
}
static int send_line(const char *s, size_t n, double end, int cleanup)
{
    if (!cleanup && g_stop)
        return -1;
    if (g_need_frame_boundary) {
        if (write_until((const uint8_t *)"\n", 1, end, cleanup) != 0)
            return -1;
        g_need_frame_boundary = 0;
    }
    if (!n || write_until((const uint8_t *)s, n, end, cleanup) != 0) {
        g_snapshot.io_error = 1;
        g_need_frame_boundary = 1;
        return -1;
    }
    return 0;
}
cartpole_send_result_t cartpole_try_send_vel(double v, double deadline_s)
{
    char line[64];
    size_t n = proto_format_vel(line, v);
    if (!n || !isfinite(deadline_s) || g_stop || g_snapshot.io_error || !g_snapshot.mode_known ||
        g_snapshot.mode != FW_READY || g_snapshot.reason != FW_REASON_NONE ||
        !(g_snapshot.valid_flags & 1))
        return CARTPOLE_SEND_ERROR;
    double now = cartpole_now_s();
    if (now >= deadline_s)
        return CARTPOLE_SEND_SKIPPED;
    if (g_snapshot.state_received_at_s < 0 ||
        now - g_snapshot.state_received_at_s > CFG_STATE_TIMEOUT_S)
        return CARTPOLE_SEND_ERROR;
    int result = uart_try_write_all(&g_uart, (const uint8_t *)line, n);
    if (result < 0) {
        g_snapshot.io_error = 1;
        g_need_frame_boundary = 1;
        return CARTPOLE_SEND_ERROR;
    }
    if (result == 0 && cartpole_now_s() >= deadline_s)
        return CARTPOLE_SEND_LATE; /* 已完整写入，不能撤销；调用方终止控制并执行停止流程。 */
    return result == 0 ? CARTPOLE_SEND_COMPLETE : CARTPOLE_SEND_SKIPPED;
}
static void await_name(const char *name)
{
    snprintf(g_wait_name, sizeof g_wait_name, "%s", name);
    g_reply_seen = 0;
    g_reply_err = 0;
    g_reply_value = 0;
}
cartpole_send_result_t cartpole_send_vel(double v)
{
    char s[64];
    /* 零速请求允许在未知模式或失效测量后用于停止；非零请求必须有新鲜小车测量。 */
    if (v != 0. && (g_stop || g_snapshot.io_error || !g_snapshot.mode_known ||
                    g_snapshot.mode != FW_READY || g_snapshot.reason != FW_REASON_NONE ||
                    !(g_snapshot.valid_flags & 1) || cartpole_snapshot_age(g_snapshot) < 0 ||
                    cartpole_snapshot_age(g_snapshot) > CFG_STATE_TIMEOUT_S))
        return -1;
    return send_line(s, proto_format_vel(s, v), cartpole_now_s() + .020, v == 0.);
}
static int send_simple(const char *cmd, double end, int cleanup)
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
    return send_line(s, proto_format_simple(s, cmd), end, cleanup);
}
static int send_config(const char *name, long value, double end)
{
    char s[64];
    await_name(name);
    return send_line(s, proto_format_config(s, name, value), end, 0);
}
int cartpole_open(const char *port, int baud)
{
    if (g_uart.fd >= 0)
        uart_close(&g_uart);
    g_stop = 0;
    g_snapshot.io_error = 0;
    g_need_frame_boundary = 0;
    g_snapshot.fault_seen = 0;
    g_snapshot.valid_flags = 0;
    g_snapshot.state_sequence = g_mode_seq = 0;
    g_snapshot.state_received_at_s = -1.;
    g_snapshot.state_count = g_snapshot.unknown_count = 0;
    g_snapshot.mode = FW_DISABLED;
    g_snapshot.reason = FW_REASON_NONE;
    g_snapshot.mode_known = 0;
    g_snapshot.x_um = g_snapshot.theta_urad = g_snapshot.vx_mm_s = g_snapshot.vth_mrad_s = 0;
    g_value_count = 0;
    g_update_hook = NULL;
    g_update_context = NULL;
    await_name("");
    proto_rx_init(&g_rx);
    int result = uart_open(&g_uart, port, baud);
    g_snapshot.connected = result == 0;
    return result;
}
void cartpole_close(void)
{
    uart_close(&g_uart);
    g_snapshot = (cartpole_snapshot_t){.state_received_at_s = -1.};
    g_value_count = 0;
    g_update_hook = NULL;
    g_update_context = NULL;
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
int cartpole_cached_value(const char *name, int64_t *value)
{
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
        g_snapshot.unknown_count++;
        if (!strncmp(line, "STATE", 5) || !strncmp(line, "MODE", 4) || !strcmp(line, "!OVERLONG"))
            g_snapshot.io_error = 2;
        return;
    }
    switch (m.type) {
    case PROTO_STATE:
        g_snapshot.valid_flags = m.valid_flags;
        g_snapshot.x_um = m.x_um;
        g_snapshot.theta_urad = m.theta_urad;
        g_snapshot.vx_mm_s = m.vx_mm_s;
        g_snapshot.vth_mrad_s = m.vth_mrad_s;
        g_snapshot.state_received_at_s = cartpole_now_s();
        g_snapshot.state_sequence++;
        g_snapshot.state_count++;
        break;
    case PROTO_MODE:
        if (!g_snapshot.mode_known || g_snapshot.mode != m.mode || g_snapshot.reason != m.reason)
            printf("[edge] MODE %s %s\n", proto_mode_name(m.mode), proto_reason_name(m.reason));
        g_snapshot.mode = m.mode;
        g_snapshot.reason = m.reason;
        g_snapshot.mode_known = 1;
        g_mode_seq++;
        if (m.mode == FW_FAULT)
            g_snapshot.fault_seen = 1;
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
        g_update_hook(m.type, &g_snapshot, g_update_context);
}
static int pump(int cleanup)
{
    uint8_t buf[512];
    for (size_t bytes = 0; bytes < 4096;) {
        int n = uart_read(&g_uart, buf, sizeof buf);
        if (n < 0) {
            g_snapshot.io_error = 1;
            return -1;
        }
        if (n == 0)
            return !cleanup && g_snapshot.io_error ? -1 : 0;
        bytes += (size_t)n;
        proto_rx_feed(&g_rx, buf, (size_t)n, handle_line, NULL);
        if (!cleanup && g_snapshot.io_error)
            return -1;
    }
    return 1;
}
cartpole_pump_result_t cartpole_pump(void)
{
    return pump(0);
}
/* 发送控制面请求前读到空边界，避免用之前积压的回执满足新请求。 */
static int drain(double deadline)
{
    if (g_stop || cartpole_now_s() >= deadline)
        return -1;
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
    if (send_simple("QUERY", end, 0) != 0)
        return -1;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0 || cartpole_now_s() >= end)
            return -1;
        if (g_mode_seq != seq)
            return 0;
        nap_ms(1);
    }
    return -1;
}
int cartpole_wait_state(double timeout_s)
{
    uint64_t seq = g_snapshot.state_sequence;
    double end = cartpole_now_s() + timeout_s;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0 || cartpole_now_s() >= end)
            return -1;
        if (g_snapshot.state_sequence != seq)
            return 0;
        nap_ms(1);
    }
    return -1;
}
static int wait_reply(double end)
{
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0 || cartpole_now_s() >= end)
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
    if (drain(end) != 0 || send_config(name, value, end) != 0)
        return -1;
    if (wait_reply(end) != 0 || g_reply_value != value)
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
    if (send_simple("CALIB?", end, 0) != 0 || send_simple("CONFIG?", end, 0) != 0)
        return -1;
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0 || cartpole_now_s() >= end)
            return -1;
        size_t i;
        for (i = 0; i < sizeof required / sizeof required[0]; i++)
            if (cartpole_cached_value(required[i], NULL) != 0)
                break;
        if (i == sizeof required / sizeof required[0])
            return 0;
        nap_ms(1);
    }
    return -1;
}
static int wait_mode(fw_mode_t target, uint64_t seq, double end)
{
    while (!g_stop && cartpole_now_s() < end) {
        if (cartpole_pump() < 0 || cartpole_now_s() >= end)
            return -1;
        if (g_reply_seen && g_reply_err)
            return -1;
        if (g_mode_seq != seq) {
            if (g_snapshot.mode == target && g_snapshot.reason == FW_REASON_NONE)
                return 0;
            if (g_snapshot.mode == FW_FAULT)
                return -1;
        }
        nap_ms(1);
    }
    return -1;
}
int cartpole_enable(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (drain(end) != 0)
        return -1;
    uint64_t seq = g_mode_seq;
    if (cartpole_now_s() >= end || send_simple("ENABLE", end, 0) != 0)
        return -1;
    return wait_mode(FW_READY, seq, end);
}
cartpole_stop_result_t cartpole_stop_and_confirm(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    /* 清理读边界不受停止标记或此前协议错误锁存阻挡。旧MODE不能证明本次停止。 */
    int drained;
    do {
        drained = pump(1);
    } while (drained == 1 && cartpole_now_s() < end);
    if (drained != 0)
        return CARTPOLE_STOP_UNCONFIRMED;
    uint64_t mode_seq = g_mode_seq, state_seq = g_snapshot.state_sequence;
    double next_send = 0.;
    int zero_observed = 0;
    while (cartpole_now_s() < end) {
        if (cartpole_now_s() >= next_send) {
            (void)send_simple("VEL 0", end, 1);
            if (zero_observed)
                (void)send_simple("DISABLE", end, 1);
            (void)send_simple("QUERY", end, 1);
            next_send = cartpole_now_s() + .02;
        }
        if (pump(1) < 0 || cartpole_now_s() >= end)
            return CARTPOLE_STOP_UNCONFIRMED;
        if (g_mode_seq != mode_seq) {
            if (g_snapshot.mode == FW_DISABLED && g_snapshot.reason == FW_REASON_NONE)
                return CARTPOLE_STOP_DISABLED;
            if (g_snapshot.mode == FW_FAULT)
                return CARTPOLE_STOP_FAULT;
        }
        if (g_snapshot.state_sequence != state_seq && (g_snapshot.valid_flags & 1) &&
            g_snapshot.vx_mm_s == 0 && !zero_observed) {
            zero_observed = 1;
            next_send = 0.;
        }
        nap_ms(1);
    }
    return CARTPOLE_STOP_UNCONFIRMED;
}

int cartpole_reset(double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (drain(end) != 0)
        return -1;
    uint64_t seq = g_mode_seq;
    if (cartpole_now_s() >= end || send_simple("RESET", end, 0) != 0)
        return -1;
    return wait_mode(FW_DISABLED, seq, end);
}
int cartpole_mark(cartpole_endpoint_t endpoint, double timeout_s)
{
    double end = cartpole_now_s() + timeout_s;
    if (endpoint != CARTPOLE_LEFT && endpoint != CARTPOLE_RIGHT)
        return -1;
    if (drain(end) != 0 || cartpole_now_s() >= end ||
        send_simple(endpoint == CARTPOLE_LEFT ? "MARK LEFT" : "MARK RIGHT", end, 0) != 0)
        return -1;
    return wait_reply(end) == 0 ? 0 : -1;
}
