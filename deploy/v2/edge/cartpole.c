/* 串口会话：明确区分有效测量、模式、请求回执及传输错误。 */
#include "cartpole_internal.h"
#include "config.h"
#include "control.h"
#include "uart.h"
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop;
static uart_t g_uart = {.fd = -1};
static proto_rx_t g_rx;
static cartpole_snapshot_t g_snapshot = {.last_state_received_host_monotonic_s = -1.};
static int g_need_frame_boundary;
static uint64_t g_mode_seq, g_fault_seq;
/* 首个流错误在发生处分类并锁存，独立于展示用snapshot。 */
static cartpole_result_t g_stream_error;
static char g_wait_name[16];
static int g_reply_seen, g_reply_err;
static int64_t g_reply_value;
struct cached_value {
    char name[16];
    int64_t value;
};
static struct cached_value g_values[24];
static size_t g_value_count;
static void (*g_update_hook)(proto_msg_type_t, const cartpole_snapshot_t *, void *);
static cartpole_event_handler_t g_event_handler;
static void *g_event_context;
static int g_callback_depth, g_callback_violation, g_loop_active;
static int g_session_event_pending;
static void flush_session_event(void);
static void *g_update_context;

void cartpole_set_update_hook(void (*hook)(proto_msg_type_t, const cartpole_snapshot_t *, void *),
                              void *context)
{
    g_update_hook = hook;
    g_update_context = context;
}
cartpole_snapshot_t cartpole_snapshot(void) { return g_snapshot; }
double cartpole_state_age_s(cartpole_snapshot_t s)
{
    return s.last_state_received_host_monotonic_s < 0
               ? -1.
               : cartpole_host_monotonic_s() - s.last_state_received_host_monotonic_s;
}

static void nap_ms(int ms)
{
    struct timespec t = {0, (long)ms * 1000000};
    nanosleep(&t, NULL);
}
double cartpole_host_monotonic_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
void cartpole_request_stop(void) { g_stop = 1; }
int cartpole_stop_requested(void) { return g_stop != 0; }
const char *cartpole_result_name(cartpole_result_t result)
{
    switch (result) {
    case CARTPOLE_OK:
        return "OK";
    case CARTPOLE_CANCELLED:
        return "CANCELLED";
    case CARTPOLE_TIMEOUT:
        return "TIMEOUT";
    case CARTPOLE_TRANSPORT_ERROR:
        return "TRANSPORT_ERROR";
    case CARTPOLE_PROTOCOL_ERROR:
        return "PROTOCOL_ERROR";
    case CARTPOLE_REJECTED:
        return "REJECTED";
    case CARTPOLE_FAULT:
        return "FAULT";
    case CARTPOLE_REPLY_MISMATCH:
        return "REPLY_MISMATCH";
    case CARTPOLE_INVALID_ARGUMENT:
        return "INVALID_ARGUMENT";
    case CARTPOLE_CALLBACK_ERROR:
        return "CALLBACK_ERROR";
    case CARTPOLE_MEASUREMENT_UNAVAILABLE:
        return "MEASUREMENT_UNAVAILABLE";
    case CARTPOLE_SESSION_UNAVAILABLE:
        return "SESSION_UNAVAILABLE";
    case CARTPOLE_CONTROL_DEADLINE:
        return "CONTROL_DEADLINE";
    case CARTPOLE_CONFIGURATION_CHANGED:
        return "CONFIGURATION_CHANGED";
    }
    return "UNKNOWN_RESULT";
}
static void stream_failed(cartpole_result_t result)
{
    if (g_stream_error == CARTPOLE_OK)
        g_stream_error = result;
    if (!g_snapshot.has_stream_failure_latch)
        g_session_event_pending = 1;
    g_snapshot.has_stream_failure_latch = 1;
    if (!g_callback_depth)
        flush_session_event();
}
static cartpole_result_t checkpoint(double end, int cleanup)
{
    if (!cleanup && g_stop)
        return CARTPOLE_CANCELLED;
    if (cartpole_host_monotonic_s() >= end)
        return CARTPOLE_TIMEOUT;
    return CARTPOLE_OK;
}
static cartpole_result_t deadline(double timeout, double *end)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return CARTPOLE_CALLBACK_ERROR;
    }
    if (!isfinite(timeout) || timeout <= 0.)
        return CARTPOLE_INVALID_ARGUMENT;
    *end = cartpole_host_monotonic_s() + timeout;
    if (!isfinite(*end))
        return CARTPOLE_INVALID_ARGUMENT;
    return CARTPOLE_OK;
}
static cartpole_result_t write_until(const uint8_t *data, size_t n, double end, int cleanup)
{
    for (;;) {
        cartpole_result_t result = checkpoint(end, cleanup);
        if (result != CARTPOLE_OK)
            return result;
        int written = uart_try_write_all(&g_uart, data, n);
        if (written < 0) {
            stream_failed(CARTPOLE_TRANSPORT_ERROR);
            g_need_frame_boundary = 1;
            return CARTPOLE_TRANSPORT_ERROR;
        }
        if (written == 0)
            return checkpoint(end, cleanup);
        nap_ms(1);
    }
}
static cartpole_result_t send_line(const char *s, size_t n, double end, int cleanup)
{
    if (!s || !n)
        return CARTPOLE_INVALID_ARGUMENT;
    cartpole_result_t result = checkpoint(end, cleanup);
    if (result != CARTPOLE_OK)
        return result;
    if (g_need_frame_boundary) {
        result = write_until((const uint8_t *)"\n", 1, end, cleanup);
        if (result != CARTPOLE_OK)
            return result;
        g_need_frame_boundary = 0;
    }
    return write_until((const uint8_t *)s, n, end, cleanup);
}
cartpole_send_result_t cartpole_try_send_vel(double v, double deadline_host_monotonic_s)
{
    char line[64];
    size_t n = proto_format_vel(line, v);
    if (!n || !isfinite(deadline_host_monotonic_s) || g_stop ||
        g_snapshot.has_stream_failure_latch || !g_snapshot.is_mode_received ||
        g_snapshot.reported_mode != FW_READY || g_snapshot.reported_mode_reason != FW_REASON_NONE ||
        !(g_snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID))
        return CARTPOLE_SEND_ERROR;
    double now = cartpole_host_monotonic_s();
    if (now >= deadline_host_monotonic_s)
        return CARTPOLE_SEND_SKIPPED;
    if (g_snapshot.last_state_received_host_monotonic_s < 0 ||
        now - g_snapshot.last_state_received_host_monotonic_s > CFG_STATE_TIMEOUT_S)
        return CARTPOLE_SEND_ERROR;
    int result = uart_try_write_all(&g_uart, (const uint8_t *)line, n);
    if (result < 0) {
        stream_failed(CARTPOLE_TRANSPORT_ERROR);
        g_need_frame_boundary = 1;
        return CARTPOLE_SEND_ERROR;
    }
    if (result == 0 && cartpole_host_monotonic_s() >= deadline_host_monotonic_s)
        return CARTPOLE_SEND_LATE; /* 已完整写入，不能撤销；调用方终止控制并执行停止流程。
                                    */
    return result == 0 ? CARTPOLE_SEND_COMPLETE : CARTPOLE_SEND_SKIPPED;
}
static void await_name(const char *name)
{
    snprintf(g_wait_name, sizeof g_wait_name, "%s", name);
    g_reply_seen = 0;
    g_reply_err = 0;
    g_reply_value = 0;
}
cartpole_send_result_t cartpole_send_vel_until(double v, double end)
{
    char line[64];
    size_t length = proto_format_vel(line, v);
    if (!length)
        return CARTPOLE_SEND_ERROR;
    /* 零速清理可在错误/取消后执行；非零维护发送失败会锁住后续非零发送。 */
    if (v != 0. && (g_stop || g_snapshot.has_stream_failure_latch || !g_snapshot.is_mode_received ||
                    g_snapshot.reported_mode != FW_READY ||
                    g_snapshot.reported_mode_reason != FW_REASON_NONE ||
                    !(g_snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID) ||
                    cartpole_state_age_s(g_snapshot) < 0 ||
                    cartpole_state_age_s(g_snapshot) > CFG_STATE_TIMEOUT_S))
        return CARTPOLE_SEND_ERROR;
    cartpole_result_t result =
        send_line(line, length, fmin(end, cartpole_host_monotonic_s() + .020), v == 0.);
    if (result != CARTPOLE_OK && v != 0.)
        stream_failed(result);
    return result == CARTPOLE_OK ? CARTPOLE_SEND_COMPLETE : CARTPOLE_SEND_ERROR;
}
cartpole_send_result_t cartpole_send_vel(double v)
{
    return cartpole_send_vel_until(v, cartpole_host_monotonic_s() + .020);
}
static cartpole_result_t send_simple(const char *cmd, double end, int cleanup)
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
static cartpole_result_t send_config(const char *name, long value, double end)
{
    char s[64];
    await_name(name);
    return send_line(s, proto_format_config(s, name, value), end, 0);
}
int cartpole_open(const char *port, int baud)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return -1;
    }
    if (g_uart.fd >= 0)
        uart_close(&g_uart);
    g_stop = 0;
    g_snapshot.has_stream_failure_latch = 0;
    g_stream_error = CARTPOLE_OK;
    g_fault_seq = 0;
    g_need_frame_boundary = 0;
    g_snapshot.has_received_fault_mode = 0;
    g_snapshot.measurement_validity_bits = 0;
    g_snapshot.received_state_count = g_mode_seq = 0;
    g_snapshot.last_state_received_host_monotonic_s = -1.;
    g_snapshot.received_unknown_line_count = 0;
    g_snapshot.reported_mode = FW_DISABLED;
    g_snapshot.reported_mode_reason = FW_REASON_NONE;
    g_snapshot.is_mode_received = 0;
    g_snapshot.x_um = g_snapshot.theta_urad = g_snapshot.vx_mm_s = g_snapshot.vth_mrad_s = 0;
    g_value_count = 0;
    g_update_hook = NULL;
    g_update_context = NULL;
    g_event_handler = NULL;
    g_event_context = NULL;
    g_callback_violation = 0;
    g_loop_active = 0;
    g_session_event_pending = 0;
    await_name("");
    proto_rx_init(&g_rx);
    int result = uart_open(&g_uart, port, baud);
    g_snapshot.is_serial_open = result == 0;
    return result;
}
void cartpole_close(void)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return;
    }
    uart_close(&g_uart);
    g_snapshot = (cartpole_snapshot_t){.last_state_received_host_monotonic_s = -1.};
    g_value_count = 0;
    g_stream_error = CARTPOLE_OK;
    g_update_hook = NULL;
    g_update_context = NULL;
    g_event_handler = NULL;
    g_event_context = NULL;
    g_callback_violation = 0;
    g_loop_active = 0;
    g_session_event_pending = 0;
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
    if (!name || !name[0])
        return -1;
    for (size_t i = 0; i < g_value_count; i++)
        if (!strcmp(name, g_values[i].name)) {
            if (value)
                *value = g_values[i].value;
            return 0;
        }
    return -1;
}
static void dispatch_business_event(const proto_msg_t *message);
static void handle_line(const char *line, size_t len, void *user)
{
    (void)len;
    (void)user;
    proto_msg_t m;
    if (proto_parse_line(line, &m) == PROTO_UNKNOWN) {
        g_snapshot.received_unknown_line_count++;
        if (!strncmp(line, "STATE", 5) || !strncmp(line, "MODE", 4) || !strcmp(line, "!OVERLONG"))
            stream_failed(CARTPOLE_PROTOCOL_ERROR);
        return;
    }
    switch (m.type) {
    case PROTO_STATE:
        g_snapshot.measurement_validity_bits = m.valid_flags;
        g_snapshot.x_um = m.x_um;
        g_snapshot.theta_urad = m.theta_urad;
        g_snapshot.vx_mm_s = m.vx_mm_s;
        g_snapshot.vth_mrad_s = m.vth_mrad_s;
        g_snapshot.last_state_received_host_monotonic_s = cartpole_host_monotonic_s();
        g_snapshot.received_state_count++;
        break;
    case PROTO_MODE:
        g_snapshot.reported_mode = m.mode;
        g_snapshot.reported_mode_reason = m.reason;
        g_snapshot.is_mode_received = 1;
        g_mode_seq++;
        if (m.mode == FW_FAULT) {
            g_snapshot.has_received_fault_mode = 1;
            g_fault_seq++;
        }
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
        break;
    case PROTO_STEP_LOSS:
        break;
    default:
        break;
    }
    if (g_update_hook && (m.type == PROTO_STATE || m.type == PROTO_MODE || m.type == PROTO_OK))
        g_update_hook(m.type, &g_snapshot, g_update_context);
    dispatch_business_event(&m);
}
static int pump(int cleanup)
{
    flush_session_event();
    uint8_t buf[512];
    for (size_t bytes = 0; bytes < 4096;) {
        int n = uart_read(&g_uart, buf, sizeof buf);
        if (n < 0) {
            stream_failed(CARTPOLE_TRANSPORT_ERROR);
            return -1;
        }
        if (n == 0)
            return !cleanup && g_snapshot.has_stream_failure_latch ? -1 : 0;
        bytes += (size_t)n;
        proto_rx_feed(&g_rx, buf, (size_t)n, handle_line, NULL);
        if (!cleanup && g_snapshot.has_stream_failure_latch)
            return -1;
    }
    return 1;
}
cartpole_pump_result_t cartpole_pump(void) { return pump(0); }
/* 在接收错误发生处已有精确分类；事务不根据最后MODE/stop倒推错误。 */
static cartpole_result_t receive(double end)
{
    cartpole_result_t result = checkpoint(end, 0);
    if (result != CARTPOLE_OK)
        return result;
    if (pump(0) < 0)
        return g_stream_error;
    return checkpoint(end, 0);
}
static cartpole_result_t drain(double end)
{
    for (;;) {
        cartpole_result_t result = checkpoint(end, 0);
        if (result != CARTPOLE_OK)
            return result;
        int pumped = pump(0);
        if (pumped < 0)
            return g_stream_error;
        result = checkpoint(end, 0);
        if (result != CARTPOLE_OK)
            return result;
        if (pumped == 0)
            return CARTPOLE_OK;
    }
}
/* 仅用于按顺序传播已分类的失败，不执行自动重试。 */
#define TX_TRY(expression)                                                                         \
    do {                                                                                           \
        cartpole_result_t result_ = (expression);                                                  \
        if (result_ != CARTPOLE_OK)                                                                \
            return result_;                                                                        \
    } while (0)
cartpole_result_t cartpole_query_mode(double timeout_s)
{
    double end;
    TX_TRY(deadline(timeout_s, &end));
    TX_TRY(drain(end));
    uint64_t seq = g_mode_seq;
    TX_TRY(send_simple("QUERY", end, 0));
    for (;;) {
        TX_TRY(receive(end));
        if (g_mode_seq != seq)
            return CARTPOLE_OK;
        nap_ms(1);
    }
}
cartpole_result_t cartpole_wait_measurement(double timeout_s)
{
    double end;
    TX_TRY(deadline(timeout_s, &end));
    uint64_t seq = g_snapshot.received_state_count;
    for (;;) {
        TX_TRY(receive(end));
        if (g_snapshot.received_state_count != seq)
            return CARTPOLE_OK;
        nap_ms(1);
    }
}
static cartpole_result_t wait_reply(double end, uint64_t faults)
{
    for (;;) {
        TX_TRY(receive(end));
        if (g_reply_seen && g_reply_err)
            return CARTPOLE_REJECTED;
        if (g_fault_seq != faults)
            return CARTPOLE_FAULT;
        if (g_reply_seen)
            return CARTPOLE_OK;
        nap_ms(1);
    }
}
cartpole_result_t cartpole_configure(cartpole_setting_t setting, long value, double timeout_s)
{
    double end;
    TX_TRY(deadline(timeout_s, &end));
    const char *name = cartpole_setting_name(setting);
    char line[64];
    if (!name || strlen(name) >= sizeof g_wait_name || !proto_format_config(line, name, value))
        return CARTPOLE_INVALID_ARGUMENT;
    TX_TRY(drain(end));
    uint64_t faults = g_fault_seq;
    TX_TRY(send_config(name, value, end));
    TX_TRY(wait_reply(end, faults));
    return g_reply_value == value ? CARTPOLE_OK : CARTPOLE_REPLY_MISMATCH;
}
cartpole_result_t cartpole_refresh_configuration(double timeout_s)
{
    static const char *required[] = {"CAL_STATE", "CAL_LENGTH_UM", "CAL_SAVED", "CAL_STORAGE",
                                     "VEL_MAX",   "ACC_MAX",       "WDT_MS",    "X_MARGIN_UM"};
    double end;
    TX_TRY(deadline(timeout_s, &end));
    TX_TRY(drain(end));
    g_value_count = 0;
    await_name(""); /* 批量查询没有可精确关联的单个ERR，缺项直到TIMEOUT。 */
    TX_TRY(send_simple("CALIB?", end, 0));
    TX_TRY(send_simple("CONFIG?", end, 0));
    for (;;) {
        TX_TRY(receive(end));
        size_t i;
        for (i = 0; i < sizeof required / sizeof required[0]; i++)
            if (cartpole_cached_value(required[i], NULL) != 0)
                break;
        if (i == sizeof required / sizeof required[0])
            return CARTPOLE_OK;
        nap_ms(1);
    }
}
static cartpole_result_t mode_transaction(const char *command, fw_mode_t target, double timeout_s)
{
    double end;
    TX_TRY(deadline(timeout_s, &end));
    TX_TRY(drain(end));
    uint64_t seq = g_mode_seq, faults = g_fault_seq;
    TX_TRY(send_simple(command, end, 0));
    for (;;) {
        TX_TRY(receive(end));
        if (g_reply_seen && g_reply_err)
            return CARTPOLE_REJECTED;
        if (g_fault_seq != faults)
            return CARTPOLE_FAULT;
        if (g_mode_seq != seq && g_snapshot.reported_mode == target &&
            g_snapshot.reported_mode_reason == FW_REASON_NONE)
            return CARTPOLE_OK;
        nap_ms(1);
    }
}
cartpole_result_t cartpole_enable(double timeout_s)
{
    return mode_transaction("ENABLE", FW_READY, timeout_s);
}
cartpole_stop_result_t cartpole_stop_and_confirm(double timeout_s)
{
    double end;
    if (deadline(timeout_s, &end) != CARTPOLE_OK)
        return CARTPOLE_STOP_UNCONFIRMED;
    /* 清理读边界不受停止标记或此前协议错误锁存阻挡。旧MODE不能证明本次停止。 */
    int drained;
    do {
        drained = pump(1);
    } while (drained == 1 && cartpole_host_monotonic_s() < end);
    if (drained != 0)
        return CARTPOLE_STOP_UNCONFIRMED;
    uint64_t mode_seq = g_mode_seq, state_seq = g_snapshot.received_state_count;
    double next_send = 0.;
    int zero_observed = 0;
    while (cartpole_host_monotonic_s() < end) {
        if (cartpole_host_monotonic_s() >= next_send) {
            (void)send_simple("VEL 0", end, 1);
            if (zero_observed)
                (void)send_simple("DISABLE", end, 1);
            (void)send_simple("QUERY", end, 1);
            next_send = cartpole_host_monotonic_s() + .02;
        }
        if (pump(1) < 0 || cartpole_host_monotonic_s() >= end)
            return CARTPOLE_STOP_UNCONFIRMED;
        if (g_mode_seq != mode_seq) {
            if (g_snapshot.reported_mode == FW_DISABLED &&
                g_snapshot.reported_mode_reason == FW_REASON_NONE)
                return CARTPOLE_STOP_DISABLED;
            if (g_snapshot.reported_mode == FW_FAULT)
                return CARTPOLE_STOP_FAULT;
        }
        if (g_snapshot.received_state_count != state_seq &&
            (g_snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID) &&
            g_snapshot.vx_mm_s == 0 && !zero_observed) {
            zero_observed = 1;
            next_send = 0.;
        }
        nap_ms(1);
    }
    return CARTPOLE_STOP_UNCONFIRMED;
}

cartpole_result_t cartpole_reset(double timeout_s)
{
    return mode_transaction("RESET", FW_DISABLED, timeout_s);
}
cartpole_result_t cartpole_mark(cartpole_endpoint_t endpoint, double timeout_s)
{
    double end;
    TX_TRY(deadline(timeout_s, &end));
    if (endpoint != CARTPOLE_LEFT && endpoint != CARTPOLE_RIGHT)
        return CARTPOLE_INVALID_ARGUMENT;
    TX_TRY(drain(end));
    uint64_t faults = g_fault_seq;
    TX_TRY(send_simple(endpoint == CARTPOLE_LEFT ? "MARK LEFT" : "MARK RIGHT", end, 0));
    return wait_reply(end, faults);
}

static cartpole_mode_t business_mode(fw_mode_t mode)
{
    switch (mode) {
    case FW_READY:
        return CARTPOLE_MODE_READY;
    case FW_FAULT:
        return CARTPOLE_MODE_FAULT;
    default:
        return CARTPOLE_MODE_DISABLED;
    }
}
static cartpole_reason_t business_reason(fw_reason_t reason)
{
    switch (reason) {
    case FW_REASON_STEP_LOSS:
        return CARTPOLE_REASON_STEP_LOSS;
    case FW_REASON_AS5600:
        return CARTPOLE_REASON_AS5600;
    case FW_REASON_MT6835:
        return CARTPOLE_REASON_MT6835;
    default:
        return CARTPOLE_REASON_NONE;
    }
}
const char *cartpole_mode_name(cartpole_mode_t mode)
{
    switch (mode) {
    case CARTPOLE_MODE_READY:
        return "READY";
    case CARTPOLE_MODE_FAULT:
        return "FAULT";
    case CARTPOLE_MODE_DISABLED:
        return "DISABLED";
    }
    return "UNKNOWN";
}
const char *cartpole_reason_name(cartpole_reason_t reason)
{
    switch (reason) {
    case CARTPOLE_REASON_NONE:
        return "NONE";
    case CARTPOLE_REASON_STEP_LOSS:
        return "STEP_LOSS";
    case CARTPOLE_REASON_AS5600:
        return "AS5600";
    case CARTPOLE_REASON_MT6835:
        return "MT6835";
    }
    return "UNKNOWN";
}
cartpole_measurement_t cartpole_measurement(void)
{
    return (cartpole_measurement_t){.cart_valid = !!(g_snapshot.measurement_validity_bits & 1),
                                    .pole_valid = !!(g_snapshot.measurement_validity_bits & 2),
                                    .cart_position_m = g_snapshot.x_um * 1e-6,
                                    .pole_angle_rad = g_snapshot.theta_urad * 1e-6,
                                    .cart_velocity_m_s = g_snapshot.vx_mm_s * 1e-3,
                                    .pole_angular_velocity_rad_s = g_snapshot.vth_mrad_s * 1e-3,
                                    .measurement_id = g_snapshot.received_state_count,
                                    .received_host_monotonic_s =
                                        g_snapshot.last_state_received_host_monotonic_s};
}
cartpole_session_t cartpole_session(void)
{
    return (cartpole_session_t){.connection_open = g_snapshot.is_serial_open,
                                .is_mode_received = g_snapshot.is_mode_received,
                                .reported_mode = business_mode(g_snapshot.reported_mode),
                                .reported_mode_reason =
                                    business_reason(g_snapshot.reported_mode_reason),
                                .has_stream_failure_latch = g_snapshot.has_stream_failure_latch,
                                .has_received_fault_mode = g_snapshot.has_received_fault_mode};
}
cartpole_statistics_t cartpole_statistics(void)
{
    return (cartpole_statistics_t){g_snapshot.received_state_count,
                                   g_snapshot.received_unknown_line_count};
}
double cartpole_measurement_age_s(cartpole_measurement_t measurement)
{
    return measurement.received_host_monotonic_s < 0
               ? -1.
               : cartpole_host_monotonic_s() - measurement.received_host_monotonic_s;
}
cartpole_configuration_t cartpole_configuration(void)
{
    cartpole_configuration_t c = {0};
    int64_t state = 0, saved = 0, storage = -1;
    c.complete = cartpole_cached_value("CAL_STATE", &state) == 0;
    c.calibration_state = state == 2   ? CARTPOLE_CALIBRATION_READY
                          : state == 3 ? CARTPOLE_CALIBRATION_REFERENCE_REQUIRED
                                       : CARTPOLE_CALIBRATION_NOT_READY;
    c.complete &= cartpole_cached_value("CAL_LENGTH_UM", &c.track_length_um) == 0;
    c.complete &= cartpole_cached_value("CAL_SAVED", &saved) == 0;
    c.calibration_saved = saved == 1;
    c.complete &= cartpole_cached_value("CAL_STORAGE", &storage) == 0;
    c.calibration_storage_ok = storage == 0;
    c.complete &= cartpole_cached_value("VEL_MAX", &c.velocity_limit_mm_s) == 0;
    c.complete &= cartpole_cached_value("ACC_MAX", &c.acceleration_limit_mm_s2) == 0;
    c.complete &= cartpole_cached_value("WDT_MS", &c.watchdog_timeout_ms) == 0;
    c.complete &= cartpole_cached_value("X_MARGIN_UM", &c.track_margin_um) == 0;
    return c;
}
const char *cartpole_setting_name(cartpole_setting_t setting)
{
    switch (setting) {
    case CARTPOLE_SETTING_VELOCITY_LIMIT:
        return "VEL_MAX";
    case CARTPOLE_SETTING_ACCELERATION_LIMIT:
        return "ACC_MAX";
    case CARTPOLE_SETTING_WATCHDOG_TIMEOUT:
        return "WDT_MS";
    case CARTPOLE_SETTING_TRACK_MARGIN:
        return "X_MARGIN_UM";
    case CARTPOLE_SETTING_POLE_ZERO:
        return "POLE_ZERO_URAD";
    case CARTPOLE_SETTING_POLE_INVERT:
        return "POLE_INV";
    case CARTPOLE_SETTING_CART_INVERT:
        return "X_INV";
    case CARTPOLE_SETTING_STEP_LOSS_THRESHOLD:
        return "STEP_LOSS_UM";
    }
    return NULL;
}
int cartpole_setting_from_name(const char *name, cartpole_setting_t *setting)
{
    if (!name || !setting)
        return -1;
    for (int i = CARTPOLE_SETTING_VELOCITY_LIMIT; i <= CARTPOLE_SETTING_STEP_LOSS_THRESHOLD; i++) {
        if (!strcmp(name, cartpole_setting_name((cartpole_setting_t)i))) {
            *setting = (cartpole_setting_t)i;
            return 0;
        }
    }
    return -1;
}
int cartpole_callback_enter(void)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return -1;
    }
    g_callback_depth = 1;
    g_callback_violation = 0;
    return 0;
}
int cartpole_callback_leave(void)
{
    g_callback_depth = 0;
    return g_callback_violation ? -1 : 0;
}
cartpole_result_t cartpole_subscribe(cartpole_event_handler_t handler, void *context)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return CARTPOLE_CALLBACK_ERROR;
    }
    g_event_handler = handler;
    g_event_context = context;
    return CARTPOLE_OK;
}
static void deliver_event(const cartpole_event_t *event)
{
    if (!g_event_handler)
        return;
    if (cartpole_callback_enter() != 0) {
        stream_failed(CARTPOLE_CALLBACK_ERROR);
        return;
    }
    g_event_handler(event, g_event_context);
    if (cartpole_callback_leave() != 0) {
        /* 不递归派发callback自身造成的失败事件，延至下一安全边界。 */
        if (g_stream_error == CARTPOLE_OK)
            g_stream_error = CARTPOLE_CALLBACK_ERROR;
        if (!g_snapshot.has_stream_failure_latch)
            g_session_event_pending = 1;
        g_snapshot.has_stream_failure_latch = 1;
    }
}
static void flush_session_event(void)
{
    if (g_callback_depth || !g_session_event_pending)
        return;
    g_session_event_pending = 0;
    cartpole_event_t event = {.kind = CARTPOLE_EVENT_SESSION, .data.session = cartpole_session()};
    deliver_event(&event);
}
static void dispatch_business_event(const proto_msg_t *message)
{
    if (!g_event_handler)
        return;
    cartpole_event_t event;
    switch (message->type) {
    case PROTO_STATE:
        event.kind = CARTPOLE_EVENT_MEASUREMENT;
        event.data.measurement = cartpole_measurement();
        break;
    case PROTO_MODE:
        event.kind = CARTPOLE_EVENT_SESSION;
        event.data.session = cartpole_session();
        break;
    case PROTO_OK:
        event.kind = CARTPOLE_EVENT_CONFIGURATION;
        event.data.configuration = cartpole_configuration();
        break;
    case PROTO_STEP_LOSS:
        event.kind = CARTPOLE_EVENT_STEP_LOSS;
        event.data.step_loss.event_id = message->event_id;
        event.data.step_loss.run_id = message->run_id;
        event.data.step_loss.device_time_us = message->device_us;
        break;
    default:
        return;
    }
    deliver_event(&event);
}
cartpole_result_t cartpole_stream_result(void) { return g_stream_error; }
int cartpole_loop_begin(void)
{
    if (g_callback_depth) {
        g_callback_violation = 1;
        return -1;
    }
    if (g_loop_active)
        return -1;
    g_loop_active = 1;
    return 0;
}
void cartpole_loop_end(void) { g_loop_active = 0; }
void cartpole_sleep_to(double when)
{
    struct timespec ts = {(time_t)when, (long)((when - floor(when)) * 1e9)};
    while (!g_stop && clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
    }
}
cartpole_result_t cartpole_run_loop(double period_s, double duration_s,
                                    cartpole_tick_handler_t tick, void *context)
{
    if (!isfinite(period_s) || period_s <= 0 || !isfinite(duration_s) || duration_s < 0 || !tick)
        return CARTPOLE_INVALID_ARGUMENT;
    if (cartpole_loop_begin() != 0)
        return CARTPOLE_CALLBACK_ERROR;
    if (!g_snapshot.is_serial_open) {
        cartpole_loop_end();
        return CARTPOLE_SESSION_UNAVAILABLE;
    }
    double start = cartpole_host_monotonic_s(), next = start;
    cartpole_result_t result = CARTPOLE_OK;
    while (!g_stop) {
        double until = duration_s > 0 ? fmin(next, start + duration_s) : next;
        cartpole_sleep_to(until);
        if (g_stop)
            break;
        double now = cartpole_host_monotonic_s();
        if (duration_s > 0 && now - start >= duration_s)
            break;
        int received = pump(0);
        if (received < 0) {
            result = g_stream_error;
            break;
        }
        if (duration_s > 0 && cartpole_host_monotonic_s() - start >= duration_s)
            break;
        if (received == 0) {
            cartpole_tick_action_t action = {.kind = CARTPOLE_TICK_OBSERVE};
            if (cartpole_callback_enter() != 0) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
            int rc = tick(now - start, &action, context);
            int violation = cartpole_callback_leave();
            if (rc || violation) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
            if (g_stop) {
                result = CARTPOLE_CANCELLED;
                break;
            }
            if (duration_s > 0 && cartpole_host_monotonic_s() - start >= duration_s)
                break;
            if (action.kind == CARTPOLE_TICK_FINISH)
                break;
            if (action.kind == CARTPOLE_TICK_VELOCITY) {
                received = pump(0);
                if (received < 0) {
                    result = g_stream_error;
                    break;
                }
                if (duration_s > 0 && cartpole_host_monotonic_s() - start >= duration_s)
                    break;
                if (received == 0 &&
                    cartpole_send_vel_until(action.velocity_mm_s,
                                            duration_s > 0 ? start + duration_s : INFINITY) !=
                        CARTPOLE_SEND_COMPLETE) {
                    result = g_stream_error == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                           : g_stream_error;
                    break;
                }
            } else if (action.kind != CARTPOLE_TICK_OBSERVE) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
        }
        next = control_next_tick(next, cartpole_host_monotonic_s(), period_s);
    }
    if (g_stop && result == CARTPOLE_OK)
        result = CARTPOLE_CANCELLED;
    cartpole_loop_end();
    return result;
}
