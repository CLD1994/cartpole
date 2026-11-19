/* 串口设备适配器：拥有传输与协议状态，通过业务能力接口提供设备行为。 */
#include "serial_cartpole.h"
#include "cartpole_format.h"
#include "config.h"
#include "proto.h"
#include "uart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { CARTPOLE_MEASUREMENT_CART_VALID = 1u, CARTPOLE_MEASUREMENT_POLE_VALID = 2u };
typedef struct {
    int is_serial_open, is_mode_received;
    fw_mode_t reported_mode;
    fw_reason_t reported_mode_reason;
    int has_stream_failure_latch, has_received_fault_mode;
    unsigned measurement_validity_bits;
    int64_t x_um, theta_urad, vx_mm_s, vth_mrad_s;
    double last_state_received_host_monotonic_s;
    uint64_t received_state_count, received_unknown_line_count;
} serial_snapshot_t;
struct cached_value {
    char name[16];
    int64_t value;
};
typedef enum {
    STARTUP_IDLE,
    STARTUP_DISCARD_FIRST_LINE,
    STARTUP_AWAIT_STATE,
    STARTUP_COMPLETE
} startup_sync_t;
struct serial_cartpole {
    const cartpole_clock_t *clock;
    cartpole_cancellation_probe_t cancellation;
    void *cancellation_context;
    uart_t uart;
    proto_rx_t rx;
    startup_sync_t startup_sync;
    serial_snapshot_t snapshot;
    int need_frame_boundary;
    uint64_t mode_seq, fault_seq, non_disabled_seq, invalid_dual_state_seq;
    cartpole_result_t stream_error;
    char wait_name[16];
    int reply_seen;
    cartpole_result_t reply_err;
    int64_t reply_value;
    struct cached_value values[24];
    size_t value_count;
    cartpole_event_handler_t event_handler;
    void *event_context;
    int session_event_pending, delivering, configuration_changed;
};
static int adapter_cancelled(serial_cartpole_t *s)
{ return s->cancellation && s->cancellation(s->cancellation_context); }
static void bind_cancellation(void *context, cartpole_cancellation_probe_t probe, void *source)
{
    serial_cartpole_t *s = context;
    s->cancellation = probe;
    s->cancellation_context = source;
}
static double adapter_now(serial_cartpole_t *s) { return cartpole_clock_get_monotonic_time_s(s->clock); }
static double state_age(serial_cartpole_t *s)
{
    return s->snapshot.last_state_received_host_monotonic_s < 0
               ? -1.
               : adapter_now(s) - s->snapshot.last_state_received_host_monotonic_s;
}
static void nap_ms(serial_cartpole_t *s, int ms)
{
    cartpole_clock_sleep_until(s->clock, adapter_now(s) + ms * .001);
}
static void flush_session_event(serial_cartpole_t *s);
static void stream_failed(serial_cartpole_t *s, cartpole_result_t result)
{
    if (s->stream_error == CARTPOLE_OK)
        s->stream_error = result;
    if (!s->snapshot.has_stream_failure_latch)
        s->session_event_pending = 1;
    s->snapshot.has_stream_failure_latch = 1;
    if (!s->delivering)
        flush_session_event(s);
}
static void deliver_event(serial_cartpole_t *s, const cartpole_event_t *event)
{
    if (!s->event_handler)
        return;
    s->delivering = 1;
    s->event_handler(event, s->event_context);
    s->delivering = 0;
}
#define TX_TRY(expression)                                                                         \
    do {                                                                                           \
        cartpole_result_t result_ = (expression);                                                  \
        if (result_ != CARTPOLE_OK)                                                                \
            return result_;                                                                        \
    } while (0)
static cartpole_result_t adapter_checkpoint(serial_cartpole_t *s, double end, int cleanup);
static cartpole_result_t adapter_deadline(serial_cartpole_t *s, double timeout, double *end);
static cartpole_result_t adapter_write_until(serial_cartpole_t *s, const uint8_t *data, size_t n,
                                             double end, int cleanup);
static cartpole_result_t adapter_send_line(serial_cartpole_t *s, const char *line_buffer, size_t n,
                                           double end, int cleanup);
static cartpole_send_result_t adapter_try_send_vel(serial_cartpole_t *s, double v,
                                                   double deadline_host_monotonic_s);
static void adapter_await_name(serial_cartpole_t *s, const char *name);
static cartpole_send_result_t adapter_send_vel_until(serial_cartpole_t *s, double v, double end);
static cartpole_result_t adapter_send_simple(serial_cartpole_t *s, const char *cmd, double end,
                                             int cleanup);
static cartpole_result_t adapter_send_config(serial_cartpole_t *s, const char *name, long value,
                                             double end);
static void adapter_cache_value(serial_cartpole_t *s, const char *name, int64_t value);
static int adapter_cached_value(serial_cartpole_t *s, const char *name, int64_t *value);
static void adapter_handle_line(const char *line, size_t len, void *user);
static int adapter_pump(serial_cartpole_t *s, int cleanup);
static cartpole_result_t adapter_receive(serial_cartpole_t *s, double end);
static cartpole_result_t adapter_drain(serial_cartpole_t *s, double end);
static cartpole_result_t adapter_query_mode(serial_cartpole_t *s, double timeout_s);
static cartpole_result_t adapter_wait_measurement(serial_cartpole_t *s, double timeout_s);
static cartpole_result_t adapter_wait_reply(serial_cartpole_t *s, double end, uint64_t faults);
static cartpole_result_t adapter_configure(serial_cartpole_t *s, cartpole_setting_t setting,
                                           long value, double timeout_s,
                                           cartpole_configure_observer_t observer, void *context);
static cartpole_result_t adapter_refresh_configuration(serial_cartpole_t *s, double timeout_s);
static cartpole_result_t adapter_mode_transaction(serial_cartpole_t *s, const char *command,
                                                  fw_mode_t target, double timeout_s);
static cartpole_result_t adapter_enable(serial_cartpole_t *s, double timeout_s);
static cartpole_stop_result_t adapter_stop_and_confirm(serial_cartpole_t *s, double timeout_s);
static cartpole_result_t adapter_reset(serial_cartpole_t *s, double timeout_s);
static cartpole_result_t adapter_mark(serial_cartpole_t *s, cartpole_endpoint_t endpoint,
                                      int64_t *value_um, double timeout_s);
static cartpole_mode_t adapter_business_mode(fw_mode_t mode);
static cartpole_reason_t adapter_business_reason(fw_reason_t reason);
static cartpole_measurement_t adapter_measurement(serial_cartpole_t *s);
static cartpole_session_t adapter_session(serial_cartpole_t *s);
static cartpole_configuration_t adapter_configuration(serial_cartpole_t *s);
static void adapter_dispatch_business_event(serial_cartpole_t *s, const proto_msg_t *message);
static const char *configuration_parameters[] = {
    CARTPOLE_TEXT_CAL_STATE, CARTPOLE_TEXT_CAL_LENGTH_UM, CARTPOLE_TEXT_CAL_SAVED,
    CARTPOLE_TEXT_CAL_STORAGE, CARTPOLE_TEXT_VEL_MAX, CARTPOLE_TEXT_ACC_MAX,
    CARTPOLE_TEXT_WDT_MS, CARTPOLE_TEXT_X_MARGIN_UM
};
static int configuration_parameter(const char *name)
{
    for (size_t i = 0; i < sizeof configuration_parameters / sizeof configuration_parameters[0];
         ++i)
        if (!strcmp(name, configuration_parameters[i]))
            return 1;
    return 0;
}
static cartpole_result_t adapter_checkpoint(serial_cartpole_t *s, double end, int cleanup)
{
    if (!cleanup && adapter_cancelled(s))
        return CARTPOLE_CANCELLED;
    if (adapter_now(s) >= end)
        return CARTPOLE_TIMEOUT;
    return CARTPOLE_OK;
}
static cartpole_result_t adapter_deadline(serial_cartpole_t *s, double timeout, double *end)
{
    if (!isfinite(timeout) || timeout <= 0.)
        return CARTPOLE_INVALID_ARGUMENT;
    *end = adapter_now(s) + timeout;
    if (!isfinite(*end))
        return CARTPOLE_INVALID_ARGUMENT;
    return CARTPOLE_OK;
}
static cartpole_result_t adapter_write_until(serial_cartpole_t *s, const uint8_t *data, size_t n,
                                             double end, int cleanup)
{
    for (;;) {
        cartpole_result_t result = adapter_checkpoint(s, end, cleanup);
        if (result != CARTPOLE_OK)
            return result;
        int written = uart_try_write_all(&s->uart, data, n);
        if (written < 0) {
            stream_failed(s, CARTPOLE_TRANSPORT_ERROR);
            s->need_frame_boundary = 1;
            return CARTPOLE_TRANSPORT_ERROR;
        }
        if (written == 0)
            return adapter_checkpoint(s, end, cleanup);
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_send_line(serial_cartpole_t *s, const char *line_buffer, size_t n,
                                           double end, int cleanup)
{
    if (!line_buffer || !n)
        return CARTPOLE_INVALID_ARGUMENT;
    cartpole_result_t result = adapter_checkpoint(s, end, cleanup);
    if (result != CARTPOLE_OK)
        return result;
    if (s->need_frame_boundary) {
        result = adapter_write_until(s, (const uint8_t *)"\n", 1, end, cleanup);
        if (result != CARTPOLE_OK)
            return result;
        s->need_frame_boundary = 0;
    }
    return adapter_write_until(s, (const uint8_t *)line_buffer, n, end, cleanup);
}
static cartpole_send_result_t adapter_try_send_vel(serial_cartpole_t *s, double v,
                                                   double deadline_host_monotonic_s)
{
    char line[64];
    size_t n = proto_format_vel(line, v);
    if (!n || !isfinite(deadline_host_monotonic_s) || adapter_cancelled(s) ||
        s->snapshot.has_stream_failure_latch || !s->snapshot.is_mode_received ||
        s->snapshot.reported_mode != FW_READY ||
        s->snapshot.reported_mode_reason != FW_REASON_NONE ||
        !(s->snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID))
        return CARTPOLE_SEND_ERROR;
    double now = adapter_now(s);
    if (now >= deadline_host_monotonic_s)
        return CARTPOLE_SEND_SKIPPED;
    if (s->snapshot.last_state_received_host_monotonic_s < 0 ||
        now - s->snapshot.last_state_received_host_monotonic_s > CFG_STATE_TIMEOUT_S)
        return CARTPOLE_SEND_ERROR;
    int result = uart_try_write_all(&s->uart, (const uint8_t *)line, n);
    if (result < 0) {
        stream_failed(s, CARTPOLE_TRANSPORT_ERROR);
        s->need_frame_boundary = 1;
        return CARTPOLE_SEND_ERROR;
    }
    if (result == 0 && adapter_now(s) >= deadline_host_monotonic_s)
        return CARTPOLE_SEND_LATE; /* 已完整写入，不能撤销；调用方终止控制并执行停止流程。
                                    */
    return result == 0 ? CARTPOLE_SEND_COMPLETE : CARTPOLE_SEND_SKIPPED;
}
static void adapter_await_name(serial_cartpole_t *s, const char *name)
{
    snprintf(s->wait_name, sizeof s->wait_name, "%s", name);
    s->reply_seen = 0;
    s->reply_err = CARTPOLE_OK;
    s->reply_value = 0;
}
static cartpole_send_result_t adapter_send_vel_until(serial_cartpole_t *s, double v, double end)
{
    char line[64];
    size_t length = proto_format_vel(line, v);
    if (!length)
        return CARTPOLE_SEND_ERROR;
    /* 零速清理可在错误/取消后执行；非零维护发送失败会锁住后续非零发送。 */
    if (v != 0. &&
        (adapter_cancelled(s) || s->snapshot.has_stream_failure_latch ||
         !s->snapshot.is_mode_received || s->snapshot.reported_mode != FW_READY ||
         s->snapshot.reported_mode_reason != FW_REASON_NONE ||
         !(s->snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID) ||
         state_age(s) < 0 || state_age(s) > CFG_STATE_TIMEOUT_S))
        return CARTPOLE_SEND_ERROR;
    cartpole_result_t result =
        adapter_send_line(s, line, length, fmin(end, adapter_now(s) + .020), v == 0.);
    if (result != CARTPOLE_OK && v != 0.)
        stream_failed(s, result);
    return result == CARTPOLE_OK ? CARTPOLE_SEND_COMPLETE : CARTPOLE_SEND_ERROR;
}
static cartpole_result_t adapter_send_simple(serial_cartpole_t *s, const char *cmd, double end,
                                             int cleanup)
{
    char line_buffer[64];
    if (!strcmp(cmd, "MARK LEFT"))
        adapter_await_name(s, "MARK_LEFT");
    else if (!strcmp(cmd, "MARK RIGHT"))
        adapter_await_name(s, "MARK_RIGHT");
    else if (!strcmp(cmd, "CONFIRM LEFT"))
        adapter_await_name(s, CARTPOLE_TEXT_CONFIRM_LEFT);
    else if (!strcmp(cmd, "ENABLE"))
        adapter_await_name(s, "ENABLE");
    else if (!strcmp(cmd, "RESET"))
        adapter_await_name(s, "RESET");
    else if (!strcmp(cmd, "DISABLE"))
        adapter_await_name(s, "DISABLE");
    return adapter_send_line(s, line_buffer, proto_format_simple(line_buffer, cmd), end, cleanup);
}
static cartpole_result_t adapter_send_config(serial_cartpole_t *s, const char *name, long value,
                                             double end)
{
    char line_buffer[64];
    adapter_await_name(s, name);
    return adapter_send_line(s, line_buffer, proto_format_config(line_buffer, name, value), end, 0);
}
static void adapter_cache_value(serial_cartpole_t *s, const char *name, int64_t value)
{
    size_t i = 0;
    for (; i < s->value_count; i++)
        if (!strcmp(s->values[i].name, name))
            break;
    if (i == s->value_count) {
        if (i >= sizeof s->values / sizeof s->values[0])
            return;
        s->value_count++;
        s->configuration_changed = 1;
    }
    snprintf(s->values[i].name, sizeof s->values[i].name, "%s", name);
    if (s->values[i].value != value)
        s->configuration_changed = 1;
    s->values[i].value = value;
}
static int adapter_cached_value(serial_cartpole_t *s, const char *name, int64_t *value)
{
    if (!name || !name[0])
        return -1;
    for (size_t i = 0; i < s->value_count; i++)
        if (!strcmp(name, s->values[i].name)) {
            if (value)
                *value = s->values[i].value;
            return 0;
        }
    return -1;
}
static cartpole_result_t adapter_special_reply_error(const char *awaiting, const char *reported)
{
    if (!strcmp(reported, CARTPOLE_TEXT_STORAGE_ERROR) &&
        (!strcmp(awaiting, CARTPOLE_TEXT_POLE_ZERO_URAD) ||
         !strcmp(awaiting, CARTPOLE_TEXT_POLE_INV) || !strcmp(awaiting, "MARK_LEFT") ||
         !strcmp(awaiting, "MARK_RIGHT")))
        return CARTPOLE_STORAGE_ERROR;
    if (!strcmp(awaiting, "MARK_RIGHT") && !strcmp(reported, CARTPOLE_TEXT_CALIBRATION_REFERENCE_ERROR))
        return CARTPOLE_CALIBRATION_REFERENCE_ERROR;
    if (!strcmp(awaiting, "ENABLE") && !strcmp(reported, "POWER"))
        return CARTPOLE_POWER_UNAVAILABLE;
    if (!strcmp(awaiting, "DISABLE") && !strcmp(reported, "BUSY"))
        return CARTPOLE_BUSY;
    return CARTPOLE_OK;
}
static void adapter_handle_line(const char *line, size_t len, void *user)
{
    serial_cartpole_t *s = user;
    proto_msg_t m;
    if (s->startup_sync == STARTUP_DISCARD_FIRST_LINE) {
        s->startup_sync = STARTUP_AWAIT_STATE;
        return;
    }
    s->configuration_changed = 0;
    if (proto_parse_line(line, &m) == PROTO_UNKNOWN) {
        if (s->startup_sync == STARTUP_AWAIT_STATE)
            return;
        s->snapshot.received_unknown_line_count++;
        if (len) {
            const cartpole_event_t event = {.kind = CARTPOLE_EVENT_UNRECOGNIZED_LINE};
            deliver_event(s, &event);
        }
        if (!strncmp(line, "STATE", 5) || !strncmp(line, "MODE", 4) || !strcmp(line, "!OVERLONG"))
            stream_failed(s, CARTPOLE_PROTOCOL_ERROR);
        return;
    }
    switch (m.type) {
    case PROTO_STATE:
        if (s->startup_sync == STARTUP_AWAIT_STATE)
            s->startup_sync = STARTUP_COMPLETE;
        s->snapshot.measurement_validity_bits = m.valid_flags;
        if (m.valid_flags != (CARTPOLE_MEASUREMENT_CART_VALID | CARTPOLE_MEASUREMENT_POLE_VALID))
            s->invalid_dual_state_seq++;
        s->snapshot.x_um = m.x_um;
        s->snapshot.theta_urad = m.theta_urad;
        s->snapshot.vx_mm_s = m.vx_mm_s;
        s->snapshot.vth_mrad_s = m.vth_mrad_s;
        s->snapshot.last_state_received_host_monotonic_s = adapter_now(s);
        s->snapshot.received_state_count++;
        break;
    case PROTO_MODE:
        s->snapshot.reported_mode = m.mode;
        s->snapshot.reported_mode_reason = m.reason;
        s->snapshot.is_mode_received = 1;
        s->mode_seq++;
        if (m.mode != FW_DISABLED || m.reason != FW_REASON_NONE)
            s->non_disabled_seq++;
        if (m.mode == FW_FAULT) {
            s->snapshot.has_received_fault_mode = 1;
            s->fault_seq++;
        }
        break;
    case PROTO_OK:
    case PROTO_ERR:
        if (m.type == PROTO_OK)
            adapter_cache_value(s, m.param, m.value);
        if (!s->reply_seen && s->wait_name[0]) {
            cartpole_result_t special =
                m.type == PROTO_ERR ? adapter_special_reply_error(s->wait_name, m.param) : CARTPOLE_OK;
            if (special != CARTPOLE_OK || !strcmp(s->wait_name, m.param)) {
                s->reply_seen = 1;
                s->reply_err = special != CARTPOLE_OK ? special
                               : m.type == PROTO_ERR ? CARTPOLE_REJECTED : CARTPOLE_OK;
                s->reply_value = m.value;
            }
        }
        break;
    case PROTO_STEP_LOSS:
        break;
    default:
        break;
    }
    adapter_dispatch_business_event(s, &m);
}
static int adapter_pump(serial_cartpole_t *s, int cleanup)
{
    flush_session_event(s);
    uint8_t buf[512];
    for (size_t bytes = 0; bytes < 4096;) {
        int n = uart_read(&s->uart, buf, sizeof buf);
        if (n < 0) {
            stream_failed(s, CARTPOLE_TRANSPORT_ERROR);
            return -1;
        }
        if (n == 0)
            return !cleanup && s->snapshot.has_stream_failure_latch ? -1 : 0;
        bytes += (size_t)n;
        proto_rx_feed(&s->rx, buf, (size_t)n, adapter_handle_line, s);
        if (!cleanup && s->snapshot.has_stream_failure_latch)
            return -1;
    }
    return 1;
}
static cartpole_result_t adapter_receive(serial_cartpole_t *s, double end)
{
    cartpole_result_t result = adapter_checkpoint(s, end, 0);
    if (result != CARTPOLE_OK)
        return result;
    if (adapter_pump(s, 0) < 0)
        return s->stream_error;
    return adapter_checkpoint(s, end, 0);
}
static cartpole_result_t adapter_drain(serial_cartpole_t *s, double end)
{
    for (;;) {
        cartpole_result_t result = adapter_checkpoint(s, end, 0);
        if (result != CARTPOLE_OK)
            return result;
        int pumped = adapter_pump(s, 0);
        if (pumped < 0)
            return s->stream_error;
        result = adapter_checkpoint(s, end, 0);
        if (result != CARTPOLE_OK)
            return result;
        if (pumped == 0)
            return CARTPOLE_OK;
    }
}
static cartpole_result_t adapter_query_mode(serial_cartpole_t *s, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_drain(s, end));
    uint64_t seq = s->mode_seq;
    TX_TRY(adapter_send_simple(s, "QUERY", end, 0));
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        if (s->mode_seq != seq)
            return CARTPOLE_OK;
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_wait_measurement(serial_cartpole_t *s, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    uint64_t seq = s->snapshot.received_state_count;
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        if (s->snapshot.received_state_count != seq)
            return CARTPOLE_OK;
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_wait_reply(serial_cartpole_t *s, double end, uint64_t faults)
{
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        if (s->reply_seen && s->reply_err)
            return s->reply_err;
        if (s->fault_seq != faults)
            return CARTPOLE_FAULT;
        if (s->reply_seen)
            return CARTPOLE_OK;
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_send_checked_config(serial_cartpole_t *s, const char *name,
                                                      const char *line, size_t n, double end,
                                                      cartpole_configure_observer_t observer,
                                                      void *context)
{
    if (s->need_frame_boundary) {
        TX_TRY(adapter_write_until(s, (const uint8_t *)"\n", 1, end, 0));
        s->need_frame_boundary = 0;
    }
    for (;;) {
        /* 背压期间也要接收和核验；上一次检查不授权下一次写入。 */
        TX_TRY(adapter_drain(s, end));
        TX_TRY(observer(CARTPOLE_CONFIGURE_CHECK, context));
        TX_TRY(adapter_checkpoint(s, end, 0));
        adapter_await_name(s, name);
        const int written = uart_try_write_all(&s->uart, (const uint8_t *)line, n);
        const cartpole_result_t observed = written <= 0
            ? observer(CARTPOLE_CONFIGURE_ATTEMPTED, context) : CARTPOLE_OK;
        if (written < 0) {
            stream_failed(s, CARTPOLE_TRANSPORT_ERROR);
            s->need_frame_boundary = 1;
            return CARTPOLE_TRANSPORT_ERROR;
        }
        if (written == 0)
            return observed != CARTPOLE_OK ? observed : adapter_checkpoint(s, end, 0);
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_configure(serial_cartpole_t *s, cartpole_setting_t setting,
                                           long value, double timeout_s,
                                           cartpole_configure_observer_t observer, void *context)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    const char *name = cartpole_setting_to_string(setting);
    char line[64];
    const size_t n = name ? proto_format_config(line, name, value) : 0;
    if (!n || strlen(name) >= sizeof s->wait_name)
        return CARTPOLE_INVALID_ARGUMENT;
    TX_TRY(adapter_drain(s, end));
    uint64_t faults = s->fault_seq;
    if (observer)
        TX_TRY(adapter_send_checked_config(s, name, line, n, end, observer, context));
    else
        TX_TRY(adapter_send_config(s, name, value, end));
    TX_TRY(adapter_wait_reply(s, end, faults));
    return s->reply_value == value ? CARTPOLE_OK : CARTPOLE_REPLY_MISMATCH;
}
/* 批量查询没有事务号。发送前排空旧输入，只清除本次查询涉及的缓存组。 */
static cartpole_result_t adapter_begin_metadata_query(serial_cartpole_t *s, double end,
                                                     int include_calibration, int include_settings)
{
    TX_TRY(adapter_drain(s, end));
    size_t kept = 0;
    for (size_t i = 0; i < s->value_count; i++) {
        const char *name = s->values[i].name;
        cartpole_setting_t setting;
        int is_setting = cartpole_parse_setting(name, &setting) == 0;
        if ((is_setting && include_settings) ||
            (!is_setting && include_calibration && configuration_parameter(name)))
            continue;
        s->values[kept++] = s->values[i];
    }
    s->value_count = kept;
    /* 没有逐项查询的等待名；缺少应答直到 TIMEOUT。 */
    adapter_await_name(s, "");
    if (include_calibration)
        TX_TRY(adapter_send_simple(s, "CALIB?", end, 0));
    if (include_settings)
        TX_TRY(adapter_send_simple(s, "CONFIG?", end, 0));
    return CARTPOLE_OK;
}
static cartpole_result_t adapter_refresh_configuration(serial_cartpole_t *s, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_begin_metadata_query(s, end, 1, 1));
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        size_t i;
        for (i = 0; i < sizeof configuration_parameters / sizeof configuration_parameters[0]; i++)
            if (adapter_cached_value(s, configuration_parameters[i], NULL) != 0)
                break;
        if (i == sizeof configuration_parameters / sizeof configuration_parameters[0])
            return CARTPOLE_OK;
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_read_settings(serial_cartpole_t *s, cartpole_settings_t *out,
                                               uint32_t *received_mask, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_begin_metadata_query(s, end, 0, 1));
    for (;;) {
        cartpole_result_t result = adapter_receive(s, end);
        cartpole_settings_t settings;
        uint32_t received = 0;
        for (int i = 0; i < CARTPOLE_SETTING_COUNT; i++)
            if (adapter_cached_value(s, cartpole_setting_to_string(i), &settings.values[i]) == 0)
                received |= UINT32_C(1) << i;
        *received_mask = received;
        if (result != CARTPOLE_OK)
            return result;
        if (received == ((UINT32_C(1) << CARTPOLE_SETTING_COUNT) - 1)) {
            *out = settings;
            return CARTPOLE_OK;
        }
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_read_pole_settings(serial_cartpole_t *s,
                                                   cartpole_pole_settings_t *out, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    /* 固件用 CONFIG? 广播参数。这里只等待摆杆零点和方向，不依赖其他参数。 */
    TX_TRY(adapter_begin_metadata_query(s, end, 0, 1));
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        cartpole_pole_settings_t settings;
        if (adapter_cached_value(s, CARTPOLE_TEXT_POLE_ZERO_URAD, &settings.zero_urad) == 0 &&
            adapter_cached_value(s, CARTPOLE_TEXT_POLE_INV, &settings.inverted) == 0) {
            *out = settings;
            return CARTPOLE_OK;
        }
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_wait_mode(serial_cartpole_t *s, fw_mode_t target, double end,
                                           uint64_t seq, uint64_t faults)
{
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        if (s->reply_seen && s->reply_err)
            return s->reply_err;
        if (s->fault_seq != faults)
            return CARTPOLE_FAULT;
        if (s->mode_seq != seq && s->snapshot.reported_mode == target &&
            s->snapshot.reported_mode_reason == FW_REASON_NONE)
            return CARTPOLE_OK;
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_mode_transaction(serial_cartpole_t *s, const char *command,
                                                  fw_mode_t target, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_drain(s, end));
    uint64_t seq = s->mode_seq, faults = s->fault_seq;
    TX_TRY(adapter_send_simple(s, command, end, 0));
    return adapter_wait_mode(s, target, end, seq, faults);
}

static int enable_mode_is_disabled(const serial_cartpole_t *s)
{
    return s->snapshot.is_mode_received && s->snapshot.reported_mode == FW_DISABLED &&
           s->snapshot.reported_mode_reason == FW_REASON_NONE;
}

static int enable_state_is_fresh(serial_cartpole_t *s)
{
    const double age = state_age(s);
    return s->snapshot.measurement_validity_bits ==
               (CARTPOLE_MEASUREMENT_CART_VALID | CARTPOLE_MEASUREMENT_POLE_VALID) &&
           age >= 0. && age <= CFG_STATE_TIMEOUT_S;
}

static cartpole_result_t adapter_enable(serial_cartpole_t *s, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    if (s->snapshot.is_mode_received && s->snapshot.reported_mode == FW_FAULT)
        return CARTPOLE_FAULT;
    if (!enable_mode_is_disabled(s))
        return CARTPOLE_REJECTED;

    const uint64_t faults = s->fault_seq, non_disabled = s->non_disabled_seq,
                   invalid_states = s->invalid_dual_state_seq;
    int have_valid_state = enable_state_is_fresh(s);
    /* 每次真正写前先处理已有输入；UART 接受零字节时，下一次尝试重新检查。 */
    for (;;) {
        cartpole_result_t result = adapter_drain(s, end);
        if (s->fault_seq != faults)
            return CARTPOLE_FAULT;
        if (result == CARTPOLE_TIMEOUT && !have_valid_state)
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        if (result != CARTPOLE_OK)
            return result;
        if (s->non_disabled_seq != non_disabled || !enable_mode_is_disabled(s))
            return CARTPOLE_REJECTED;
        if (s->invalid_dual_state_seq != invalid_states)
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        if (!enable_state_is_fresh(s)) {
            if (have_valid_state)
                return CARTPOLE_MEASUREMENT_UNAVAILABLE;
            nap_ms(s, 1);
            continue;
        }
        have_valid_state = 1;
        TX_TRY(adapter_checkpoint(s, end, 0));
        if (!enable_state_is_fresh(s))
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        /* 前一帧部分写入会锁住流，不允许用 ENABLE 修复帧边界。 */
        if (s->need_frame_boundary)
            return CARTPOLE_TRANSPORT_ERROR;
        adapter_await_name(s, "ENABLE");
        uint64_t seq = s->mode_seq;
        const char command[] = "ENABLE\n";
        int written = uart_try_write_all(&s->uart, (const uint8_t *)command, sizeof command - 1);
        if (written < 0) {
            stream_failed(s, CARTPOLE_TRANSPORT_ERROR);
            s->need_frame_boundary = 1;
            return CARTPOLE_TRANSPORT_ERROR;
        }
        if (written == 0) {
            TX_TRY(adapter_checkpoint(s, end, 0));
            return adapter_wait_mode(s, FW_READY, end, seq, faults);
        }
        nap_ms(s, 1);
    }
}
static cartpole_result_t adapter_disable(serial_cartpole_t *s, double timeout_s)
{
    return adapter_mode_transaction(s, "DISABLE", FW_DISABLED, timeout_s);
}
static cartpole_result_t adapter_read_calibration_status(serial_cartpole_t *s,
                                                          cartpole_calibration_status_t *out,
                                                          double timeout_s)
{
    if (!out)
        return CARTPOLE_INVALID_ARGUMENT;
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_begin_metadata_query(s, end, 1, 0));
    for (;;) {
        TX_TRY(adapter_receive(s, end));
        cartpole_calibration_status_t status;
        if (adapter_cached_value(s, CARTPOLE_TEXT_CAL_STATE, &status.state) == 0 &&
            adapter_cached_value(s, CARTPOLE_TEXT_CAL_LENGTH_UM, &status.length_um) == 0 &&
            adapter_cached_value(s, CARTPOLE_TEXT_CAL_SAVED, &status.saved) == 0 &&
            adapter_cached_value(s, CARTPOLE_TEXT_CAL_STORAGE, &status.storage) == 0) {
            *out = status;
            return CARTPOLE_OK;
        }
        nap_ms(s, 1);
    }
}
static cartpole_stop_result_t adapter_stop_and_confirm(serial_cartpole_t *s, double timeout_s)
{
    double end;
    if (adapter_deadline(s, timeout_s, &end) != CARTPOLE_OK)
        return CARTPOLE_STOP_UNCONFIRMED;
    /* 清理读边界不受停止标记或此前协议错误锁存阻挡。旧MODE不能证明本次停止。 */
    int drained;
    do {
        drained = adapter_pump(s, 1);
    } while (drained == 1 && adapter_now(s) < end);
    if (drained != 0)
        return CARTPOLE_STOP_UNCONFIRMED;
    uint64_t mode_seq = s->mode_seq, state_seq = s->snapshot.received_state_count;
    double next_send = 0.;
    int zero_observed = 0;
    while (adapter_now(s) < end) {
        if (adapter_now(s) >= next_send) {
            (void)adapter_send_simple(s, "VEL 0", end, 1);
            if (zero_observed)
                (void)adapter_send_simple(s, "DISABLE", end, 1);
            (void)adapter_send_simple(s, "QUERY", end, 1);
            next_send = adapter_now(s) + .02;
        }
        if (adapter_pump(s, 1) < 0 || adapter_now(s) >= end)
            return CARTPOLE_STOP_UNCONFIRMED;
        if (s->mode_seq != mode_seq) {
            if (s->snapshot.reported_mode == FW_DISABLED &&
                s->snapshot.reported_mode_reason == FW_REASON_NONE)
                return CARTPOLE_STOP_DISABLED;
            if (s->snapshot.reported_mode == FW_FAULT)
                return CARTPOLE_STOP_FAULT;
        }
        if (s->snapshot.received_state_count != state_seq &&
            (s->snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID) &&
            s->snapshot.vx_mm_s == 0 && !zero_observed) {
            zero_observed = 1;
            next_send = 0.;
        }
        nap_ms(s, 1);
    }
    return CARTPOLE_STOP_UNCONFIRMED;
}
static cartpole_result_t adapter_reset(serial_cartpole_t *s, double timeout_s)
{
    return adapter_mode_transaction(s, "RESET", FW_DISABLED, timeout_s);
}
static cartpole_result_t adapter_mark(serial_cartpole_t *s, cartpole_endpoint_t endpoint,
                                      int64_t *value_um, double timeout_s)
{
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    if (endpoint != CARTPOLE_LEFT && endpoint != CARTPOLE_RIGHT)
        return CARTPOLE_INVALID_ARGUMENT;
    TX_TRY(adapter_drain(s, end));
    uint64_t faults = s->fault_seq;
    TX_TRY(adapter_send_simple(s, endpoint == CARTPOLE_LEFT ? "MARK LEFT" : "MARK RIGHT", end, 0));
    TX_TRY(adapter_wait_reply(s, end, faults));
    *value_um = s->reply_value;
    return CARTPOLE_OK;
}
static cartpole_result_t adapter_confirm_left(serial_cartpole_t *s, int64_t *position_um,
                                               double timeout_s)
{
    if (!position_um)
        return CARTPOLE_INVALID_ARGUMENT;
    double end;
    TX_TRY(adapter_deadline(s, timeout_s, &end));
    TX_TRY(adapter_drain(s, end));
    uint64_t faults = s->fault_seq;
    TX_TRY(adapter_send_simple(s, "CONFIRM LEFT", end, 0));
    TX_TRY(adapter_wait_reply(s, end, faults));
    *position_um = s->reply_value;
    return CARTPOLE_OK;
}
static cartpole_mode_t adapter_business_mode(fw_mode_t mode)
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
static cartpole_reason_t adapter_business_reason(fw_reason_t reason)
{
    switch (reason) {
    case FW_REASON_STEP_LOSS:
        return CARTPOLE_REASON_STEP_LOSS;
    case FW_REASON_AS5600:
        return CARTPOLE_REASON_AS5600;
    case FW_REASON_MT6835:
        return CARTPOLE_REASON_MT6835;
    case FW_REASON_POWER:
        return CARTPOLE_REASON_POWER;
    default:
        return CARTPOLE_REASON_NONE;
    }
}
static cartpole_measurement_t adapter_measurement(serial_cartpole_t *s)
{
    return (cartpole_measurement_t){.cart_valid = !!(s->snapshot.measurement_validity_bits & 1),
                                    .pole_valid = !!(s->snapshot.measurement_validity_bits & 2),
                                    .cart_position_m = s->snapshot.x_um * 1e-6,
                                    .pole_angle_rad = s->snapshot.theta_urad * 1e-6,
                                    .cart_velocity_m_s = s->snapshot.vx_mm_s * 1e-3,
                                    .pole_angular_velocity_rad_s = s->snapshot.vth_mrad_s * 1e-3,
                                    .measurement_id = s->snapshot.received_state_count,
                                    .received_host_monotonic_s =
                                        s->snapshot.last_state_received_host_monotonic_s};
}
static cartpole_session_t adapter_session(serial_cartpole_t *s)
{
    return (cartpole_session_t){.connection_open = s->snapshot.is_serial_open,
                                .is_mode_received = s->snapshot.is_mode_received,
                                .reported_mode = adapter_business_mode(s->snapshot.reported_mode),
                                .reported_mode_reason =
                                    adapter_business_reason(s->snapshot.reported_mode_reason),
                                .has_stream_failure_latch = s->snapshot.has_stream_failure_latch,
                                .has_received_fault_mode = s->snapshot.has_received_fault_mode,
                                .received_fault_mode_count = s->fault_seq};
}
static cartpole_configuration_t adapter_configuration(serial_cartpole_t *s)
{
    cartpole_configuration_t c = {0};
    int64_t state = 0, saved = 0, storage = -1;
    c.complete = adapter_cached_value(s, CARTPOLE_TEXT_CAL_STATE, &state) == 0;
    c.calibration_state = state == 2   ? CARTPOLE_CALIBRATION_READY
                          : state == 3 ? CARTPOLE_CALIBRATION_REFERENCE_REQUIRED
                                       : CARTPOLE_CALIBRATION_NOT_READY;
    c.complete &= adapter_cached_value(s, CARTPOLE_TEXT_CAL_LENGTH_UM, &c.track_length_um) == 0;
    c.complete &= adapter_cached_value(s, CARTPOLE_TEXT_CAL_SAVED, &saved) == 0;
    c.calibration_saved = saved == 1;
    c.complete &= adapter_cached_value(s, CARTPOLE_TEXT_CAL_STORAGE, &storage) == 0;
    c.calibration_storage_ok = storage == 0;
    c.complete &= adapter_cached_value(s, "VEL_MAX", &c.velocity_limit_mm_s) == 0;
    c.complete &= adapter_cached_value(s, "ACC_MAX", &c.acceleration_limit_mm_s2) == 0;
    c.complete &= adapter_cached_value(s, "WDT_MS", &c.watchdog_timeout_ms) == 0;
    c.complete &= adapter_cached_value(s, "X_MARGIN_UM", &c.track_margin_um) == 0;
    c.has_legacy_watchdog_deceleration = adapter_cached_value(s, "WDT_DEC", NULL) == 0;
    return c;
}
static void adapter_dispatch_business_event(serial_cartpole_t *s, const proto_msg_t *message)
{
    if (!s->event_handler)
        return;
    cartpole_event_t event;
    switch (message->type) {
    case PROTO_STATE:
        event.kind = CARTPOLE_EVENT_MEASUREMENT;
        event.data.measurement = adapter_measurement(s);
        break;
    case PROTO_MODE:
        event.kind = CARTPOLE_EVENT_SESSION;
        event.data.session = adapter_session(s);
        break;
    case PROTO_OK:
        if (!s->configuration_changed ||
            (!configuration_parameter(message->param) && strcmp(message->param, "WDT_DEC") != 0))
            return;
        event.kind = CARTPOLE_EVENT_CONFIGURATION;
        event.data.configuration = adapter_configuration(s);
        break;
    case PROTO_ERR:
        event.kind = CARTPOLE_EVENT_FIRMWARE_ERROR;
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
    deliver_event(s, &event);
}

static void flush_session_event(serial_cartpole_t *s)
{
    if (s->delivering || !s->session_event_pending)
        return;
    s->session_event_pending = 0;
    cartpole_event_t event = {.kind = CARTPOLE_EVENT_SESSION, .data.session = adapter_session(s)};
    deliver_event(s, &event);
}
static cartpole_measurement_t port_get_latest_measurement(void *context)
{
    serial_cartpole_t *s = context;
    return adapter_measurement(s);
}
static cartpole_session_t port_get_session_state(void *context)
{
    serial_cartpole_t *s = context;
    return adapter_session(s);
}
static cartpole_configuration_t port_get_configuration(void *context)
{
    serial_cartpole_t *s = context;
    return adapter_configuration(s);
}
static cartpole_receive_result_t port_receive(void *context)
{
    serial_cartpole_t *s = context;
    return (cartpole_receive_result_t)adapter_pump(s, 0);
}
static cartpole_result_t port_get_error(void *context)
{
    serial_cartpole_t *s = context;
    return s->stream_error;
}
static cartpole_result_t port_query_mode(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_query_mode(s, timeout);
}
static cartpole_result_t port_wait_measurement(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_wait_measurement(s, timeout);
}
static cartpole_result_t port_enable(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_enable(s, timeout);
}
static cartpole_result_t port_disable(void *context, double timeout)
{
    return adapter_disable(context, timeout);
}
static cartpole_result_t port_reset(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_reset(s, timeout);
}
static cartpole_result_t port_refresh_configuration(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_refresh_configuration(s, timeout);
}
static cartpole_result_t port_read_pole_settings(void *context, cartpole_pole_settings_t *out,
                                                double timeout)
{
    return adapter_read_pole_settings(context, out, timeout);
}
static cartpole_result_t port_read_settings(void *context, cartpole_settings_t *out,
                                             uint32_t *received_mask, double timeout)
{
    return adapter_read_settings(context, out, received_mask, timeout);
}
static cartpole_result_t port_mark(void *context, cartpole_endpoint_t endpoint,
                                  int64_t *value_um, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_mark(s, endpoint, value_um, timeout);
}
static cartpole_result_t port_confirm_left(void *context, int64_t *position_um, double timeout)
{
    return adapter_confirm_left(context, position_um, timeout);
}
static cartpole_result_t port_read_calibration_status(void *context,
                                                      cartpole_calibration_status_t *out,
                                                      double timeout)
{
    return adapter_read_calibration_status(context, out, timeout);
}
static cartpole_result_t port_configure(void *context, cartpole_setting_t setting, long value,
                                        double timeout, cartpole_configure_observer_t observer,
                                        void *observer_context)
{
    serial_cartpole_t *s = context;
    return adapter_configure(s, setting, value, timeout, observer, observer_context);
}
static cartpole_stop_result_t port_stop_and_confirm(void *context, double timeout)
{
    serial_cartpole_t *s = context;
    return adapter_stop_and_confirm(s, timeout);
}
static cartpole_result_t port_attach_observer(void *context, cartpole_event_handler_t handler,
                                              void *user)
{
    serial_cartpole_t *s = context;
    if (!handler)
        return CARTPOLE_INVALID_ARGUMENT;
    if (s->event_handler)
        return CARTPOLE_REJECTED;
    s->event_handler = handler;
    s->event_context = user;
    return CARTPOLE_OK;
}
static void port_detach_observer(void *context)
{
    serial_cartpole_t *s = context;
    s->event_handler = NULL;
    s->event_context = NULL;
}
static void port_report_failure(void *context, cartpole_result_t result)
{
    stream_failed(context, result);
}
static cartpole_send_result_t port_send_velocity(void *context, double velocity, double deadline,
                                                 int try_once)
{
    serial_cartpole_t *s = context;
    return try_once ? adapter_try_send_vel(s, velocity, deadline)
                    : adapter_send_vel_until(s, velocity, deadline);
}
static const cartpole_device_ops_t operations = {
    .bind_cancellation = bind_cancellation,
    .get_latest_measurement = port_get_latest_measurement,
    .get_session_state = port_get_session_state,
    .get_configuration = port_get_configuration,
    .receive = port_receive,
    .get_error = port_get_error,
    .query_mode = port_query_mode,
    .wait_for_measurement = port_wait_measurement,
    .enable = port_enable,
    .disable = port_disable,
    .reset = port_reset,
    .refresh_configuration = port_refresh_configuration,
    .read_settings = port_read_settings,
    .read_pole_settings = port_read_pole_settings,
    .mark_endpoint = port_mark,
    .confirm_left = port_confirm_left,
    .read_calibration_status = port_read_calibration_status,
    .configure = port_configure,
    .stop_and_confirm = port_stop_and_confirm,
    .attach_observer = port_attach_observer,
    .detach_observer = port_detach_observer,
    .report_failure = port_report_failure,
    .send_velocity = port_send_velocity,
};
serial_cartpole_t *serial_cartpole_open(const char *port, int baud, const cartpole_clock_t *clock)
{
    if (!port || !clock || !clock->now || !clock->sleep_until)
        return NULL;
    serial_cartpole_t *s = calloc(1, sizeof *s);
    if (!s)
        return NULL;
    s->clock = clock;
    s->uart.fd = -1;
    s->snapshot.last_state_received_host_monotonic_s = -1.;
    proto_rx_init(&s->rx);
    if (uart_open(&s->uart, port, baud) != 0) {
        free(s);
        return NULL;
    }
    s->snapshot.is_serial_open = 1;
    return s;
}
void serial_cartpole_sync_startup(serial_cartpole_t *s)
{
    if (!s || s->startup_sync != STARTUP_IDLE)
        return;
    /* 首批字节可能从一行中间开始。等待第二条边界明确的有效 STATE，
     * 不要求传感器数据有效，也不占用后续命令的回执等待时间。 */
    const double end = adapter_now(s) + .025;
    s->startup_sync = STARTUP_DISCARD_FIRST_LINE;
    while (adapter_checkpoint(s, end, 0) == CARTPOLE_OK) {
        if (adapter_pump(s, 0) < 0 || s->startup_sync == STARTUP_COMPLETE)
            break;
        cartpole_clock_sleep_until(s->clock, fmin(end, adapter_now(s) + .001));
    }
    if (s->startup_sync != STARTUP_COMPLETE) {
        /* 启动窗口耗尽时不等待遥测，并清空已缓存的半行，避免与命令回执拼接。 */
        proto_rx_init(&s->rx);
        s->startup_sync = STARTUP_COMPLETE;
    }
}
cartpole_device_t serial_cartpole_get_device(serial_cartpole_t *s)
{
    return (cartpole_device_t){.ops = &operations, .context = s, .clock = s ? s->clock : NULL};
}
void serial_cartpole_close(serial_cartpole_t *s)
{
    if (!s)
        return;
    uart_close(&s->uart);
    free(s);
}
serial_cartpole_statistics_t serial_cartpole_get_statistics(const serial_cartpole_t *s)
{
    return (serial_cartpole_statistics_t){s->snapshot.received_state_count,
                                          s->snapshot.received_unknown_line_count};
}
