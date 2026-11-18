/* 最小集成边界：公开 cartpole 业务接口 -> serial_cartpole -> proto。
 * UART 字节流和时钟为替身，不使用真实串口。适配器专属读写场景通过
 * 公开 cartpole_device_t 契约执行，控制器/事务场景通过业务 API 执行。 */
#include "cartpole.h"
#include "cartpole_control.h"
#include "serial_cartpole.h"
#include "../support/fake_uart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
_Static_assert(sizeof(((cartpole_measurement_t *)0)->measurement_id) == sizeof(uint64_t),
               "measurement id stays 64-bit");
_Static_assert(sizeof(((serial_cartpole_statistics_t *)0)->unknown_received_lines) ==
                   sizeof(uint64_t),
               "unknown count stays 64-bit");
static cartpole_t *device;
static serial_cartpole_t *serial;
static cartpole_clock_t test_clock;
static fake_uart_t *transport;
#define used (transport->pending)
#define reads (transport->reads)
#define broken (transport->read_error)
static int write_fail;
static int respond(fake_uart_t *, const uint8_t *, size_t);
static int write_command(fake_uart_t *, const uint8_t *, size_t);
static void before_read(fake_uart_t *);
static const char *query_reply, *enable_reply, *disable_reply, *config_reply, *mark_reply,
    *reset_reply;
static int zero_state = 1;
static int nonzero_writes;
static double largest_sleep, velocity_write_delay;
static int config_queries, calib_queries;
static int try_result, try_calls, framing_newlines, cancel_once, enable_writes;
static int cancel_transport, metadata_missing;
static double clock_s = 1., write_delay, read_delay;
static double fake_now(void *context)
{
    (void)context;
    double value = clock_s;
    clock_s += .0001;
    return value;
}
static void fake_sleep(void *context, double target)
{
    (void)context;
    double delay = target - clock_s;
    if (delay > largest_sleep)
        largest_sleep = delay;
    if (target > clock_s)
        clock_s = target;
}
static void close_test(void)
{
    cartpole_destroy(device);
    serial_cartpole_close(serial);
    device = NULL;
    serial = NULL;
}
static int open_test(void)
{
    close_test();
    fake_uart_reset();
    transport = fake_uart_port(0);
    transport->write = write_command;
    transport->before_read = before_read;
    write_fail = 0;
    query_reply = enable_reply = disable_reply = config_reply = mark_reply = reset_reply = NULL;
    test_clock = (cartpole_clock_t){fake_now, fake_sleep, NULL};
    serial = serial_cartpole_open("fake", 460800, &test_clock);
    if (!serial)
        return -1;
    device = cartpole_create(serial_cartpole_get_device(serial), &test_clock);
    return device ? 0 : -1;
}
static int write_command(fake_uart_t *u, const uint8_t *data, size_t n)
{
    (void)u;
    (void)data;
    (void)n;
    try_calls++;
    if (cancel_transport) {
        cartpole_request_stop(device);
        return -1;
    }
    if (cancel_once) {
        cancel_once = 0;
        cartpole_request_stop(device);
        return 1;
    }
    clock_s += write_delay;
    if (n >= 4 && !memcmp(data, "VEL ", 4))
        clock_s += velocity_write_delay;
    return try_result ? try_result : respond(u, data, n);
}
static void queue(const char *s) { fake_uart_queue(transport, s); }
static void before_read(fake_uart_t *p) { if (p->pending) clock_s += read_delay; }
static int respond(fake_uart_t *u, const uint8_t *d, size_t n)
{
    (void)u;
    if (write_fail)
        return -1;
    char cmd[128];
    assert(n < sizeof cmd);
    memcpy(cmd, d, n);
    cmd[n] = 0;
    if (!strncmp(cmd, "VEL ", 4) && strcmp(cmd, "VEL 0\n"))
        nonzero_writes++;
    if (!strcmp(cmd, "\n"))
        framing_newlines++;
    if (!strcmp(cmd, "VEL 0\n") && zero_state)
        queue("STATE 3 0 0 0 0\n");
    if (!strcmp(cmd, "QUERY\n") && query_reply)
        queue(query_reply);
    if (!strcmp(cmd, "MARK LEFT\n") && mark_reply)
        queue(mark_reply);
    if (!strcmp(cmd, "RESET\n") && reset_reply)
        queue(reset_reply);
    if (!strcmp(cmd, "ENABLE\n"))
        enable_writes++;
    if (!strcmp(cmd, "ENABLE\n") && enable_reply)
        queue(enable_reply);
    if (!strcmp(cmd, "DISABLE\n") && disable_reply)
        queue(disable_reply);
    if (!strncmp(cmd, "CONFIG ", 7) && config_reply)
        queue(config_reply);
    if (!strcmp(cmd, "CONFIG?\n")) {
        config_queries++;
        queue(metadata_missing ? "ERR VEL_MAX\n"
                               : "OK VEL_MAX 900\nOK ACC_MAX 10000\nOK WDT_MS "
                                 "100\nOK X_MARGIN_UM 10000\n");
    }
    if (!strcmp(cmd, "CALIB?\n")) {
        calib_queries++;
        queue("OK CAL_STATE 2\nOK CAL_LENGTH_UM 894845\nOK CAL_SAVED 1\nOK "
              "CAL_STORAGE 0\n");
    }
    return 0;
}
static cartpole_send_result_t send_velocity(double v)
{
    cartpole_device_t d = serial_cartpole_get_device(serial);
    return d.ops->send_velocity(d.context, v, cartpole_get_monotonic_time_s(device) + .020, 0);
}
static cartpole_send_result_t try_velocity(double v, double end)
{
    cartpole_device_t d = serial_cartpole_get_device(serial);
    return d.ops->send_velocity(d.context, v, end, 1);
}
static cartpole_receive_result_t receive(void)
{
    cartpole_device_t d = serial_cartpole_get_device(serial);
    return d.ops->receive(d.context);
}
static void open_ready(void)
{
    assert(open_test() == 0);
    query_reply = "MODE READY NONE\n";
    assert(cartpole_query_mode(device, .02) == 0);
    queue("STATE 3 1 2 3 4\n");
    assert(receive() == 0);
}
static unsigned measurement_flags(void)
{
    cartpole_measurement_t m = cartpole_get_latest_measurement(device);
    return (m.cart_valid ? 1u : 0u) | (m.pole_valid ? 2u : 0u);
}
static int observed_flags[8], hook_calls;
static cartpole_mode_t observed_modes[8];
static cartpole_event_kind_t observed_events[8];
static cartpole_measurement_t observed_snapshots[8];
static void update_hook(const cartpole_event_t *event, void *context)
{
    assert(context == &hook_calls);
    assert(hook_calls < 8);
    observed_events[hook_calls] = event->kind;
    observed_snapshots[hook_calls] = cartpole_get_latest_measurement(device);
    observed_flags[hook_calls] = (int)measurement_flags();
    observed_modes[hook_calls] = cartpole_get_session_state(device).reported_mode;
    hook_calls++;
}

static int business_events, bad_callback, infer_count, infer_behavior;
static void business_event(const cartpole_event_t *event, void *context)
{
    (void)context;
    business_events++;
    if (event->kind == CARTPOLE_EVENT_MEASUREMENT) {
        assert(event->data.measurement.measurement_id ==
               cartpole_get_latest_measurement(device).measurement_id);
    }
    if (bad_callback) {
        assert(cartpole_enable(device, .01) == CARTPOLE_CALLBACK_ERROR);
        assert(cartpole_create(serial_cartpole_get_device(serial), &test_clock) == NULL);
        cartpole_destroy(device);
        assert(cartpole_get_session_state(device).connection_open);
        assert(cartpole_subscribe(device, NULL, NULL) == CARTPOLE_CALLBACK_ERROR);
        assert(cartpole_stop_and_confirm(device, .01) == CARTPOLE_STOP_UNCONFIRMED);
    }
}
static int infer_test(const cartpole_measurement_t *measurement, float *action, void *context)
{
    (void)context;
    assert(measurement->cart_valid && measurement->pole_valid);
    infer_count++;
    if (infer_behavior == 3)
        return 0;
    *action = .1f;
    if (infer_behavior == 1 && infer_count == 1)
        queue("STATE 1 0 0 0 0\nSTATE 3 0 0 0 0\n");
    if (infer_behavior == 1 && infer_count == 2)
        cartpole_request_stop(device);
    if (infer_behavior == 2)
        queue("MODE FAULT MT6835\nMODE READY NONE\n");
    return 0;
}
static int observed_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    (void)context;
    action->kind = CARTPOLE_TICK_OBSERVE;
    return 0;
}
static int expired_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    (void)context;
    clock_s += .02;
    action->kind = CARTPOLE_TICK_VELOCITY;
    action->velocity_mm_s = 10;
    return 0;
}
static int drive_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    (void)context;
    action->kind = CARTPOLE_TICK_VELOCITY;
    action->velocity_mm_s = 10;
    write_delay = .008;
    return 0;
}
static int cancel_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    (void)context;
    cartpole_request_stop(device);
    action->kind = CARTPOLE_TICK_VELOCITY;
    action->velocity_mm_s = 10;
    return 0;
}
static void test_business_controller(void)
{
    open_ready();
    assert(cartpole_refresh_configuration(device, .02) == CARTPOLE_OK);
    cartpole_controller_options_t options = {.control_hz = 400,
                                             .action_scale_mm_s = 900,
                                             .velocity_limit_mm_s = 900,
                                             .expected_configuration =
                                                 cartpole_get_configuration(device)};
    infer_count = 0;
    infer_behavior = 0;
    int writes = nonzero_writes;
    assert(cartpole_run_controller(device, &options, infer_test, NULL) ==
           CARTPOLE_CONTROL_DEADLINE);
    assert(infer_count == 1 && nonzero_writes == writes + 1); /* no duplicate inference */
    open_ready();
    assert(cartpole_refresh_configuration(device, .02) == CARTPOLE_OK);
    assert(cartpole_subscribe(device, business_event, NULL) == CARTPOLE_OK);
    business_events = 0;
    infer_count = 0;
    infer_behavior = 1;
    writes = nonzero_writes;
    assert(cartpole_run_controller(device, &options, infer_test, NULL) == CARTPOLE_CANCELLED);
    assert(infer_count == 2 && nonzero_writes == writes && business_events >= 2);
    open_ready();
    assert(cartpole_refresh_configuration(device, .02) == CARTPOLE_OK);
    infer_count = 0;
    infer_behavior = 2;
    writes = nonzero_writes;
    assert(cartpole_run_controller(device, &options, infer_test, NULL) == CARTPOLE_FAULT);
    assert(nonzero_writes == writes);
    open_ready();
    assert(cartpole_subscribe(device, business_event, NULL) == CARTPOLE_OK);
    bad_callback = 1;
    query_reply = "MODE READY NONE\n";
    assert(cartpole_query_mode(device, .02) == CARTPOLE_CALLBACK_ERROR);
    assert(cartpole_get_session_state(device).connection_open);
    bad_callback = 0;
    open_ready();
    writes = nonzero_writes;
    assert(cartpole_run_loop(device, .001, .01, expired_tick, NULL) == CARTPOLE_OK);
    assert(nonzero_writes == writes);
    open_ready();
    assert(cartpole_run_loop(device, .001, .004, drive_tick, NULL) == CARTPOLE_TIMEOUT);
    write_delay = 0;
    open_ready();
    largest_sleep = 0;
    assert(cartpole_run_loop(device, 1., .01, observed_tick, NULL) == CARTPOLE_OK);
    assert(largest_sleep < .02);
    open_ready();
    writes = nonzero_writes;
    assert(cartpole_run_loop(device, .001, 0, cancel_tick, NULL) == CARTPOLE_CANCELLED);
    assert(nonzero_writes == writes);
    open_ready();
    assert(cartpole_refresh_configuration(device, .02) == CARTPOLE_OK);
    infer_behavior = 3;
    writes = nonzero_writes;
    assert(cartpole_run_controller(device, &options, infer_test, NULL) == CARTPOLE_CALLBACK_ERROR);
    assert(nonzero_writes == writes);
    open_ready();
    queue("STATE 3 0 0 0 0\n");
    assert(receive() == 0);
    zero_state = 0;
    assert(cartpole_move_to_center(device, .02) ==
           CARTPOLE_TIMEOUT); /* old zero sample is not stop proof */
    zero_state = 1;
    open_ready();
    queue("STATE 3 10000 0 0 0\n");
    assert(receive() == 0);
    velocity_write_delay = .008;
    assert(cartpole_move_to_center(device, .006) == CARTPOLE_TIMEOUT);
    velocity_write_delay = 0;
    close_test();
    assert(device == NULL && serial == NULL);
}

int main(void)
{
    open_ready();
    query_reply = "MODE FAULT MT6835\n";
    assert(cartpole_query_mode(device, .02) == CARTPOLE_OK);
    assert(cartpole_get_session_state(device).reported_mode == CARTPOLE_MODE_FAULT);
    cartpole_request_stop(device);
    close_test();
    assert(device == NULL && serial == NULL);
    assert(open_test() == 0);
    assert(!cartpole_is_stop_requested(device));
    assert(cartpole_get_session_state(device).connection_open);
    assert(cartpole_get_latest_measurement(device).measurement_id == 0);
    assert(serial_cartpole_get_statistics(serial).unknown_received_lines == 0);
    assert(!cartpole_get_session_state(device).has_received_fault_mode);
    assert(!cartpole_get_session_state(device).has_stream_failure_latch);
    int known;
    (known = cartpole_get_session_state(device).is_mode_received,
     cartpole_get_session_state(device).reported_mode);
    assert(!known);
    assert(cartpole_calculate_measurement_age_s(device, cartpole_get_latest_measurement(device)) < 0);
    assert(measurement_flags() == 0);
    /* 事务预算不能在drain/send后重置；过期也不得接受已排队回执。 */
    open_ready();
    enable_reply = "MODE READY NONE\n";
    write_delay = .01;
    double started = cartpole_get_monotonic_time_s(device);
    assert(cartpole_enable(device, .003) == CARTPOLE_TIMEOUT);
    assert(cartpole_get_monotonic_time_s(device) - started < .014);
    write_delay = 0;
    /* 每次QUERY必须等新响应。 */
    open_ready();
    query_reply = NULL;
    assert(cartpole_query_mode(device, .003) == CARTPOLE_TIMEOUT);
    /* ENABLE按新MODE而非OK确认，不能复用已有READY。 */
    open_ready();
    assert(cartpole_enable(device, .003) == CARTPOLE_TIMEOUT);
    enable_reply = "MODE READY NONE\n";
    assert(cartpole_enable(device, .02) == 0);
    enable_reply = "ERR ENABLE\nMODE READY NONE\n";
    assert(cartpole_enable(device, .02) == CARTPOLE_REJECTED);
    /* 无效小车不能变成零值有效测量。 */
    queue("STATE 2 0 20 0 2\n");
    assert(receive() == 0);
    assert(measurement_flags() == 2);
    assert(send_velocity(10) == -1);
    assert(send_velocity(0) == 0);
    /* 不匹配回执不能满足配置，回显不同不能报成功。 */
    open_ready();
    config_reply = "OK ACC_MAX 5000\n";
    assert(cartpole_configure(device, CARTPOLE_SETTING_VELOCITY_LIMIT, 900, .003) ==
           CARTPOLE_TIMEOUT);
    config_reply = "OK VEL_MAX 800\n";
    assert(cartpole_configure(device, CARTPOLE_SETTING_VELOCITY_LIMIT, 900, .02) ==
           CARTPOLE_REPLY_MISMATCH);
    config_reply = "ERR VEL_MAX\n";
    assert(cartpole_configure(device, CARTPOLE_SETTING_VELOCITY_LIMIT, 900, .02) ==
           CARTPOLE_REJECTED);
    config_reply = "ERR VEL_MAX\nOK VEL_MAX 900\n";
    assert(cartpole_configure(device, CARTPOLE_SETTING_VELOCITY_LIMIT, 900, .02) ==
           CARTPOLE_REJECTED);
    config_reply = "OK ACC_MAX 5000\nOK VEL_MAX 900\n";
    assert(cartpole_configure(device, CARTPOLE_SETTING_VELOCITY_LIMIT, 900, .02) == 0);
    /* 一次刷新采集全部元数据，读缓存不再发查询。 */
    assert(cartpole_refresh_configuration(device, .02) == 0);
    int cq = config_queries, ca = calib_queries;
    cartpole_configuration_t config = cartpole_get_configuration(device);
    assert(config.complete && config.track_length_um == 894845 &&
           config.velocity_limit_mm_s == 900);
    cartpole_measurement_t before_read = cartpole_get_latest_measurement(device);
    cartpole_session_t session_before = cartpole_get_session_state(device);
    (void)cartpole_get_configuration(device);
    assert(cartpole_get_latest_measurement(device).measurement_id == before_read.measurement_id);
    assert(cartpole_get_session_state(device).has_stream_failure_latch ==
           session_before.has_stream_failure_latch);
    assert(cq == config_queries && ca == calib_queries);
    /* 有界接收返回积压；后续调用继续消费且不丢失测量。 */
    open_ready();
    for (int i = 0; i < 600; i++)
        queue("STATE 3 0 0 0 0\n");
    reads = 0;
    assert(receive() == CARTPOLE_RECEIVE_BACKLOG && reads > 0 && used > 0);
    for (unsigned attempts = 0; used && attempts < 600; attempts++) {
        size_t pending = used;
        cartpole_receive_result_t result = receive();
        assert(result != CARTPOLE_RECEIVE_ERROR && used < pending);
    }
    assert(used == 0);
    assert(receive() == CARTPOLE_RECEIVE_EMPTY);
    assert(cartpole_get_latest_measurement(device).measurement_id == 601);
    open_ready();
    broken = 1;
    assert(receive() == -1 &&
           cartpole_get_session_state(device).has_stream_failure_latch);
    /* 信号之后仍确认退出，FAULT不冒充DISABLED。 */
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE DISABLED NONE\n";
    cartpole_request_stop(device);
    assert(cartpole_stop_and_confirm(device, .02) == 0);
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE FAULT MT6835\n";
    assert(cartpole_stop_and_confirm(device, .02) == 1);
    open_ready();
    query_reply = NULL;
    assert(cartpole_stop_and_confirm(device, .003) == -1);
    open_ready();
    queue("STATE 3 invalid\n");
    assert(receive() == -1);
    open_ready();
    query_reply = NULL;
    queue("MODE DISABLED NONE\n");
    zero_state = 0;
    assert(cartpole_stop_and_confirm(device, .003) == -1);
    zero_state = 1;
    /* 一次pump中FAULT后READY仍保留故障历史。 */
    open_ready();
    queue("MODE FAULT MT6835\nMODE READY NONE\n");
    assert(receive() == 0);
    assert(cartpole_get_session_state(device).has_received_fault_mode);
    open_ready();
    double received_at = cartpole_get_latest_measurement(device).received_host_monotonic_s;
    assert(received_at > 0 && received_at <= cartpole_get_monotonic_time_s(device));
    cartpole_measurement_t before = cartpole_get_latest_measurement(device);
    assert(cartpole_subscribe(device, update_hook, &hook_calls) == CARTPOLE_OK);
    queue("STATE 1 0 0 0 0\nSTATE 3 0 0 0 0\nMODE DISABLED NONE\nMODE READY "
          "NONE\nOK ACC_MAX "
          "10000\n");
    assert(receive() == 0 && hook_calls == 5);
    assert(observed_flags[0] == 1 && observed_flags[1] == 3);
    assert(before.measurement_id + 2 == cartpole_get_latest_measurement(device).measurement_id);
    assert(before.received_host_monotonic_s == received_at);
    assert(observed_events[0] == CARTPOLE_EVENT_MEASUREMENT &&
           observed_events[2] == CARTPOLE_EVENT_SESSION &&
           observed_events[4] == CARTPOLE_EVENT_CONFIGURATION);
    for (int i = 2; i < 5; i++) {
        assert(observed_snapshots[i].measurement_id == observed_snapshots[1].measurement_id);
        assert(observed_snapshots[i].received_host_monotonic_s ==
               observed_snapshots[1].received_host_monotonic_s);
    }
    assert(observed_modes[2] == CARTPOLE_MODE_DISABLED && observed_modes[3] == CARTPOLE_MODE_READY);
    assert(cartpole_get_latest_measurement(device).received_host_monotonic_s >= received_at);
    open_ready();
    assert(hook_calls == 5); /* open clears hook */
    open_ready();
    try_result = 0;
    int old_calls = try_calls;
    assert(try_velocity(10, cartpole_get_monotonic_time_s(device) - 1.) == 1 && try_calls == old_calls);
    assert(try_velocity(10, cartpole_get_monotonic_time_s(device) + 1.) == 0);
    write_delay = .02;
    assert(try_velocity(10, cartpole_get_monotonic_time_s(device) + .01) == 2);
    write_delay = 0.;
    try_result = 1;
    assert(try_velocity(10, cartpole_get_monotonic_time_s(device) + 1.) == 1 &&
           !cartpole_get_session_state(device).has_stream_failure_latch);
    try_result = -2;
    assert(try_velocity(10, cartpole_get_monotonic_time_s(device) + 1.) == -1 &&
           cartpole_get_session_state(device).has_stream_failure_latch);
    try_result = 0;
    int old_newlines = framing_newlines;
    assert(send_velocity(0) == 0 && framing_newlines == old_newlines + 1);
    open_ready();
    query_reply = "MODE DISABLED NONE\n";
    read_delay = .01;
    assert(cartpole_stop_and_confirm(device, .003) == CARTPOLE_STOP_UNCONFIRMED);
    read_delay = 0.;
    open_ready();
    assert(cartpole_refresh_configuration(device, .02) == 0);
    close_test();
    assert(open_test() == 0);
    assert(!cartpole_get_configuration(device).complete);
    open_ready();
    queue("OK MARK_LEFT -42\n");
    assert(cartpole_mark_endpoint(device, CARTPOLE_LEFT, .003) ==
           CARTPOLE_TIMEOUT); /* stale response discarded */
    mark_reply = "OK MARK_RIGHT 84\n";
    assert(cartpole_mark_endpoint(device, CARTPOLE_LEFT, .003) ==
           CARTPOLE_TIMEOUT); /* request correlation */
    mark_reply = "OK MARK_LEFT -42\n";
    assert(cartpole_mark_endpoint(device, CARTPOLE_LEFT, .02) == 0);
    assert(cartpole_mark_endpoint(device, (cartpole_endpoint_t)99, .02) == CARTPOLE_INVALID_ARGUMENT);
    reset_reply = "MODE DISABLED NONE\n";
    assert(cartpole_reset(device, .02) == 0);
    assert(cartpole_get_session_state(device).reported_mode == CARTPOLE_MODE_DISABLED);
    reset_reply = "ERR RESET\nMODE DISABLED NONE\n";
    assert(cartpole_reset(device, .02) == CARTPOLE_REJECTED);
    open_ready();
    int before_enable = enable_writes;
    cancel_once = 1;
    assert(cartpole_enable(device, .02) == CARTPOLE_CANCELLED);
    assert(enable_writes == before_enable);
    query_reply = "MODE DISABLED NONE\n";
    assert(cartpole_stop_and_confirm(device, .02) == CARTPOLE_STOP_DISABLED);
    open_ready();
    assert(cartpole_configure(device, (cartpole_setting_t)-1, 900, .02) ==
           CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_configure(device, (cartpole_setting_t)99, 900, .02) ==
           CARTPOLE_INVALID_ARGUMENT);
    assert(!cartpole_get_session_state(device).has_stream_failure_latch);
    assert(cartpole_query_mode(device, 0) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_wait_for_measurement(device, -1) == CARTPOLE_INVALID_ARGUMENT);
    broken = 1;
    assert(cartpole_query_mode(device, .02) == CARTPOLE_TRANSPORT_ERROR);
    open_ready();
    queue("STATE invalid\n");
    assert(cartpole_wait_for_measurement(device, .02) == CARTPOLE_PROTOCOL_ERROR);
    open_ready();
    enable_reply = "MODE FAULT MT6835\nMODE READY NONE\n";
    assert(cartpole_enable(device, .02) == CARTPOLE_FAULT);
    open_ready();
    assert(cartpole_enable(device, NAN) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_reset(device, INFINITY) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_refresh_configuration(device, -.1) == CARTPOLE_INVALID_ARGUMENT);
    metadata_missing = 1;
    assert(cartpole_refresh_configuration(device, .003) == CARTPOLE_TIMEOUT);
    metadata_missing = 0;
    cancel_transport = 1;
    assert(cartpole_enable(device, .02) == CARTPOLE_TRANSPORT_ERROR);
    cancel_transport = 0;
    open_ready();
    query_reply = "MODE FAULT MT6835\n";
    assert(cartpole_query_mode(device, .02) == CARTPOLE_OK);
    assert(cartpole_refresh_configuration(device, .02) == CARTPOLE_OK);
    reset_reply = "MODE DISABLED NONE\n";
    assert(cartpole_reset(device, .02) == CARTPOLE_OK); /* historicalFAULT isn't newFAULT */
    open_ready();
    assert(send_velocity(NAN) == CARTPOLE_SEND_ERROR &&
           !cartpole_get_session_state(device).has_stream_failure_latch);
    try_result = 1;
    assert(send_velocity(10) == CARTPOLE_SEND_ERROR);
    assert(cartpole_get_session_state(device).has_stream_failure_latch);
    try_result = 0;
    int before_retry = try_calls;
    assert(send_velocity(10) == CARTPOLE_SEND_ERROR && try_calls == before_retry);
    assert(send_velocity(0) == CARTPOLE_SEND_COMPLETE);
    assert(cartpole_query_mode(device, .02) ==
           CARTPOLE_TIMEOUT); /* original timeout, not invented IO error */
    test_business_controller();
    close_test();
    puts("edge session passed");
    return 0;
}
