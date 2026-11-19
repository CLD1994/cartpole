/* 最小集成：真实 serial_cartpole + proto；UART 边界使用共享字节流替身。
 * 验证解析结果、通知顺序、故障锁存与两个连接的状态隔离。 */
#include "serial_cartpole.h"
#include "../support/fake_uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static fake_uart_t *transport;
static double now;
static double get_time(void *ctx)
{
    (void)ctx;
    return now;
}
static void sleep_time(void *ctx, double target)
{
    (void)ctx;
    now = target;
}
static unsigned config_events, measurement_events, session_events;
static cartpole_device_t *fail_device;
static int in_event;
static void event(const cartpole_event_t *e, void *ctx)
{
    (void)ctx;
    assert(!in_event);
    in_event = 1;
    if (e->kind == CARTPOLE_EVENT_CONFIGURATION)
        config_events++;
    if (e->kind == CARTPOLE_EVENT_SESSION)
        session_events++;
    if (e->kind == CARTPOLE_EVENT_MEASUREMENT) {
        measurement_events++;
        if (fail_device)
            fail_device->ops->report_failure(fail_device->context, CARTPOLE_CALLBACK_ERROR);
    }
    in_event = 0;
}

static const char *request, *response;
static unsigned request_count;
static int firmware_reply(fake_uart_t *port, const uint8_t *bytes, size_t size)
{
    assert(size == strlen(request) && memcmp(bytes, request, size) == 0);
    request_count++;
    fake_uart_queue(port, response);
    return 0;
}

static void expect_request(fake_uart_t *port, const char *wire_request, const char *wire_response)
{
    request = wire_request;
    response = wire_response;
    request_count = 0;
    port->write = firmware_reply;
}

static double first_state_at, reply_at;
static void startup_state(fake_uart_t *port)
{
    if (now >= first_state_at) {
        fake_uart_queue(port, "STATE 0 0 0 0 0\n");
        port->before_read = NULL;
    }
}

static void test_startup_sync_ends_at_first_valid_boundary(const cartpole_clock_t *clock)
{
    fake_uart_reset();
    serial_cartpole_t *s = serial_cartpole_open("startup", 460800, clock);
    assert(s);
    fake_uart_t *p = fake_uart_port(0);
    cartpole_device_t d = serial_cartpole_get_device(s);
    double started = now;
    first_state_at = started + .004;
    fake_uart_queue(p, "STATE 3 1 2 3 4\n");
    p->before_read = startup_state;
    expect_request(p, "QUERY\n", "MODE DISABLED NONE\n");

    serial_cartpole_sync_startup(s);
    assert(now >= first_state_at && now - started < .010);
    assert(request_count == 0);
    cartpole_measurement_t sample = d.ops->get_latest_measurement(d.context);
    assert(sample.measurement_id == 1 && !sample.cart_valid && !sample.pole_valid);
    double synchronized_at = now;
    serial_cartpole_sync_startup(s);
    assert(now == synchronized_at);
    assert(d.ops->query_mode(d.context, .010) == CARTPOLE_OK && request_count == 1);
    serial_cartpole_close(s);
}

static void delayed_mode(fake_uart_t *port)
{
    if (now >= reply_at) {
        fake_uart_queue(port, "MODE DISABLED NONE\n");
        port->before_read = NULL;
    }
}

static int query_with_delayed_mode(fake_uart_t *port, const uint8_t *bytes, size_t size)
{
    assert(size == 6 && memcmp(bytes, "QUERY\n", size) == 0);
    request_count++;
    reply_at = now + .008;
    port->before_read = delayed_mode;
    return 0;
}

static void test_startup_fallback_does_not_spend_command_deadline(const cartpole_clock_t *clock)
{
    fake_uart_reset();
    serial_cartpole_t *s = serial_cartpole_open("no-telemetry", 460800, clock);
    assert(s);
    fake_uart_t *p = fake_uart_port(0);
    cartpole_device_t d = serial_cartpole_get_device(s);
    fake_uart_queue(p, "incomplete garbage without newline");
    request_count = 0;
    p->write = query_with_delayed_mode;
    double started = now;
    serial_cartpole_sync_startup(s);
    assert(now - started > .024 && now - started < .026);
    assert(request_count == 0);
    assert(d.ops->query_mode(d.context, .010) == CARTPOLE_OK && request_count == 1);
    serial_cartpole_close(s);
}

static int congested_config_write(fake_uart_t *port, const uint8_t *bytes, size_t size)
{
    const char command[] = "CONFIG POLE_ZERO_URAD 17\n";
    assert(size == sizeof(command) - 1 && memcmp(bytes, command, size) == 0);
    ++request_count;
    fake_uart_queue(port, "STATE 1 123 0 0 0\n");
    return 1; /* 输出拥塞，此次没有写入任何字节。 */
}

static cartpole_result_t configure_gate(cartpole_configure_stage_t stage, void *context)
{
    cartpole_device_t *d = context;
    assert(stage == CARTPOLE_CONFIGURE_CHECK);
    return d->ops->get_latest_measurement(d->context).pole_valid ? CARTPOLE_OK
                                                               : CARTPOLE_MEASUREMENT_UNAVAILABLE;
}

static void test_configure_rechecks_telemetry_after_backpressure(const cartpole_clock_t *clock)
{
    fake_uart_reset();
    serial_cartpole_t *s = serial_cartpole_open("guarded-configure", 460800, clock);
    assert(s);
    fake_uart_t *p = fake_uart_port(0);
    cartpole_device_t d = serial_cartpole_get_device(s);
    request_count = 0;
    p->write = congested_config_write;
    fake_uart_queue(p, "STATE 3 123 0 0 0\n");
    assert(d.ops->configure(d.context, CARTPOLE_SETTING_POLE_ZERO, 17, .1,
                            configure_gate, &d) == CARTPOLE_MEASUREMENT_UNAVAILABLE);
    assert(request_count == 1); /* 第一次未写入，第二次收到失效帧后拒绝写入。 */
    serial_cartpole_close(s);
}

static void test_maintenance_wire_contracts(const cartpole_clock_t *clock)
{
    fake_uart_reset();
    serial_cartpole_t *s = serial_cartpole_open("maintenance", 460800, clock);
    assert(s);
    fake_uart_t *p = fake_uart_port(0);
    cartpole_device_t d = serial_cartpole_get_device(s);

    /* Stale success arrives before write; only response to the new request counts. */
    fake_uart_queue(p, "OK CONFIRM_LEFT -300000\n");
    expect_request(p, "CONFIRM LEFT\n", "OK CONFIRM_LEFT -447422\n");
    int64_t position = 0;
    assert(d.ops->confirm_left(d.context, &position, .1) == CARTPOLE_OK);
    assert(position == -447422 && request_count == 1);
    expect_request(p, "CONFIRM LEFT\n", "ERR CONFIRM_LEFT\n");
    assert(d.ops->confirm_left(d.context, &position, .1) == CARTPOLE_REJECTED);
    assert(position == -447422 && request_count == 1);

    fake_uart_queue(p, "MODE DISABLED NONE\n");
    expect_request(p, "DISABLE\n", "ERR BUSY\n");
    assert(d.ops->disable(d.context, .1) == CARTPOLE_BUSY && request_count == 1);
    expect_request(p, "DISABLE\n", "MODE DISABLED NONE\n");
    assert(d.ops->disable(d.context, .1) == CARTPOLE_OK && request_count == 1);
    expect_request(p, "DISABLE\n", "MODE FAULT POWER\n");
    assert(d.ops->disable(d.context, .1) == CARTPOLE_FAULT && request_count == 1);
    assert(d.ops->get_session_state(d.context).reported_mode_reason == CARTPOLE_REASON_POWER);
    fake_uart_queue(p, "MODE DISABLED NONE\nSTATE 3 0 0 0 0\n");
    assert(d.ops->receive(d.context) == CARTPOLE_RECEIVE_EMPTY);
    expect_request(p, "ENABLE\n", "ERR POWER\n");
    assert(d.ops->enable(d.context, .1) == CARTPOLE_POWER_UNAVAILABLE && request_count == 1);
    /* A previous DISABLED response cannot stand in for the current request. */
    fake_uart_queue(p, "MODE DISABLED NONE\n");
    expect_request(p, "DISABLE\n", "");
    assert(d.ops->disable(d.context, .1) == CARTPOLE_TIMEOUT && request_count == 1);

    expect_request(p, "CALIB?\n", "OK CAL_STATE 3\nOK CAL_LENGTH_UM 894845\nOK CAL_SAVED 1\nOK CAL_STORAGE 7\n");
    cartpole_calibration_status_t status = {0};
    assert(d.ops->read_calibration_status(d.context, &status, .1) == CARTPOLE_OK);
    assert(status.state == 3 && status.length_um == 894845 &&
           status.saved == 1 && status.storage == 7 && request_count == 1);
    expect_request(p, "CALIB?\n", "OK CAL_STATE 2\nOK CAL_LENGTH_UM 800000\nOK CAL_SAVED 0\n");
    assert(d.ops->read_calibration_status(d.context, &status, .1) == CARTPOLE_TIMEOUT);
    assert(status.state == 3 && status.length_um == 894845 &&
           status.saved == 1 && status.storage == 7 && request_count == 1);

    /* 摆杆标定只需要这两个设置；新查询不能沿用上一批参数。 */
    cartpole_pole_settings_t pole_settings = {0};
    expect_request(p, "CONFIG?\n", "OK POLE_ZERO_URAD -32100\nOK POLE_INV 1\n");
    assert(d.ops->read_pole_settings(d.context, &pole_settings, .1) == CARTPOLE_OK);
    assert(pole_settings.zero_urad == -32100 && pole_settings.inverted == 1 &&
           request_count == 1);
    expect_request(p, "CONFIG?\n", "OK POLE_ZERO_URAD 13500\n");
    assert(d.ops->read_pole_settings(d.context, &pole_settings, .1) == CARTPOLE_TIMEOUT);
    assert(pole_settings.zero_urad == -32100 && pole_settings.inverted == 1 &&
           request_count == 1);
    serial_cartpole_close(s);
}

int main(void)
{
    fake_uart_reset();
    transport = fake_uart_port(0);
    cartpole_clock_t clock = {get_time, sleep_time, NULL};
    serial_cartpole_t *a = serial_cartpole_open("one", 460800, &clock);
    serial_cartpole_t *b = serial_cartpole_open("two", 460800, &clock);
    assert(a && b);
    cartpole_device_t d = serial_cartpole_get_device(a), other = serial_cartpole_get_device(b);
    assert(d.ops->attach_observer(d.context, event, NULL) == CARTPOLE_OK);
    assert(d.ops->attach_observer(d.context, event, NULL) == CARTPOLE_REJECTED);
    fake_uart_queue(transport, "STATE 3 1000 2000 3 4\n");
    assert(d.ops->receive(d.context) == 0);
    assert(d.ops->get_latest_measurement(d.context).measurement_id == 1);
    assert(other.ops->get_latest_measurement(other.context).measurement_id == 0);
    assert(measurement_events == 1);
    /* 第二条物理连接收到的数据不能被第一条连接消费。 */
    fake_uart_queue(fake_uart_port(1), "STATE 3 7000 8000 9 10\n");
    assert(d.ops->receive(d.context) == CARTPOLE_RECEIVE_EMPTY);
    assert(other.ops->get_latest_measurement(other.context).measurement_id == 0);
    assert(other.ops->receive(other.context) == CARTPOLE_RECEIVE_EMPTY);
    assert(other.ops->get_latest_measurement(other.context).measurement_id == 1);
    assert(d.ops->get_latest_measurement(d.context).measurement_id == 1);
    assert(measurement_events == 1);
    fake_uart_queue(transport, "OK ENABLE 1\n");
    d.ops->receive(d.context);
    assert(config_events == 0);
    fake_uart_queue(transport, "OK VEL_MAX 900\n");
    d.ops->receive(d.context);
    assert(config_events == 1);
    fake_uart_queue(transport, "OK VEL_MAX 900\n");
    d.ops->receive(d.context);
    assert(config_events == 1);
    fake_uart_queue(transport, "MODE FAULT MT6835\nMODE READY NONE\n");
    d.ops->receive(d.context);
    assert(d.ops->get_session_state(d.context).reported_mode == CARTPOLE_MODE_READY);
    assert(d.ops->get_session_state(d.context).has_received_fault_mode);
    assert(!other.ops->get_session_state(other.context).has_received_fault_mode);
    unsigned before = session_events;
    fail_device = &d;
    fake_uart_queue(transport, "STATE 3 2 3 4 5\n");
    assert(d.ops->receive(d.context) == CARTPOLE_RECEIVE_ERROR);
    assert(session_events == before);
    assert(d.ops->get_error(d.context) == CARTPOLE_CALLBACK_ERROR);
    assert(d.ops->receive(d.context) == CARTPOLE_RECEIVE_ERROR);
    assert(session_events == before + 1);
    assert(d.ops->get_session_state(d.context).has_stream_failure_latch);
    serial_cartpole_statistics_t stats = serial_cartpole_get_statistics(a);
    assert(stats.received_measurements == 2 && stats.unknown_received_lines == 0);
    serial_cartpole_close(a);
    serial_cartpole_close(b);
    test_startup_sync_ends_at_first_valid_boundary(&clock);
    test_startup_fallback_does_not_spend_command_deadline(&clock);
    test_maintenance_wire_contracts(&clock);
    test_configure_rechecks_telemetry_after_backpressure(&clock);
    puts("serial_cartpole: ok");
}
