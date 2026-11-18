#include "serial_cartpole.h"
#include "uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char *line;
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
int uart_open(uart_t *uart, const char *path, int baud)
{
    (void)path;
    (void)baud;
    uart->fd = 1;
    return 0;
}
void uart_close(uart_t *uart) { uart->fd = -1; }
int uart_read(uart_t *uart, uint8_t *bytes, size_t cap)
{
    (void)uart;
    if (!line)
        return 0;
    size_t n = strlen(line);
    assert(n <= cap);
    memcpy(bytes, line, n);
    line = NULL;
    return (int)n;
}
int uart_try_write_all(uart_t *uart, const uint8_t *bytes, size_t len)
{
    (void)uart;
    (void)bytes;
    (void)len;
    return 0;
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
int main(void)
{
    cartpole_runtime_t runtime = {0};
    cartpole_clock_t clock = {get_time, sleep_time, NULL};
    assert(cartpole_runtime_init(&runtime, &clock) == 0);
    serial_cartpole_t *a = serial_cartpole_open("one", 460800, &runtime);
    serial_cartpole_t *b = serial_cartpole_open("two", 460800, &runtime);
    assert(a && b);
    cartpole_device_t d = serial_cartpole_get_device(a), other = serial_cartpole_get_device(b);
    assert(d.ops->attach_observer(d.context, event, NULL) == CARTPOLE_OK);
    assert(d.ops->attach_observer(d.context, event, NULL) == CARTPOLE_REJECTED);
    line = "STATE 3 1000 2000 3 4\n";
    assert(d.ops->receive(d.context) == 0);
    assert(d.ops->get_latest_measurement(d.context).measurement_id == 1);
    assert(other.ops->get_latest_measurement(other.context).measurement_id == 0);
    assert(measurement_events == 1);
    line = "OK ENABLE 1\n";
    d.ops->receive(d.context);
    assert(config_events == 0);
    line = "OK VEL_MAX 900\n";
    d.ops->receive(d.context);
    assert(config_events == 1);
    line = "OK VEL_MAX 900\n";
    d.ops->receive(d.context);
    assert(config_events == 1);
    line = "MODE FAULT MT6835\nMODE READY NONE\n";
    d.ops->receive(d.context);
    assert(d.ops->get_session_state(d.context).reported_mode == CARTPOLE_MODE_READY);
    assert(d.ops->get_session_state(d.context).has_received_fault_mode);
    assert(!other.ops->get_session_state(other.context).has_received_fault_mode);
    unsigned before = session_events;
    fail_device = &d;
    line = "STATE 3 2 3 4 5\n";
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
    puts("serial_cartpole: ok");
}
