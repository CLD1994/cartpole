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
    puts("serial_cartpole: ok");
}
