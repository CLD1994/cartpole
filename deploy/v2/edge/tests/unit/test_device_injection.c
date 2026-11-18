/* 业务模块仅链接设备替身，不链接 uart/proto/serial_cartpole。 */
#include "cartpole.h"
#include "cartpole_control.h"
#include <signal.h>
#include "cartpole_device.h"
#include "cartpole_clock.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include "fake_cartpole_device.h"
static int infer(const cartpole_measurement_t *m, float *action, void *context)
{
    (void)context;
    assert(m->measurement_id == 1);
    *action = .5f;
    return 0;
}
static int cancelled(void *ctx) { return *(volatile sig_atomic_t *)ctx != 0; }
static void test_lifecycle_and_cached_queries(void)
{
    fake_device a, b;
    cartpole_device_t da = fake_device_prepare(&a), db = fake_device_prepare(&b);
    cartpole_device_ops_t incomplete = {0};
    cartpole_device_t invalid = {&incomplete, &a, &a.clock};
    assert(!cartpole_create(invalid, &a.clock));
    assert(!a.observer);
    assert(!cartpole_create(da, &b.clock));
    assert(!a.observer);
    cartpole_t *ca = cartpole_create(da, &a.clock), *cb = cartpole_create(db, &b.clock);
    assert(ca && cb);
    assert(!cartpole_create(da, &a.clock));
    assert(a.observer);
    assert(cartpole_get_latest_measurement(ca).measurement_id == 1);
    assert(cartpole_get_session_state(ca).reported_mode == CARTPOLE_MODE_READY);
    assert(cartpole_get_configuration(ca).complete);
    assert(a.reads == 0);
    a.now = 1.005;
    assert(cartpole_calculate_measurement_age_s(ca, cartpole_get_latest_measurement(ca)) > .0049);
    cartpole_request_stop(ca);
    assert(cartpole_is_stop_requested(ca));
    assert(!cartpole_is_stop_requested(cb));
    cartpole_destroy(ca);
    assert(!a.observer && b.observer);
    cartpole_t *again = cartpole_create(da, &a.clock);
    assert(again);
    cartpole_destroy(again);
    volatile sig_atomic_t external_stop = 0;
    assert(cartpole_set_cancellation_source(cb, cancelled, (void *)&external_stop) == CARTPOLE_OK);
    assert(!cartpole_is_stop_requested(cb));
    external_stop = 1;
    assert(cartpole_is_stop_requested(cb));
    cartpole_destroy(cb);
    assert(!b.observer);
    fake_device_emit(&a, CARTPOLE_EVENT_SESSION); /* 销毁已解绑，不再触及已释放context。 */
}
typedef struct { int scenario; cartpole_t *session; } controller_scenario;
static cartpole_receive_result_t controller_receive(fake_device *d)
{
    controller_scenario *context = d->scenario_context;
    int scenario = context->scenario;
    if (d->reads == 2 && scenario == 1) {
        d->measurement.pole_valid = 0;
        fake_device_emit(d, CARTPOLE_EVENT_MEASUREMENT);
        d->measurement.pole_valid = 1;
        fake_device_emit(d, CARTPOLE_EVENT_MEASUREMENT);
    }
    if (d->reads == 2 && scenario == 2) {
        d->configuration.velocity_limit_mm_s = 800;
        fake_device_emit(d, CARTPOLE_EVENT_CONFIGURATION);
        d->configuration.velocity_limit_mm_s = 900;
        fake_device_emit(d, CARTPOLE_EVENT_CONFIGURATION);
    }
    if (d->reads == 2 && scenario == 3) {
        d->session.reported_mode = CARTPOLE_MODE_FAULT;
        d->session.has_received_fault_mode = 1;
        fake_device_emit(d, CARTPOLE_EVENT_SESSION);
        d->session.reported_mode = CARTPOLE_MODE_READY;
        fake_device_emit(d, CARTPOLE_EVENT_SESSION);
    }
    if (d->reads >= 3)
        cartpole_request_stop(((controller_scenario *)d->scenario_context)->session);
    return CARTPOLE_RECEIVE_EMPTY;
}
static cartpole_send_result_t controller_send(fake_device *d, double velocity, double deadline, int once)
{
    assert(velocity == 450.);
    assert(deadline == 1.020);
    assert(once);
    cartpole_request_stop(((controller_scenario *)d->scenario_context)->session);
    return CARTPOLE_SEND_COMPLETE;
}
static void test_shared_clock_independent_sessions(void)
{
    fake_device a, b;
    cartpole_device_t da = fake_device_prepare(&a), db = fake_device_prepare(&b);
    db.clock = &a.clock;
    cartpole_t *ca = cartpole_create(da, &a.clock);
    cartpole_t *cb = cartpole_create(db, &a.clock);
    assert(ca && cb);
    a.now = 2.5;
    assert(cartpole_get_monotonic_time_s(ca) == 2.5);
    assert(cartpole_get_monotonic_time_s(cb) == 2.5);
    assert(a.cancellation_probe && b.cancellation_probe);
    cartpole_request_stop(ca);
    assert(a.cancellation_probe(a.cancellation_context));
    assert(!b.cancellation_probe(b.cancellation_context));
    cartpole_destroy(ca);
    assert(!a.cancellation_probe && b.cancellation_probe);
    assert(!cartpole_is_stop_requested(cb));
    cartpole_destroy(cb);
    assert(!b.cancellation_probe);
}
static void test_controller(int test_case, cartpole_result_t expected, int sends)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    controller_scenario context = {.scenario = test_case};
    d.scenario_context = &context;
    d.on_receive = controller_receive;
    d.on_send = controller_send;
    cartpole_t *c = cartpole_create(device, &d.clock);
    assert(c);
    context.session = c;
    cartpole_controller_options_t options = {400, 900, 900, d.configuration};
    assert(cartpole_run_controller(c, &options, infer, NULL) == expected);
    assert(d.sends == sends);
    cartpole_destroy(c);
}
static cartpole_result_t nested_hold(cartpole_t *c, void *context)
{
    (void)context;
    return cartpole_hold_velocity(c, 100., 1.);
}
static void test_nested_hold_has_no_io_side_effects(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    assert(cartpole_execute_operation(c, nested_hold, NULL, NULL) == CARTPOLE_CALLBACK_ERROR);
    assert(d.call_count == 0 && d.reads == 0 && d.sends == 0);
    cartpole_destroy(c);
}
int main(void)
{
    test_lifecycle_and_cached_queries();
    test_shared_clock_independent_sessions();
    test_nested_hold_has_no_io_side_effects();
    test_controller(0, CARTPOLE_CANCELLED, 1);
    test_controller(1, CARTPOLE_CANCELLED, 0); /* 批内无效→有效使推理结果失效。 */
    test_controller(2, CARTPOLE_CONFIGURATION_CHANGED, 0);
    test_controller(3, CARTPOLE_FAULT, 0);
    puts("device injection tests passed");
    return 0;
}
