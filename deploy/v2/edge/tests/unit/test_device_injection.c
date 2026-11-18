/* 业务模块仅链接设备替身，不链接 uart/proto/serial_cartpole。 */
#include "cartpole.h"
#include "cartpole_device.h"
#include "cartpole_runtime.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
typedef struct {
    double now;
    cartpole_runtime_t runtime;
    cartpole_measurement_t measurement;
    cartpole_session_t session;
    cartpole_configuration_t configuration;
    cartpole_event_handler_t observer;
    void *observer_context;
    cartpole_result_t error;
    int reads, sends, test_case;
} fake_device;
static double now(void *ctx) { return ((fake_device *)ctx)->now; }
static void sleep_until(void *ctx, double deadline)
{
    fake_device *d = ctx;
    if (deadline > d->now)
        d->now = deadline;
}
static cartpole_measurement_t measurement(void *ctx) { return ((fake_device *)ctx)->measurement; }
static cartpole_session_t session(void *ctx) { return ((fake_device *)ctx)->session; }
static cartpole_configuration_t configuration(void *ctx)
{
    return ((fake_device *)ctx)->configuration;
}
static cartpole_result_t error(void *ctx) { return ((fake_device *)ctx)->error; }
static void report_failure(void *ctx, cartpole_result_t result)
{
    fake_device *d = ctx;
    d->error = result;
    d->session.has_stream_failure_latch = 1;
}
static cartpole_result_t observe(void *ctx, cartpole_event_handler_t handler, void *context)
{
    fake_device *d = ctx;
    if (handler && d->observer)
        return CARTPOLE_REJECTED;
    d->observer = handler;
    d->observer_context = context;
    return CARTPOLE_OK;
}
static void detach(void *ctx)
{
    fake_device *d = ctx;
    d->observer = NULL;
    d->observer_context = NULL;
}
static void emit(fake_device *d, cartpole_event_kind_t kind)
{
    cartpole_event_t e = {.kind = kind};
    if (kind == CARTPOLE_EVENT_MEASUREMENT)
        e.data.measurement = d->measurement;
    if (kind == CARTPOLE_EVENT_SESSION)
        e.data.session = d->session;
    if (kind == CARTPOLE_EVENT_CONFIGURATION)
        e.data.configuration = d->configuration;
    if (d->observer)
        d->observer(&e, d->observer_context);
}
static cartpole_receive_result_t receive(void *ctx)
{
    fake_device *d = ctx;
    d->reads++;
    if (d->reads == 2 && d->test_case == 1) {
        d->measurement.pole_valid = 0;
        emit(d, CARTPOLE_EVENT_MEASUREMENT);
        d->measurement.pole_valid = 1;
        emit(d, CARTPOLE_EVENT_MEASUREMENT);
    }
    if (d->reads == 2 && d->test_case == 2) {
        d->configuration.velocity_limit_mm_s = 800;
        emit(d, CARTPOLE_EVENT_CONFIGURATION);
        d->configuration.velocity_limit_mm_s = 900;
        emit(d, CARTPOLE_EVENT_CONFIGURATION);
    }
    if (d->reads == 2 && d->test_case == 3) {
        d->session.reported_mode = CARTPOLE_MODE_FAULT;
        d->session.has_received_fault_mode = 1;
        emit(d, CARTPOLE_EVENT_SESSION);
        d->session.reported_mode = CARTPOLE_MODE_READY;
        emit(d, CARTPOLE_EVENT_SESSION);
    }
    if (d->reads >= 3)
        d->runtime.stop_requested = 1;
    return CARTPOLE_RECEIVE_EMPTY;
}
static cartpole_result_t transaction(void *ctx, double timeout)
{
    (void)ctx;
    (void)timeout;
    return CARTPOLE_OK;
}
static cartpole_result_t mark(void *ctx, cartpole_endpoint_t endpoint, double timeout)
{
    (void)endpoint;
    return transaction(ctx, timeout);
}
static cartpole_result_t configure(void *ctx, cartpole_setting_t setting, long value,
                                   double timeout)
{
    (void)setting;
    (void)value;
    return transaction(ctx, timeout);
}
static cartpole_stop_result_t stop(void *ctx, double timeout)
{
    (void)ctx;
    (void)timeout;
    return CARTPOLE_STOP_DISABLED;
}
static cartpole_send_result_t send_velocity(void *ctx, double velocity, double deadline, int once)
{
    fake_device *d = ctx;
    assert(velocity == 450.);
    assert(deadline == 1.020);
    assert(once);
    d->sends++;
    d->runtime.stop_requested = 1;
    return CARTPOLE_SEND_COMPLETE;
}
static const cartpole_device_ops_t ops = {.get_latest_measurement = measurement,
                                          .get_session_state = session,
                                          .get_configuration = configuration,
                                          .receive = receive,
                                          .get_error = error,
                                          .attach_observer = observe,
                                          .detach_observer = detach,
                                          .report_failure = report_failure,
                                          .query_mode = transaction,
                                          .wait_measurement = transaction,
                                          .enable = transaction,
                                          .reset = transaction,
                                          .refresh_configuration = transaction,
                                          .mark = mark,
                                          .configure = configure,
                                          .stop_and_confirm = stop,
                                          .send_velocity = send_velocity};
static cartpole_device_t prepare(fake_device *d)
{
    *d = (fake_device){.now = 1.,
                       .measurement = {.cart_valid = 1,
                                       .pole_valid = 1,
                                       .measurement_id = 1,
                                       .received_host_monotonic_s = 1.},
                       .session = {.connection_open = 1,
                                   .is_mode_received = 1,
                                   .reported_mode = CARTPOLE_MODE_READY},
                       .configuration = {.complete = 1,
                                         .calibration_state = CARTPOLE_CALIBRATION_READY,
                                         .track_length_um = 900000,
                                         .calibration_saved = 1,
                                         .calibration_storage_ok = 1,
                                         .velocity_limit_mm_s = 900,
                                         .acceleration_limit_mm_s2 = 10000,
                                         .watchdog_timeout_ms = 100,
                                         .track_margin_um = 10000}};
    cartpole_clock_t clock = {now, sleep_until, d};
    assert(!cartpole_runtime_init(&d->runtime, &clock));
    return (cartpole_device_t){&ops, d, &d->runtime};
}
static int infer(const cartpole_measurement_t *m, float *action, void *context)
{
    (void)context;
    assert(m->measurement_id == 1);
    *action = .5f;
    return 0;
}
static void test_lifecycle_and_cached_queries(void)
{
    fake_device a, b;
    cartpole_device_t da = prepare(&a), db = prepare(&b);
    cartpole_device_ops_t incomplete = {.get_latest_measurement = measurement};
    cartpole_device_t invalid = {&incomplete, &a, &a.runtime};
    assert(!cartpole_create(invalid, &a.runtime));
    assert(!a.observer);
    assert(!cartpole_create(da, &b.runtime));
    assert(!a.observer);
    cartpole_t *ca = cartpole_create(da, &a.runtime), *cb = cartpole_create(db, &b.runtime);
    assert(ca && cb);
    assert(!cartpole_create(da, &a.runtime));
    assert(a.observer);
    assert(cartpole_get_latest_measurement(ca).measurement_id == 1);
    assert(cartpole_get_session_state(ca).reported_mode == CARTPOLE_MODE_READY);
    assert(cartpole_get_configuration(ca).complete);
    assert(a.reads == 0);
    a.now = 1.005;
    assert(cartpole_measurement_age_s(ca, measurement(&a)) > .0049);
    cartpole_request_stop(ca);
    assert(cartpole_stop_requested(ca));
    assert(!cartpole_stop_requested(cb));
    cartpole_destroy(ca);
    assert(!a.observer && b.observer);
    assert(!a.runtime.session_bound);
    volatile sig_atomic_t external_stop = 0;
    b.runtime.external_stop = &external_stop;
    assert(!cartpole_stop_requested(cb));
    external_stop = 1;
    assert(cartpole_stop_requested(cb));
    cartpole_destroy(cb);
    assert(!b.observer);
    emit(&a, CARTPOLE_EVENT_SESSION); /* 销毁已解绑，不再触及已释放context。 */
}
static void test_controller(int test_case, cartpole_result_t expected, int sends)
{
    fake_device d;
    cartpole_device_t device = prepare(&d);
    d.test_case = test_case;
    cartpole_t *c = cartpole_create(device, &d.runtime);
    assert(c);
    cartpole_controller_options_t options = {400, 900, 900, d.configuration};
    assert(cartpole_run_controller(c, &options, infer, NULL) == expected);
    assert(d.sends == sends);
    assert(!d.runtime.loop_active);
    cartpole_destroy(c);
}
int main(void)
{
    test_lifecycle_and_cached_queries();
    test_controller(0, CARTPOLE_CANCELLED, 1);
    test_controller(1, CARTPOLE_CANCELLED, 0); /* 批内无效→有效使推理结果失效。 */
    test_controller(2, CARTPOLE_CONFIGURATION_CHANGED, 0);
    test_controller(3, CARTPOLE_FAULT, 0);
    puts("device injection tests passed");
    return 0;
}
