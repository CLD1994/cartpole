#include "fake_cartpole_device.h"
#include <assert.h>
#include <stddef.h>

static void bind_cancellation(void *ctx, cartpole_cancellation_probe_t probe, void *context)
{
    fake_device *d = ctx;
    d->cancellation_probe = probe;
    d->cancellation_context = context;
}
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
    d->attaches++;
    if (d->attach_result != CARTPOLE_OK)
        return d->attach_result;
    if (handler && d->observer)
        return CARTPOLE_REJECTED;
    d->observer = handler;
    d->observer_context = context;
    return CARTPOLE_OK;
}
static void detach(void *ctx)
{
    fake_device *d = ctx;
    d->detaches++;
    d->observer = NULL;
    d->observer_context = NULL;
}
void fake_device_emit(fake_device *d, cartpole_event_kind_t kind)
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
    return d->on_receive ? d->on_receive(d) : CARTPOLE_RECEIVE_EMPTY;
}
static void record(fake_device *d, int operation, double timeout, int argument, long value)
{
    assert(d->call_count < 64);
    int i = d->call_count++;
    d->calls[i].operation = operation;
    d->calls[i].timeout = timeout;
    d->calls[i].argument = argument;
    d->calls[i].value = value;
}
#define TRANSACTION(name, op) \
static cartpole_result_t name(void *ctx, double timeout) \
{ fake_device *d = ctx; record(d, op, timeout, 0, 0); return d->transaction_result; }
TRANSACTION(query, FAKE_QUERY)
TRANSACTION(wait_measurement, FAKE_WAIT)
TRANSACTION(enable, FAKE_ENABLE)
TRANSACTION(disable, FAKE_DISABLE)
TRANSACTION(reset, FAKE_RESET)
TRANSACTION(refresh, FAKE_REFRESH)
static cartpole_result_t read_settings(void *ctx, cartpole_settings_t *out, double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_READ_SETTINGS, timeout, 0, 0);
    if (d->transaction_result == CARTPOLE_OK)
        *out = d->settings;
    return d->transaction_result;
}
static cartpole_result_t mark(void *ctx, cartpole_endpoint_t endpoint, double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_MARK, timeout, endpoint, 0);
    return d->transaction_result;
}
static cartpole_result_t confirm_left(void *ctx, int64_t *position_um, double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_CONFIRM_LEFT, timeout, 0, 0);
    if (d->transaction_result == CARTPOLE_OK)
        *position_um = d->confirmed_left_position_um;
    return d->transaction_result;
}
static cartpole_result_t read_calibration_status(void *ctx, cartpole_calibration_status_t *out,
                                                 double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_READ_CALIBRATION_STATUS, timeout, 0, 0);
    if (d->transaction_result == CARTPOLE_OK)
        *out = d->calibration_status;
    return d->transaction_result;
}
static cartpole_result_t configure(void *ctx, cartpole_setting_t setting, long value, double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_CONFIGURE, timeout, setting, value);
    return d->transaction_result;
}
static cartpole_stop_result_t stop(void *ctx, double timeout)
{
    fake_device *d = ctx;
    record(d, FAKE_STOP, timeout, 0, 0);
    return d->stop_result;
}
static cartpole_send_result_t send_velocity(void *ctx, double velocity, double deadline, int once)
{
    fake_device *d = ctx;
    d->sends++;
    return d->on_send ? d->on_send(d, velocity, deadline, once) : CARTPOLE_SEND_COMPLETE;
}
static const cartpole_device_ops_t ops = {.bind_cancellation = bind_cancellation,.get_latest_measurement = measurement,
                                          .get_session_state = session,
                                          .get_configuration = configuration,
                                          .receive = receive,
                                          .get_error = error,
                                          .attach_observer = observe,
                                          .detach_observer = detach,
                                          .report_failure = report_failure,
                                          .query_mode = query,
                                          .wait_for_measurement = wait_measurement,
                                          .enable = enable,
                                          .disable = disable,
                                          .reset = reset,
                                          .refresh_configuration = refresh,
                                          .read_settings = read_settings,
                                          .mark_endpoint = mark,
                                          .confirm_left = confirm_left,
                                          .read_calibration_status = read_calibration_status,
                                          .configure = configure,
                                          .stop_and_confirm = stop,
                                          .send_velocity = send_velocity};
cartpole_device_t fake_device_prepare(fake_device *d)
{
    *d = (fake_device){.now = 1., .stop_result = CARTPOLE_STOP_DISABLED,
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
    d->clock = (cartpole_clock_t){now, sleep_until, d};
    return (cartpole_device_t){&ops, d, &d->clock};
}
