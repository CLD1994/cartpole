#include "cartpole_internal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int mutation_allowed(cartpole_t *);
static int callback_enter(cartpole_t *);
static int callback_leave(cartpole_t *);
static int cancellation_probe(void *context) { return cartpole_is_stop_requested(context); }
static void on_device_event(const cartpole_event_t *event, void *context);
cartpole_t *cartpole_create(cartpole_device_t device)
{
    const cartpole_device_ops_t *o = device.ops;
    const cartpole_clock_t *clock = device.clock;
    if (!o || !clock ||
        !clock->now || !clock->sleep_until || !o->bind_cancellation || !o->get_latest_measurement ||
        !o->get_session_state || !o->get_configuration || !o->receive || !o->get_error ||
        !o->report_failure || !o->attach_observer || !o->detach_observer || !o->query_mode ||
        !o->wait_for_measurement || !o->enable || !o->reset || !o->refresh_configuration || !o->mark_endpoint ||
        !o->configure || !o->stop_and_confirm || !o->send_velocity)
        return NULL;
    cartpole_t *c = calloc(1, sizeof *c);
    if (!c)
        return NULL;
    c->device = device;
    if (o->attach_observer(device.context, on_device_event, c) != CARTPOLE_OK) {
        free(c);
        return NULL;
    }
    o->bind_cancellation(device.context, cancellation_probe, c);
    return c;
}
void cartpole_destroy(cartpole_t *c)
{
    if (!c)
        return;
    if (c->callback_depth || c->operation_active) {
        if (c->callback_depth) c->callback_violation = 1;
        if (c->operation_active) c->operation_violation = 1;
        return;
    }
    c->device.ops->detach_observer(c->device.context);
    c->device.ops->bind_cancellation(c->device.context, NULL, NULL);
    free(c);
}
cartpole_measurement_t cartpole_get_latest_measurement(cartpole_t *c)
{
    return c->device.ops->get_latest_measurement(c->device.context);
}
cartpole_session_t cartpole_get_session_state(cartpole_t *c)
{
    return c->device.ops->get_session_state(c->device.context);
}
cartpole_configuration_t cartpole_get_configuration(cartpole_t *c)
{
    return c->device.ops->get_configuration(c->device.context);
}
double cartpole_get_monotonic_time_s(cartpole_t *c) { return cartpole_clock_get_monotonic_time_s(c->device.clock); }
double cartpole_calculate_measurement_age_s(cartpole_t *c, cartpole_measurement_t m)
{
    return m.received_host_monotonic_s < 0
               ? -1.
               : cartpole_get_monotonic_time_s(c) - m.received_host_monotonic_s;
}
void cartpole_request_stop(cartpole_t *c) { c->stop_requested = 1; }
int cartpole_is_stop_requested(cartpole_t *c) { return c->stop_requested || (c->cancellation && c->cancellation(c->cancellation_context)); }
cartpole_result_t cartpole_sleep_until(cartpole_t *c, double t)
{
    if (!mutation_allowed(c)) return CARTPOLE_CALLBACK_ERROR;
    if (!isfinite(t) || t < 0) return CARTPOLE_INVALID_ARGUMENT;
    while (!cartpole_is_stop_requested(c) && cartpole_get_monotonic_time_s(c) < t)
        cartpole_clock_sleep_until(c->device.clock, t);
    return cartpole_is_stop_requested(c) ? CARTPOLE_CANCELLED : CARTPOLE_OK;
}
cartpole_result_t cartpole_get_stream_result(cartpole_t *c)
{
    return c->device.ops->get_error(c->device.context);
}
static int mutation_allowed(cartpole_t *c)
{
    if (c->callback_depth) {
        c->callback_violation = 1;
        return 0;
    }
    return 1;
}
static int callback_enter(cartpole_t *c)
{
    if (!mutation_allowed(c))
        return -1;
    c->callback_depth = 1;
    c->callback_violation = 0;
    return 0;
}
static int callback_leave(cartpole_t *c)
{
    c->callback_depth = 0;
    if (c->callback_violation) {
        c->device.ops->report_failure(c->device.context, CARTPOLE_CALLBACK_ERROR);
        return -1;
    }
    return 0;
}
cartpole_result_t cartpole_subscribe(cartpole_t *c, cartpole_event_handler_t handler, void *context)
{
    if (!mutation_allowed(c))
        return CARTPOLE_CALLBACK_ERROR;
    c->subscriber = handler;
    c->subscriber_context = context;
    return CARTPOLE_OK;
}
static void on_device_event(const cartpole_event_t *event, void *context)
{
    cartpole_t *c = context;
    /* 控制观察器逐事件先于展示订阅处理，保持批内短暂故障和门控转换。 */
    if (c->operation_observer && callback_enter(c) == 0) {
        c->operation_observer(event, c->operation_context);
        (void)callback_leave(c);
    }
    if (c->subscriber) {
        if (callback_enter(c))
            return;
        c->subscriber(event, c->subscriber_context);
        (void)callback_leave(c);
    }
}
cartpole_receive_result_t cartpole_receive(cartpole_t *c)
{
    if (!mutation_allowed(c)) return CARTPOLE_RECEIVE_ERROR;
    return c->device.ops->receive(c->device.context);
}
cartpole_send_result_t cartpole_send_velocity(cartpole_t *c, double v, double end, int once)
{
    if (!mutation_allowed(c))
        return CARTPOLE_SEND_ERROR;
    if (!isfinite(v) || fabs(v) > 1e9 || !isfinite(end) || (once != 0 && once != 1))
        return CARTPOLE_SEND_ERROR;
    return c->device.ops->send_velocity(c->device.context, v, end, once);
}
cartpole_result_t cartpole_set_cancellation_source(cartpole_t *c,
    cartpole_cancellation_probe_t probe, void *context)
{
    if (!mutation_allowed(c) || c->operation_active) return CARTPOLE_CALLBACK_ERROR;
    c->cancellation = probe;
    c->cancellation_context = context;
    return CARTPOLE_OK;
}
cartpole_result_t cartpole_evaluate_measurement(cartpole_t *c,
    cartpole_measurement_evaluator_t evaluate, void *context)
{
    if (!evaluate) return CARTPOLE_INVALID_ARGUMENT;
    if (callback_enter(c)) return CARTPOLE_CALLBACK_ERROR;
    int result = evaluate(context);
    int violation = callback_leave(c);
    return result || violation ? CARTPOLE_CALLBACK_ERROR : CARTPOLE_OK;
}
cartpole_result_t cartpole_execute_operation(cartpole_t *c, cartpole_operation_t operation,
    cartpole_event_handler_t observer, void *context)
{
    if (!operation) return CARTPOLE_INVALID_ARGUMENT;
    if (!mutation_allowed(c) || c->operation_active) return CARTPOLE_CALLBACK_ERROR;
    c->operation_active = 1;
    c->operation_violation = 0;
    c->operation_observer = observer;
    c->operation_context = context;
    cartpole_result_t result = operation(c, context);
    if (c->operation_violation) {
        c->device.ops->report_failure(c->device.context, CARTPOLE_CALLBACK_ERROR);
        result = CARTPOLE_CALLBACK_ERROR;
    }
    c->operation_observer = NULL;
    c->operation_context = NULL;
    c->operation_active = 0;
    return result;
}
static cartpole_result_t transaction_check(cartpole_t *c, double timeout)
{
    if (!mutation_allowed(c))
        return CARTPOLE_CALLBACK_ERROR;
    if (!isfinite(timeout) || timeout <= 0)
        return CARTPOLE_INVALID_ARGUMENT;
    return CARTPOLE_OK;
}
cartpole_result_t cartpole_query_mode(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result : c->device.ops->query_mode(c->device.context, timeout);
}
cartpole_result_t cartpole_wait_for_measurement(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result
                                 : c->device.ops->wait_for_measurement(c->device.context, timeout);
}
cartpole_result_t cartpole_enable(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result : c->device.ops->enable(c->device.context, timeout);
}
cartpole_result_t cartpole_reset(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result : c->device.ops->reset(c->device.context, timeout);
}
cartpole_result_t cartpole_refresh_configuration(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result
                                 : c->device.ops->refresh_configuration(c->device.context, timeout);
}
cartpole_result_t cartpole_mark_endpoint(cartpole_t *c, cartpole_endpoint_t endpoint, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result
                                 : c->device.ops->mark_endpoint(c->device.context, endpoint, timeout);
}
cartpole_result_t cartpole_configure(cartpole_t *c, cartpole_setting_t setting, long value,
                                     double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK
               ? result
               : c->device.ops->configure(c->device.context, setting, value, timeout);
}
cartpole_stop_result_t cartpole_stop_and_confirm(cartpole_t *c, double timeout)
{
    if (transaction_check(c, timeout) != CARTPOLE_OK)
        return CARTPOLE_STOP_UNCONFIRMED;
    return c->device.ops->stop_and_confirm(c->device.context, timeout);
}
