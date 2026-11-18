#include "cartpole_internal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

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

static void on_device_event(const cartpole_event_t *event, void *context);
cartpole_t *cartpole_create(cartpole_device_t device, cartpole_runtime_t *runtime)
{
    const cartpole_device_ops_t *o = device.ops;
    if (!o || !runtime || runtime->session_bound || device.runtime != runtime ||
        !runtime->clock.now || !runtime->clock.sleep_until || !o->get_latest_measurement ||
        !o->get_session_state || !o->get_configuration || !o->receive || !o->get_error ||
        !o->report_failure || !o->attach_observer || !o->detach_observer || !o->query_mode ||
        !o->wait_measurement || !o->enable || !o->reset || !o->refresh_configuration || !o->mark ||
        !o->configure || !o->stop_and_confirm || !o->send_velocity)
        return NULL;
    cartpole_t *c = calloc(1, sizeof *c);
    if (!c)
        return NULL;
    c->device = device;
    c->runtime = runtime;
    if (o->attach_observer(device.context, on_device_event, c) != CARTPOLE_OK) {
        free(c);
        return NULL;
    }
    runtime->session_bound = 1;
    return c;
}
void cartpole_destroy(cartpole_t *c)
{
    if (!c)
        return;
    if (c->runtime->callback_depth || c->runtime->loop_active) {
        c->runtime->callback_violation = 1;
        return;
    }
    c->device.ops->detach_observer(c->device.context);
    c->runtime->session_bound = 0;
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
double cartpole_host_monotonic_s(cartpole_t *c) { return cartpole_runtime_now(c->runtime); }
double cartpole_measurement_age_s(cartpole_t *c, cartpole_measurement_t m)
{
    return m.received_host_monotonic_s < 0
               ? -1.
               : cartpole_host_monotonic_s(c) - m.received_host_monotonic_s;
}
void cartpole_request_stop(cartpole_t *c) { c->runtime->stop_requested = 1; }
int cartpole_stop_requested(cartpole_t *c) { return cartpole_runtime_is_cancelled(c->runtime); }
void cartpole_sleep_to(cartpole_t *c, double t) { cartpole_runtime_sleep_until(c->runtime, t); }
cartpole_result_t cartpole_stream_result(cartpole_t *c)
{
    return c->device.ops->get_error(c->device.context);
}
static int mutation_allowed(cartpole_t *c)
{
    if (c->runtime->callback_depth) {
        c->runtime->callback_violation = 1;
        return 0;
    }
    return 1;
}
int cartpole_callback_enter(cartpole_t *c)
{
    if (!mutation_allowed(c))
        return -1;
    c->runtime->callback_depth = 1;
    c->runtime->callback_violation = 0;
    return 0;
}
int cartpole_callback_leave(cartpole_t *c)
{
    c->runtime->callback_depth = 0;
    if (c->runtime->callback_violation) {
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
    if (c->controller_observer)
        c->controller_observer(event, c->controller_context);
    if (c->subscriber) {
        if (cartpole_callback_enter(c))
            return;
        c->subscriber(event, c->subscriber_context);
        (void)cartpole_callback_leave(c);
    }
}
void cartpole_set_control_observer(cartpole_t *c, cartpole_event_handler_t handler, void *context)
{
    c->controller_observer = handler;
    c->controller_context = context;
}
cartpole_receive_result_t cartpole_receive(cartpole_t *c)
{
    return c->device.ops->receive(c->device.context);
}
cartpole_send_result_t cartpole_send_velocity(cartpole_t *c, double v, double end, int once)
{
    if (!mutation_allowed(c))
        return CARTPOLE_SEND_ERROR;
    return c->device.ops->send_velocity(c->device.context, v, end, once);
}
int cartpole_loop_begin(cartpole_t *c)
{
    if (!mutation_allowed(c) || c->runtime->loop_active)
        return -1;
    c->runtime->loop_active = 1;
    return 0;
}
void cartpole_loop_end(cartpole_t *c) { c->runtime->loop_active = 0; }
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
cartpole_result_t cartpole_wait_measurement(cartpole_t *c, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result
                                 : c->device.ops->wait_measurement(c->device.context, timeout);
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
cartpole_result_t cartpole_mark(cartpole_t *c, cartpole_endpoint_t endpoint, double timeout)
{
    cartpole_result_t result = transaction_check(c, timeout);
    return result != CARTPOLE_OK ? result
                                 : c->device.ops->mark(c->device.context, endpoint, timeout);
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
