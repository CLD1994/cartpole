/* Business-session delegation, with no transport/parser implementation linked. */
#include "fake_cartpole_device.h"
#include <assert.h>
#include <math.h>
#include <stddef.h>

typedef cartpole_result_t (*transaction_fn)(cartpole_t *, double);
static const transaction_fn transactions[] = {
    cartpole_query_mode, cartpole_wait_for_measurement, cartpole_enable,
    cartpole_reset, cartpole_refresh_configuration
};
static void test_delegation_and_invalid_timeouts(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    assert(c);
    const cartpole_result_t outcomes[] = {CARTPOLE_OK, CARTPOLE_TIMEOUT, CARTPOLE_FAULT,
                                         CARTPOLE_TRANSPORT_ERROR, CARTPOLE_REJECTED};
    for (unsigned i = 0; i < sizeof(transactions)/sizeof(*transactions); i++) {
        for (unsigned j = 0; j < sizeof(outcomes)/sizeof(*outcomes); j++) {
            d.call_count = 0;
            d.transaction_result = outcomes[j];
            assert(transactions[i](c, .375) == outcomes[j]);
            assert(d.call_count == 1 && d.calls[0].operation == (int)i);
            assert(d.calls[0].timeout == .375);
            assert(d.reads == 0 && d.sends == 0); /* no hidden fallback motion/IO */
        }
        const double bad[] = {0, -1, NAN, INFINITY};
        for (unsigned j = 0; j < sizeof(bad)/sizeof(*bad); j++) {
            d.call_count = 0;
            assert(transactions[i](c, bad[j]) == CARTPOLE_INVALID_ARGUMENT);
            assert(d.call_count == 0);
        }
    }
    d.call_count = 0;
    d.transaction_result = CARTPOLE_REPLY_MISMATCH;
    assert(cartpole_mark_endpoint(c, CARTPOLE_RIGHT, .25) == CARTPOLE_REPLY_MISMATCH);
    assert(d.call_count == 1 && d.calls[0].operation == FAKE_MARK);
    assert(d.calls[0].argument == CARTPOLE_RIGHT && d.calls[0].timeout == .25);
    assert(cartpole_configure(c, CARTPOLE_SETTING_ACCELERATION_LIMIT, 10000, .5) == CARTPOLE_REPLY_MISMATCH);
    assert(d.call_count == 2 && d.calls[1].operation == FAKE_CONFIGURE);
    assert(d.calls[1].argument == CARTPOLE_SETTING_ACCELERATION_LIMIT && d.calls[1].value == 10000);
    assert(d.calls[1].timeout == .5);
    assert(cartpole_mark_endpoint(c, CARTPOLE_LEFT, NAN) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_configure(c, CARTPOLE_SETTING_ACCELERATION_LIMIT, 5000, -1) == CARTPOLE_INVALID_ARGUMENT);
    assert(d.call_count == 2);
    d.stop_result = CARTPOLE_STOP_FAULT;
    assert(cartpole_stop_and_confirm(c, .75) == CARTPOLE_STOP_FAULT);
    assert(d.call_count == 3 && d.calls[2].operation == FAKE_STOP && d.calls[2].timeout == .75);
    assert(cartpole_stop_and_confirm(c, 0) == CARTPOLE_STOP_UNCONFIRMED);
    assert(d.call_count == 3);
    cartpole_destroy(c);
    assert(d.detaches == 1 && d.sends == 0 && d.call_count == 3);
}

typedef struct { cartpole_t *session; int calls; cartpole_result_t result; } subscriber;
static void forbidden_mutation(const cartpole_event_t *event, void *context)
{
    subscriber *s = context;
    assert(event->kind == CARTPOLE_EVENT_MEASUREMENT);
    assert(cartpole_get_latest_measurement(s->session).measurement_id == event->data.measurement.measurement_id);
    s->calls++;
    s->result = cartpole_enable(s->session, .5);
}
static void stop_from_event(const cartpole_event_t *event, void *context)
{
    subscriber *s = context;
    assert(event->kind == CARTPOLE_EVENT_SESSION);
    s->calls++;
    cartpole_request_stop(s->session);
}
static void test_event_subscription_and_callback_contract(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    subscriber first = {c, 0, CARTPOLE_OK}, second = {c, 0, CARTPOLE_OK};
    assert(cartpole_subscribe(c, forbidden_mutation, &first) == CARTPOLE_OK);
    fake_device_emit(&d, CARTPOLE_EVENT_MEASUREMENT);
    assert(first.calls == 1 && first.result == CARTPOLE_CALLBACK_ERROR);
    assert(d.call_count == 0 && d.sends == 0);
    assert(cartpole_get_session_state(c).has_stream_failure_latch);
    assert(d.error == CARTPOLE_CALLBACK_ERROR);
    assert(cartpole_subscribe(c, stop_from_event, &second) == CARTPOLE_OK);
    fake_device_emit(&d, CARTPOLE_EVENT_SESSION);
    assert(first.calls == 1 && second.calls == 1 && cartpole_is_stop_requested(c));
    assert(cartpole_subscribe(c, NULL, NULL) == CARTPOLE_OK);
    fake_device_emit(&d, CARTPOLE_EVENT_SESSION);
    assert(second.calls == 1);
    cartpole_destroy(c);
    fake_device_emit(&d, CARTPOLE_EVENT_SESSION);
    assert(second.calls == 1 && d.detaches == 1);
}
static void test_failed_attach_allows_same_clock_retry(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    d.attach_result = CARTPOLE_REJECTED;
    assert(!cartpole_create(device, &d.clock));
    assert(d.attaches == 1 && d.detaches == 0 && !d.observer);
    d.attach_result = CARTPOLE_OK;
    cartpole_t *c = cartpole_create(device, &d.clock);
    assert(c && d.attaches == 2);
    assert(!cartpole_create(device, &d.clock) && d.attaches == 3);
    cartpole_destroy(c);
    assert(d.detaches == 1 && !d.observer);
}
typedef struct { fake_device *device; cartpole_t *session; int order[4], count; } operation_context;
static void operation_observer(const cartpole_event_t *event, void *opaque)
{
    operation_context *ctx = opaque;
    assert(event->kind == CARTPOLE_EVENT_MEASUREMENT);
    ctx->order[ctx->count++] = 1;
}
static void display_observer(const cartpole_event_t *event, void *opaque)
{
    operation_context *ctx = opaque;
    assert(event->kind == CARTPOLE_EVENT_MEASUREMENT);
    ctx->order[ctx->count++] = 2;
}
static cartpole_result_t emit_operation(cartpole_t *session, void *opaque)
{
    operation_context *ctx = opaque;
    assert(cartpole_execute_operation(session, emit_operation, NULL, ctx) == CARTPOLE_CALLBACK_ERROR);
    fake_device_emit(ctx->device, CARTPOLE_EVENT_MEASUREMENT);
    return CARTPOLE_TIMEOUT;
}
static int forbidden_evaluation(void *opaque)
{
    operation_context *ctx = opaque;
    assert(cartpole_enable(ctx->session, .5) == CARTPOLE_CALLBACK_ERROR);
    return 0; /* 忽略内层错误也不能把只读评估报告为成功。 */
}
static void test_operation_scope_and_readonly_evaluation(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    operation_context context = { .device = &d, .session = c };
    assert(cartpole_subscribe(c, display_observer, &context) == CARTPOLE_OK);
    assert(cartpole_execute_operation(c, emit_operation, operation_observer, &context) == CARTPOLE_TIMEOUT);
    assert(context.count == 2 && context.order[0] == 1 && context.order[1] == 2);
    fake_device_emit(&d, CARTPOLE_EVENT_MEASUREMENT);
    assert(context.count == 3 && context.order[2] == 2); /* 作用域结束，业务观察器已解除 */
    assert(cartpole_execute_operation(c, NULL, NULL, NULL) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_evaluate_measurement(c, forbidden_evaluation, &context) == CARTPOLE_CALLBACK_ERROR);
    assert(d.call_count == 0 && cartpole_get_session_state(c).has_stream_failure_latch);
    cartpole_destroy(c);
}

static cartpole_send_result_t skipped_send(fake_device *d, double velocity, double deadline, int once)
{
    (void)d;
    assert(velocity == -123.5 && deadline == 1.25 && once == 1);
    return CARTPOLE_SEND_SKIPPED;
}
static cartpole_receive_result_t backlog_receive(fake_device *d)
{
    (void)d;
    return CARTPOLE_RECEIVE_BACKLOG;
}
static void test_basic_io_delegation_and_invalid_input(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    d.on_send = skipped_send;
    d.on_receive = backlog_receive;
    assert(cartpole_send_velocity(c, -123.5, 1.25, 1) == CARTPOLE_SEND_SKIPPED);
    assert(d.sends == 1);
    assert(cartpole_receive(c) == CARTPOLE_RECEIVE_BACKLOG && d.reads == 1);
    assert(cartpole_send_velocity(c, NAN, 1.25, 1) == CARTPOLE_SEND_ERROR);
    assert(cartpole_send_velocity(c, 1., INFINITY, 1) == CARTPOLE_SEND_ERROR);
    assert(cartpole_send_velocity(c, 1., 1.25, 2) == CARTPOLE_SEND_ERROR);
    assert(d.sends == 1);
    assert(cartpole_sleep_until(c, NAN) == CARTPOLE_INVALID_ARGUMENT);
    assert(d.now == 1.);
    assert(cartpole_sleep_until(c, 1.25) == CARTPOLE_OK && d.now == 1.25);
    cartpole_destroy(c);
}
typedef struct { cartpole_t *session; int wakes, cancel; } sleep_scenario;
static void interrupted_sleep(void *opaque, double deadline)
{
    fake_device *d = opaque;
    sleep_scenario *scenario = d->scenario_context;
    scenario->wakes++;
    if (scenario->wakes == 1) {
        d->now += .01;
        if (scenario->cancel) cartpole_request_stop(scenario->session);
    } else {
        d->now = deadline;
    }
}
static void test_early_wakeup_respects_deadline_or_cancellation(void)
{
    for (int cancel = 0; cancel <= 1; cancel++) {
        fake_device d;
        cartpole_device_t device = fake_device_prepare(&d);
        d.clock.sleep_until = interrupted_sleep;
        cartpole_t *c = cartpole_create(device, &d.clock);
        sleep_scenario scenario = {.session = c, .cancel = cancel};
        d.scenario_context = &scenario;
        assert(cartpole_sleep_until(c, 2.) == (cancel ? CARTPOLE_CANCELLED : CARTPOLE_OK));
        assert(scenario.wakes == (cancel ? 1 : 2));
        assert(d.now == (cancel ? 1.01 : 2.));
        cartpole_destroy(c);
    }
}
static cartpole_result_t destroy_during_operation(cartpole_t *c, void *context)
{
    (void)context;
    cartpole_destroy(c);
    return CARTPOLE_OK;
}
static void test_operation_cannot_hide_illegal_destroy(void)
{
    fake_device d;
    cartpole_device_t device = fake_device_prepare(&d);
    cartpole_t *c = cartpole_create(device, &d.clock);
    assert(cartpole_execute_operation(c, destroy_during_operation, NULL, NULL) == CARTPOLE_CALLBACK_ERROR);
    assert(d.detaches == 0 && cartpole_get_session_state(c).connection_open);
    assert(cartpole_get_session_state(c).has_stream_failure_latch);
    cartpole_destroy(c);
    assert(d.detaches == 1 && !d.cancellation_probe);
}
int main(void)
{
    test_delegation_and_invalid_timeouts();
    test_event_subscription_and_callback_contract();
    test_failed_attach_allows_same_clock_retry();
    test_operation_scope_and_readonly_evaluation();
    test_basic_io_delegation_and_invalid_input();
    test_early_wakeup_respects_deadline_or_cancellation();
    test_operation_cannot_hide_illegal_destroy();
    return 0;
}
