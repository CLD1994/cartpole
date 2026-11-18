#include "cartpole_control_internal.h"
#include "control.h"
#include <math.h>
#include <stddef.h>
typedef struct {
    double period, duration, elapsed;
    cartpole_tick_handler_t tick;
    void *context;
    cartpole_tick_action_t *action;
} schedule_t;
static int evaluate_tick(void *context)
{
    schedule_t *s = context;
    return s->tick(s->elapsed, s->action, s->context);
}
static cartpole_result_t run_schedule(cartpole_t *c, void *context)
{
    schedule_t *s = context;
    double period_s = s->period, duration_s = s->duration;
    if (!cartpole_get_session_state(c).connection_open) {
        return CARTPOLE_SESSION_UNAVAILABLE;
    }
    double start = cartpole_get_monotonic_time_s(c), next = start;
    cartpole_result_t result = CARTPOLE_OK;
    while (!cartpole_is_stop_requested(c)) {
        double until = duration_s > 0 ? fmin(next, start + duration_s) : next;
        cartpole_sleep_until(c, until);
        if (cartpole_is_stop_requested(c))
            break;
        double now = cartpole_get_monotonic_time_s(c);
        if (duration_s > 0 && now - start >= duration_s)
            break;
        int received = cartpole_receive(c);
        if (received < 0) {
            result = cartpole_get_stream_result(c);
            break;
        }
        if (duration_s > 0 && cartpole_get_monotonic_time_s(c) - start >= duration_s)
            break;
        if (received == 0) {
            cartpole_tick_action_t action = {.kind = CARTPOLE_TICK_OBSERVE};
            s->elapsed = now - start;
            s->action = &action;
            if (cartpole_evaluate_measurement(c, evaluate_tick, s) != CARTPOLE_OK) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
            if (cartpole_is_stop_requested(c)) {
                result = CARTPOLE_CANCELLED;
                break;
            }
            if (duration_s > 0 && cartpole_get_monotonic_time_s(c) - start >= duration_s)
                break;
            if (action.kind == CARTPOLE_TICK_FINISH)
                break;
            if (action.kind == CARTPOLE_TICK_VELOCITY) {
                received = cartpole_receive(c);
                if (received < 0) {
                    result = cartpole_get_stream_result(c);
                    break;
                }
                if (duration_s > 0 && cartpole_get_monotonic_time_s(c) - start >= duration_s)
                    break;
                if (received == 0 &&
                    cartpole_send_velocity(c, action.velocity_mm_s,
                                           duration_s > 0 ? start + duration_s : cartpole_get_monotonic_time_s(c) + .020,
                                           0) != CARTPOLE_SEND_COMPLETE) {
                    result = cartpole_get_stream_result(c) == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                                      : cartpole_get_stream_result(c);
                    break;
                }
            } else if (action.kind != CARTPOLE_TICK_OBSERVE) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
        }
        next = control_calculate_next_tick(next, cartpole_get_monotonic_time_s(c), period_s);
    }
    if (cartpole_is_stop_requested(c) && result == CARTPOLE_OK)
        result = CARTPOLE_CANCELLED;
    return result;
}

cartpole_result_t cartpole_run_loop(cartpole_t *c, double period_s, double duration_s,
                                    cartpole_tick_handler_t tick, void *context)
{
    if (!isfinite(period_s) || period_s <= 0 || !isfinite(duration_s) || duration_s < 0 || !tick)
        return CARTPOLE_INVALID_ARGUMENT;
    schedule_t schedule = {.period = period_s, .duration = duration_s, .tick = tick, .context = context};
    return cartpole_execute_operation(c, run_schedule, NULL, &schedule);
}

cartpole_result_t cartpole_control_run_schedule(cartpole_t *c, double period_s, double duration_s,
                                               cartpole_tick_handler_t tick, void *context)
{
    schedule_t schedule = {.period = period_s, .duration = duration_s, .tick = tick, .context = context};
    return run_schedule(c, &schedule);
}
