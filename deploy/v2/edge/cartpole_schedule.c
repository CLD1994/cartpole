#include "cartpole_internal.h"
#include "control.h"
#include <math.h>
cartpole_result_t cartpole_run_loop(cartpole_t *c, double period_s, double duration_s,
                                    cartpole_tick_handler_t tick, void *context)
{
    if (!isfinite(period_s) || period_s <= 0 || !isfinite(duration_s) || duration_s < 0 || !tick)
        return CARTPOLE_INVALID_ARGUMENT;
    if (cartpole_loop_begin(c) != 0)
        return CARTPOLE_CALLBACK_ERROR;
    if (!cartpole_get_session_state(c).connection_open) {
        cartpole_loop_end(c);
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
            if (cartpole_callback_enter(c) != 0) {
                result = CARTPOLE_CALLBACK_ERROR;
                break;
            }
            int rc = tick(now - start, &action, context);
            int violation = cartpole_callback_leave(c);
            if (rc || violation) {
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
                                           duration_s > 0 ? start + duration_s : INFINITY,
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
    cartpole_loop_end(c);
    return result;
}
