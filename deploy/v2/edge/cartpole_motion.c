#include "cartpole_internal.h"
#include "config.h"
#include <math.h>

typedef struct {
    cartpole_t *cartpole;
    double velocity;
    cartpole_result_t rejected;
} hold_t;
static int hold_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    hold_t *hold = context;
    cartpole_t *c = hold->cartpole;
    cartpole_measurement_t measurement = cartpole_get_latest_measurement(c);
    cartpole_session_t session = cartpole_get_session_state(c);
    if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_READY ||
        session.has_received_fault_mode || session.has_stream_failure_latch ||
        !measurement.cart_valid || cartpole_measurement_age_s(c, measurement) < 0 ||
        cartpole_measurement_age_s(c, measurement) > CFG_STATE_TIMEOUT_S) {
        if (session.has_stream_failure_latch)
            hold->rejected = cartpole_stream_result(c);
        else if (session.has_received_fault_mode)
            hold->rejected = CARTPOLE_FAULT;
        else if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_READY)
            hold->rejected = CARTPOLE_SESSION_UNAVAILABLE;
        else
            hold->rejected = CARTPOLE_MEASUREMENT_UNAVAILABLE;
        action->kind = CARTPOLE_TICK_FINISH;
        return 0;
    }
    action->kind = CARTPOLE_TICK_VELOCITY;
    action->velocity_mm_s = hold->velocity;
    return 0;
}
cartpole_result_t cartpole_hold_velocity(cartpole_t *c, double velocity, double duration)
{
    if (!isfinite(velocity) || fabs(velocity) > 1.0e9 || !isfinite(duration) || duration <= 0)
        return CARTPOLE_INVALID_ARGUMENT;
    cartpole_result_t preflight = cartpole_query_mode(c, 1.);
    if (preflight != CARTPOLE_OK)
        return preflight;
    preflight = cartpole_wait_measurement(c, CFG_STATE_TIMEOUT_S);
    if (preflight != CARTPOLE_OK)
        return preflight;
    hold_t hold = {.cartpole = c, .velocity = velocity};
    cartpole_result_t result = cartpole_run_loop(c, .020, duration, hold_tick, &hold);
    if (cartpole_send_velocity(c, 0., cartpole_host_monotonic_s(c) + .020, 0) !=
            CARTPOLE_SEND_COMPLETE &&
        result == CARTPOLE_OK)
        result = cartpole_stream_result(c) == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                          : cartpole_stream_result(c);
    if (hold.rejected != CARTPOLE_OK && result == CARTPOLE_OK)
        result = hold.rejected;
    return result;
}
cartpole_result_t cartpole_move_to_center(cartpole_t *c, double timeout_s)
{
    if (!isfinite(timeout_s) || timeout_s <= 0)
        return CARTPOLE_INVALID_ARGUMENT;
    if (cartpole_loop_begin(c) != 0)
        return CARTPOLE_CALLBACK_ERROR;
    double end = cartpole_host_monotonic_s(c) + timeout_s;
    cartpole_result_t result = cartpole_query_mode(c, timeout_s);
    if (result != CARTPOLE_OK) {
        cartpole_loop_end(c);
        return result;
    }
    int zero_sent = 0;
    uint64_t zero_measurement = 0;
    result = CARTPOLE_TIMEOUT;
    while (!cartpole_stop_requested(c) && cartpole_host_monotonic_s(c) < end) {
        int received = cartpole_receive(c);
        if (received < 0) {
            result = cartpole_stream_result(c);
            break;
        }
        if (cartpole_stop_requested(c)) {
            result = CARTPOLE_CANCELLED;
            break;
        }
        if (cartpole_host_monotonic_s(c) >= end)
            break;
        if (received > 0) {
            cartpole_sleep_to(c, cartpole_host_monotonic_s(c) + .001);
            continue;
        }
        cartpole_measurement_t measurement = cartpole_get_latest_measurement(c);
        cartpole_session_t session = cartpole_get_session_state(c);
        if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_READY ||
            !measurement.cart_valid || cartpole_measurement_age_s(c, measurement) < 0 ||
            cartpole_measurement_age_s(c, measurement) > CFG_STATE_TIMEOUT_S) {
            result = CARTPOLE_SESSION_UNAVAILABLE;
            break;
        }
        if (zero_sent && measurement.measurement_id != zero_measurement &&
            measurement.cart_velocity_m_s == 0.) {
            result = CARTPOLE_OK;
            break;
        }
        double velocity = zero_sent || fabs(measurement.cart_position_m) < .005 ? 0.
                          : measurement.cart_position_m > 0 ? -CFG_CENTER_VEL_MMS
                                                            : CFG_CENTER_VEL_MMS;
        if (cartpole_send_velocity(c, velocity, end, 0) != CARTPOLE_SEND_COMPLETE) {
            result = cartpole_stream_result(c) == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                              : cartpole_stream_result(c);
            break;
        }
        if (velocity == 0. && !zero_sent) {
            zero_sent = 1;
            zero_measurement = measurement.measurement_id;
        }
        cartpole_sleep_to(c, fmin(end, cartpole_host_monotonic_s(c) + .005));
    }
    if (cartpole_stop_requested(c) && result == CARTPOLE_TIMEOUT)
        result = CARTPOLE_CANCELLED;
    if (result != CARTPOLE_OK)
        (void)cartpole_send_velocity(c, 0., cartpole_host_monotonic_s(c) + .020, 0);
    cartpole_loop_end(c);
    return result;
}
