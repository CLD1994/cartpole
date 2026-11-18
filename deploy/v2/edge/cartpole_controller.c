#include "cartpole_control.h"
#include "control.h"
#include <math.h>
#include <stddef.h>

typedef struct {
    cartpole_t *cartpole;
    control_gate gate;
    cartpole_result_t stop_reason;
    const cartpole_controller_options_t *options;
    cartpole_infer_t infer;
    void *infer_context;
    cartpole_measurement_t inference_measurement;
    float normalized_action;
} controller_t;
static int configuration_matches(cartpole_t *c, const cartpole_configuration_t *expected)
{
    cartpole_configuration_t actual = cartpole_get_configuration(c);
    return actual.complete && expected->complete &&
           actual.calibration_state == expected->calibration_state &&
           actual.track_length_um == expected->track_length_um &&
           actual.calibration_saved == expected->calibration_saved &&
           actual.calibration_storage_ok == expected->calibration_storage_ok &&
           actual.velocity_limit_mm_s == expected->velocity_limit_mm_s &&
           actual.acceleration_limit_mm_s2 == expected->acceleration_limit_mm_s2 &&
           actual.watchdog_timeout_ms == expected->watchdog_timeout_ms &&
           actual.track_margin_um == expected->track_margin_um;
}
typedef struct {
    cartpole_measurement_t measurement;
    cartpole_session_t session;
} control_snapshot_t;
static control_snapshot_t current_state(cartpole_t *c)
{
    return (control_snapshot_t){cartpole_get_latest_measurement(c), cartpole_get_session_state(c)};
}
static control_input control_state(cartpole_t *c, const control_snapshot_t *snapshot, int received)
{
    return (control_input){.ready =
                               snapshot->session.is_mode_received &&
                               snapshot->session.reported_mode == CARTPOLE_MODE_READY &&
                               snapshot->session.reported_mode_reason == CARTPOLE_REASON_NONE &&
                               !snapshot->session.has_received_fault_mode,
                           .flags = (snapshot->measurement.cart_valid ? 1 : 0) |
                                    (snapshot->measurement.pole_valid ? 2 : 0),
                           .age_s = cartpole_calculate_measurement_age_s(c, snapshot->measurement),
                           .theta_rad = snapshot->measurement.pole_angle_rad,
                           .backlog = received > 0,
                           .io_error = received < 0 || snapshot->session.has_stream_failure_latch};
}
static cartpole_result_t gate_reason(cartpole_t *c, const control_input *input,
                                     const control_snapshot_t *snapshot)
{
    if (input->io_error)
        return cartpole_get_stream_result(c) == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                        : cartpole_get_stream_result(c);
    if (snapshot->session.has_received_fault_mode)
        return CARTPOLE_FAULT;
    if (!input->ready)
        return CARTPOLE_SESSION_UNAVAILABLE;
    if (!(input->flags & 1) || !isfinite(input->age_s) || input->age_s < 0)
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    if (input->age_s >= .100)
        return CARTPOLE_CONTROL_DEADLINE;
    return CARTPOLE_OK;
}
static void observe_control(const cartpole_event_t *event, void *context)
{
    if (event->kind == CARTPOLE_EVENT_STEP_LOSS)
        return;
    controller_t *controller = context;
    cartpole_t *c = controller->cartpole;
    control_snapshot_t state = current_state(c);
    const control_snapshot_t *snapshot = &state;
    control_input input = control_state(c, snapshot, 0);
    cartpole_result_t cause = gate_reason(c, &input, snapshot);
    if (!configuration_matches(c, &controller->options->expected_configuration)) {
        input.io_error = 1;
        cause = CARTPOLE_CONFIGURATION_CHANGED;
    }
    if (cause == CARTPOLE_OK && controller->gate.has_time &&
        (snapshot->measurement.received_host_monotonic_s < controller->gate.last_observed_at ||
         snapshot->measurement.received_host_monotonic_s - controller->gate.last_observed_at >=
             .100))
        cause = CARTPOLE_CONTROL_DEADLINE;
    if (controller->stop_reason == CARTPOLE_OK && cause != CARTPOLE_OK)
        controller->stop_reason = cause;
    (void)control_observe(&controller->gate, &input,
                          snapshot->measurement.received_host_monotonic_s);
}
static cartpole_result_t stopped_reason(controller_t *controller, const control_input *input,
                                        const control_snapshot_t *snapshot)
{
    if (controller->stop_reason != CARTPOLE_OK)
        return controller->stop_reason;
    cartpole_result_t reason = gate_reason(controller->cartpole, input, snapshot);
    return reason == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE : reason;
}
static int evaluate_policy(void *context)
{
    controller_t *c = context;
    return c->infer(&c->inference_measurement, &c->normalized_action, c->infer_context);
}
static cartpole_result_t run_controller(cartpole_t *c, void *context)
{
    controller_t *state = context;
    const cartpole_controller_options_t *options = state->options;
    if (!cartpole_get_session_state(c).connection_open) {
        return CARTPOLE_SESSION_UNAVAILABLE;
    }
    state->gate.has_time = 1;
    state->gate.last_observed_at = cartpole_get_latest_measurement(c).received_host_monotonic_s;
    double next = cartpole_get_monotonic_time_s(c), period = 1. / options->control_hz;
    double track_limit = (options->expected_configuration.track_length_um * .5 -
                          options->expected_configuration.track_margin_um) *
                         1e-6;
    uint64_t consumed = 0;
    while (!cartpole_is_stop_requested(c)) {
        cartpole_sleep_until(c, next);
        if (cartpole_is_stop_requested(c))
            break;
        int received = cartpole_receive(c);
        if (received < 0)
            return cartpole_get_stream_result(c);
        control_snapshot_t snapshot = current_state(c);
        control_input input = control_state(c, &snapshot, received);
        if (!configuration_matches(c, &options->expected_configuration))
            return CARTPOLE_CONFIGURATION_CHANGED;
        control_decision decision = control_check(&state->gate, &input);
        if (decision == CONTROL_STOP)
            return stopped_reason(state, &input, &snapshot);
        cartpole_measurement_t measurement = cartpole_get_latest_measurement(c);
        if (decision == CONTROL_DRIVE && measurement.measurement_id != consumed) {
            state->normalized_action = NAN;
            state->inference_measurement = measurement;
            consumed = measurement.measurement_id;
            unsigned long long generation = state->gate.generation;
            if (cartpole_evaluate_measurement(c, evaluate_policy, state) != CARTPOLE_OK ||
                !isfinite(state->normalized_action))
                return CARTPOLE_CALLBACK_ERROR;
            received = cartpole_receive(c);
            if (received < 0)
                return cartpole_get_stream_result(c);
            snapshot = current_state(c);
            input = control_state(c, &snapshot, received);
            if (!configuration_matches(c, &options->expected_configuration))
                return CARTPOLE_CONFIGURATION_CHANGED;
            decision = control_check(&state->gate, &input);
            if (decision == CONTROL_STOP)
                return stopped_reason(state, &input, &snapshot);
            if (decision == CONTROL_DRIVE && generation == state->gate.generation &&
                !cartpole_is_stop_requested(c) && cartpole_calculate_measurement_age_s(c, measurement) <= .020) {
                double velocity = control_calculate_velocity(
                    state->normalized_action, options->action_scale_mm_s, options->velocity_limit_mm_s,
                    snapshot.measurement.cart_position_m, track_limit);
                if (!isfinite(velocity))
                    return CARTPOLE_CALLBACK_ERROR;
                cartpole_send_result_t sent = cartpole_send_velocity(
                    c, velocity, measurement.received_host_monotonic_s + .020, 1);
                if (sent == CARTPOLE_SEND_LATE)
                    return CARTPOLE_TIMEOUT;
                if (sent == CARTPOLE_SEND_ERROR)
                    return cartpole_get_stream_result(c) == CARTPOLE_OK
                                                    ? CARTPOLE_SESSION_UNAVAILABLE
                                                    : cartpole_get_stream_result(c);
            }
        }
        next = control_calculate_next_tick(next, cartpole_get_monotonic_time_s(c), period);
    }
    return CARTPOLE_CANCELLED;
}

cartpole_result_t cartpole_run_controller(cartpole_t *c,
    const cartpole_controller_options_t *options, cartpole_infer_t infer, void *context)
{
    if (!options || !infer || !isfinite(options->control_hz) ||
        (options->control_hz != 400 && options->control_hz != 500) ||
        !isfinite(options->action_scale_mm_s) || options->action_scale_mm_s <= 0 ||
        !isfinite(options->velocity_limit_mm_s) || options->velocity_limit_mm_s <= 0 ||
        !options->expected_configuration.complete ||
        options->expected_configuration.track_length_um <= 0 ||
        options->expected_configuration.track_margin_um < 0 ||
        options->expected_configuration.track_margin_um >=
            options->expected_configuration.track_length_um * .5)
        return CARTPOLE_INVALID_ARGUMENT;
    controller_t state = {.cartpole = c, .options = options, .infer = infer, .infer_context = context};
    return cartpole_execute_operation(c, run_controller, observe_control, &state);
}
