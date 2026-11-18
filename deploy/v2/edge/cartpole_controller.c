#include "cartpole_internal.h"
#include "control.h"
#include <math.h>

typedef struct {
    control_gate gate;
    cartpole_result_t stop_reason;
    const cartpole_controller_options_t *options;
} controller_t;
static int configuration_matches(const cartpole_configuration_t *expected)
{
    cartpole_configuration_t actual = cartpole_configuration();
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
static control_input control_state(const cartpole_snapshot_t *snapshot, int received)
{
    return (control_input){.ready = snapshot->is_mode_received &&
                                    snapshot->reported_mode == FW_READY &&
                                    snapshot->reported_mode_reason == FW_REASON_NONE &&
                                    !snapshot->has_received_fault_mode,
                           .flags = (int)snapshot->measurement_validity_bits,
                           .age_s = cartpole_state_age_s(*snapshot),
                           .theta_rad = snapshot->theta_urad * 1e-6,
                           .backlog = received > 0,
                           .io_error = received < 0 || snapshot->has_stream_failure_latch};
}
static cartpole_result_t gate_reason(const control_input *input,
                                     const cartpole_snapshot_t *snapshot)
{
    if (input->io_error)
        return cartpole_stream_result() == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE
                                                       : cartpole_stream_result();
    if (snapshot->has_received_fault_mode)
        return CARTPOLE_FAULT;
    if (!input->ready)
        return CARTPOLE_SESSION_UNAVAILABLE;
    if (!(input->flags & 1) || !isfinite(input->age_s) || input->age_s < 0)
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    if (input->age_s >= .100)
        return CARTPOLE_CONTROL_DEADLINE;
    return CARTPOLE_OK;
}
static void observe_control(proto_msg_type_t type, const cartpole_snapshot_t *snapshot,
                            void *context)
{
    (void)type;
    controller_t *controller = context;
    control_input input = control_state(snapshot, 0);
    cartpole_result_t cause = gate_reason(&input, snapshot);
    if (!configuration_matches(&controller->options->expected_configuration)) {
        input.io_error = 1;
        cause = CARTPOLE_CONFIGURATION_CHANGED;
    }
    if (cause == CARTPOLE_OK && controller->gate.has_time &&
        (snapshot->last_state_received_host_monotonic_s < controller->gate.last_observed_at ||
         snapshot->last_state_received_host_monotonic_s - controller->gate.last_observed_at >=
             .100))
        cause = CARTPOLE_CONTROL_DEADLINE;
    if (controller->stop_reason == CARTPOLE_OK && cause != CARTPOLE_OK)
        controller->stop_reason = cause;
    (void)control_observe(&controller->gate, &input,
                          snapshot->last_state_received_host_monotonic_s);
}
static cartpole_result_t stopped_reason(controller_t *controller, const control_input *input,
                                        const cartpole_snapshot_t *snapshot)
{
    if (controller->stop_reason != CARTPOLE_OK)
        return controller->stop_reason;
    cartpole_result_t reason = gate_reason(input, snapshot);
    return reason == CARTPOLE_OK ? CARTPOLE_SESSION_UNAVAILABLE : reason;
}
static cartpole_result_t finish_controller(cartpole_result_t result)
{
    cartpole_set_update_hook(NULL, NULL);
    cartpole_loop_end();
    return result;
}
cartpole_result_t cartpole_run_controller(const cartpole_controller_options_t *options,
                                          cartpole_infer_t infer, void *context)
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
    if (cartpole_loop_begin() != 0)
        return CARTPOLE_CALLBACK_ERROR;
    if (!cartpole_session().connection_open) {
        cartpole_loop_end();
        return CARTPOLE_SESSION_UNAVAILABLE;
    }
    controller_t controller = {.options = options};
    controller.gate.has_time = 1;
    controller.gate.last_observed_at = cartpole_measurement().received_host_monotonic_s;
    cartpole_set_update_hook(observe_control, &controller);
    double next = cartpole_host_monotonic_s(), period = 1. / options->control_hz;
    double track_limit = (options->expected_configuration.track_length_um * .5 -
                          options->expected_configuration.track_margin_um) *
                         1e-6;
    uint64_t consumed = 0;
    while (!cartpole_stop_requested()) {
        cartpole_sleep_to(next);
        if (cartpole_stop_requested())
            break;
        int received = cartpole_pump();
        if (received < 0)
            return finish_controller(cartpole_stream_result());
        cartpole_snapshot_t snapshot = cartpole_snapshot();
        control_input input = control_state(&snapshot, received);
        if (!configuration_matches(&options->expected_configuration))
            return finish_controller(CARTPOLE_CONFIGURATION_CHANGED);
        control_decision decision = control_check(&controller.gate, &input);
        if (decision == CONTROL_STOP)
            return finish_controller(stopped_reason(&controller, &input, &snapshot));
        cartpole_measurement_t measurement = cartpole_measurement();
        if (decision == CONTROL_DRIVE && measurement.measurement_id != consumed) {
            float normalized_action = NAN;
            consumed = measurement.measurement_id;
            unsigned long long generation = controller.gate.generation;
            if (cartpole_callback_enter() != 0)
                return finish_controller(CARTPOLE_CALLBACK_ERROR);
            int inferred = infer(&measurement, &normalized_action, context);
            int violation = cartpole_callback_leave();
            if (inferred || violation || !isfinite(normalized_action))
                return finish_controller(CARTPOLE_CALLBACK_ERROR);
            received = cartpole_pump();
            if (received < 0)
                return finish_controller(cartpole_stream_result());
            snapshot = cartpole_snapshot();
            input = control_state(&snapshot, received);
            if (!configuration_matches(&options->expected_configuration))
                return finish_controller(CARTPOLE_CONFIGURATION_CHANGED);
            decision = control_check(&controller.gate, &input);
            if (decision == CONTROL_STOP)
                return finish_controller(stopped_reason(&controller, &input, &snapshot));
            if (decision == CONTROL_DRIVE && generation == controller.gate.generation &&
                !cartpole_stop_requested() && cartpole_measurement_age_s(measurement) <= .020) {
                double velocity = control_velocity(normalized_action, options->action_scale_mm_s,
                                                   options->velocity_limit_mm_s,
                                                   snapshot.x_um * 1e-6, track_limit);
                if (!isfinite(velocity))
                    return finish_controller(CARTPOLE_CALLBACK_ERROR);
                cartpole_send_result_t sent =
                    cartpole_try_send_vel(velocity, measurement.received_host_monotonic_s + .020);
                if (sent == CARTPOLE_SEND_LATE)
                    return finish_controller(CARTPOLE_TIMEOUT);
                if (sent == CARTPOLE_SEND_ERROR)
                    return finish_controller(cartpole_stream_result() == CARTPOLE_OK
                                                 ? CARTPOLE_SESSION_UNAVAILABLE
                                                 : cartpole_stream_result());
            }
        }
        next = control_next_tick(next, cartpole_host_monotonic_s(), period);
    }
    return finish_controller(CARTPOLE_CANCELLED);
}
