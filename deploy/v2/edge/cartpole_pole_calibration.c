#include "cartpole_pole_calibration.h"
#include <math.h>

enum {
    HALF_TURN_URAD = 3141592,
    TURN_URAD = 6283185,
    MAX_SPAN_URAD = 8727, /* 0.5° */
    MIN_SAMPLES = 200
};
static const double angle_scale = 6.28318530717958647692 / TURN_URAD;
static const double max_sample_gap_s = 0.1;
static const double max_speed_rad_s = 0.020;
static const double transaction_timeout_s = 1.5;

typedef enum { WATCH_MODE, COLLECT_SAMPLES, GUARD_SAMPLES, WRITE_SETTING } calibration_phase_t;
typedef struct {
    cartpole_t *cartpole;
    calibration_phase_t phase;
    cartpole_result_t error;
    cartpole_pole_calibration_t *output;
    unsigned samples;
    double last_s, reference, low, high, last_angle;
    double sin_sum, cos_sum;
} calibration_t;

static double wrap_urad(double value)
{
    double r = fmod(value + HALF_TURN_URAD, TURN_URAD);
    return (r < 0 ? r + TURN_URAD : r) - HALF_TURN_URAD;
}

static cartpole_result_t check_mode(cartpole_session_t state)
{
    if (state.is_mode_received && state.reported_mode == CARTPOLE_MODE_FAULT)
        return CARTPOLE_FAULT;
    if (!state.is_mode_received || state.reported_mode != CARTPOLE_MODE_DISABLED ||
        state.reported_mode_reason != CARTPOLE_REASON_NONE)
        return CARTPOLE_REJECTED;
    return CARTPOLE_OK;
}

static void observe(const cartpole_event_t *event, void *context)
{
    calibration_t *run = context;
    if (run->error != CARTPOLE_OK)
        return;
    if (event->kind == CARTPOLE_EVENT_SESSION) {
        run->error = check_mode(event->data.session);
        /* 配置事务先排空积压，再写串口。发现禁止状态即取消这次会话，
         * 避免接下来同批的 DISABLED 覆盖前面的状态变化并继续写入。 */
        if (run->error != CARTPOLE_OK && run->phase == WRITE_SETTING)
            cartpole_request_stop(run->cartpole);
        return;
    }
    if (event->kind != CARTPOLE_EVENT_MEASUREMENT ||
        (run->phase != COLLECT_SAMPLES && run->phase != GUARD_SAMPLES))
        return;
    const cartpole_measurement_t m = event->data.measurement;
    const double angle = m.pole_angle_rad * 1e6;
    if (!m.pole_valid || !isfinite(angle) || fabs(angle) > HALF_TURN_URAD + 0.5 ||
        !isfinite(m.pole_angular_velocity_rad_s) ||
        fabs(m.pole_angular_velocity_rad_s) > max_speed_rad_s ||
        !isfinite(m.received_host_monotonic_s) || m.received_host_monotonic_s < run->last_s ||
        m.received_host_monotonic_s - run->last_s > max_sample_gap_s) {
        run->error = CARTPOLE_MEASUREMENT_UNAVAILABLE;
        return;
    }
    run->last_s = m.received_host_monotonic_s;
    if (run->phase == GUARD_SAMPLES) {
        if (fabs(wrap_urad(angle - run->last_angle)) > MAX_SPAN_URAD)
            run->error = CARTPOLE_MEASUREMENT_UNAVAILABLE;
        return;
    }
    if (!run->samples)
        run->reference = angle;
    const double offset = wrap_urad(angle - run->reference);
    run->low = fmin(run->low, offset);
    run->high = fmax(run->high, offset);
    if (run->high - run->low > MAX_SPAN_URAD) {
        run->error = CARTPOLE_MEASUREMENT_UNAVAILABLE;
        return;
    }
    run->sin_sum += sin(angle * angle_scale);
    run->cos_sum += cos(angle * angle_scale);
    run->last_angle = angle;
    run->samples++;
}

static cartpole_result_t mode_transaction(cartpole_t *cartpole, calibration_t *run)
{
    const cartpole_result_t result = cartpole_query_mode(cartpole, transaction_timeout_s);
    if (run->error != CARTPOLE_OK)
        return run->error;
    return result != CARTPOLE_OK ? result : check_mode(cartpole_get_session_state(cartpole));
}

static cartpole_result_t settings_transaction(cartpole_t *cartpole, calibration_t *run,
                                              cartpole_settings_t *out)
{
    const cartpole_result_t result = cartpole_read_settings(cartpole, out, transaction_timeout_s);
    return run->error != CARTPOLE_OK ? run->error : result;
}

static int pole_settings_valid(const cartpole_settings_t *settings)
{
    return settings->values[CARTPOLE_SETTING_POLE_ZERO] >= -HALF_TURN_URAD &&
           settings->values[CARTPOLE_SETTING_POLE_ZERO] <= HALF_TURN_URAD &&
           (settings->values[CARTPOLE_SETTING_POLE_INVERT] == 0 ||
            settings->values[CARTPOLE_SETTING_POLE_INVERT] == 1);
}

static cartpole_result_t collect_samples(cartpole_t *cartpole, calibration_t *run)
{
    run->last_s = cartpole_get_monotonic_time_s(cartpole);
    const double end = run->last_s + 1.0;
    run->phase = COLLECT_SAMPLES;
    for (;;) {
        if (cartpole_is_stop_requested(cartpole))
            return CARTPOLE_CANCELLED;
        const cartpole_receive_result_t received = cartpole_receive(cartpole);
        if (run->error != CARTPOLE_OK)
            return run->error;
        if (received == CARTPOLE_RECEIVE_ERROR) {
            const cartpole_result_t error = cartpole_get_stream_result(cartpole);
            return error == CARTPOLE_OK ? CARTPOLE_TRANSPORT_ERROR : error;
        }
        const double now = cartpole_get_monotonic_time_s(cartpole);
        if (now - run->last_s > max_sample_gap_s)
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        if (now >= end)
            return run->samples >= MIN_SAMPLES ? CARTPOLE_OK : CARTPOLE_MEASUREMENT_UNAVAILABLE;
        const cartpole_result_t sleep = cartpole_sleep_until(cartpole, fmin(now + 0.001, end));
        if (sleep != CARTPOLE_OK)
            return sleep;
    }
}

static cartpole_result_t calibrate(cartpole_t *cartpole, void *context)
{
    calibration_t *run = context;
    cartpole_result_t result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;

    cartpole_settings_t original, actual;
    result = settings_transaction(cartpole, run, &original);
    if (result != CARTPOLE_OK)
        return result;
    if (!pole_settings_valid(&original))
        return CARTPOLE_PROTOCOL_ERROR;

    result = collect_samples(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;
    const int64_t old_zero = original.values[CARTPOLE_SETTING_POLE_ZERO];
    const int inverted = (int)original.values[CARTPOLE_SETTING_POLE_INVERT];
    const double mean = atan2(run->sin_sum, run->cos_sum) / angle_scale;
    const int64_t new_zero = (int64_t)wrap_urad(round((double)old_zero + (inverted ? -mean : mean)
                                                     + HALF_TURN_URAD));
    run->phase = GUARD_SAMPLES;
    result = settings_transaction(cartpole, run, &actual);
    if (result != CARTPOLE_OK)
        return result;
    if (actual.values[CARTPOLE_SETTING_POLE_ZERO] != old_zero ||
        actual.values[CARTPOLE_SETTING_POLE_INVERT] != inverted)
        return CARTPOLE_CONFIGURATION_CHANGED;
    result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;
    if (cartpole_get_monotonic_time_s(cartpole) - run->last_s > max_sample_gap_s)
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    if (cartpole_is_stop_requested(cartpole))
        return CARTPOLE_CANCELLED;

    /* 写入后观测切换到新零点，不能继续用旧角度检查。 */
    run->phase = WRITE_SETTING;
    run->output->write_attempted = 1;
    result = cartpole_configure(cartpole, CARTPOLE_SETTING_POLE_ZERO, (long)new_zero,
                                transaction_timeout_s);
    if (run->error != CARTPOLE_OK || result != CARTPOLE_OK)
        return run->error != CARTPOLE_OK ? run->error : result;
    run->phase = WATCH_MODE;
    result = settings_transaction(cartpole, run, &actual);
    if (result != CARTPOLE_OK)
        return result;
    result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;
    if (actual.values[CARTPOLE_SETTING_POLE_ZERO] != new_zero ||
        actual.values[CARTPOLE_SETTING_POLE_INVERT] != inverted)
        return CARTPOLE_REPLY_MISMATCH;

    *run->output = (cartpole_pole_calibration_t){ .old_zero_urad = old_zero,
                                                .new_zero_urad = new_zero,
                                                .pole_inverted = inverted,
                                                .write_attempted = 1 };
    return CARTPOLE_OK;
}

cartpole_result_t cartpole_calibrate_pole_down(cartpole_t *cartpole, cartpole_pole_calibration_t *out)
{
    if (!out)
        return CARTPOLE_INVALID_ARGUMENT;
    *out = (cartpole_pole_calibration_t){0};
    if (!cartpole)
        return CARTPOLE_INVALID_ARGUMENT;
    calibration_t run = {.cartpole = cartpole, .output = out};
    return cartpole_execute_operation(cartpole, calibrate, observe, &run);
}
