#include "cartpole_pole_calibration.h"
#include <math.h>
#include <stddef.h>

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
static const double acquisition_timeout_s = 3.0;
static const double verify_timeout_s = 0.25;

typedef enum { WATCH_MODE, COLLECT_SAMPLES, GUARD_SAMPLES, WRITE_SETTING, VERIFY_SAMPLES }
    calibration_phase_t;
typedef struct {
    cartpole_t *cartpole;
    calibration_phase_t phase;
    cartpole_result_t error;
    cartpole_pole_calibration_t *output;
    unsigned samples;
    double first_s, last_s, reference, low, high, last_angle;
    double sin_sum, cos_sum;
    double verify_from;
    int saw_valid_verification, verified;
    cartpole_pole_calibration_detail_t sample_detail;
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

static void fail_sample(calibration_t *run, cartpole_pole_calibration_detail_t detail)
{
    run->sample_detail = detail;
    run->error = CARTPOLE_MEASUREMENT_UNAVAILABLE;
}

static void reset_samples(calibration_t *run)
{
    run->samples = 0;
    run->sin_sum = run->cos_sum = run->low = run->high = 0.0;
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
    if (event->kind != CARTPOLE_EVENT_MEASUREMENT)
        return;
    const cartpole_measurement_t m = event->data.measurement;
    const double angle = m.pole_angle_rad * 1e6;
    const int angle_ok = isfinite(angle) && fabs(angle) <= HALF_TURN_URAD + 0.5;
    const int speed_ok = isfinite(m.pole_angular_velocity_rad_s) &&
                         fabs(m.pole_angular_velocity_rad_s) <= max_speed_rad_s;
    if (run->phase == VERIFY_SAMPLES) {
        if (m.pole_valid && angle_ok && isfinite(m.received_host_monotonic_s) &&
            m.received_host_monotonic_s >= run->verify_from) {
            run->saw_valid_verification = 1;
            run->verified = speed_ok && fabs(wrap_urad(angle - HALF_TURN_URAD)) <= MAX_SPAN_URAD;
            if (run->verified)
                run->output->verified_down_angle_rad = m.pole_angle_rad;
        } else {
            run->verified = 0;
        }
        return;
    }
    if (run->phase != COLLECT_SAMPLES && run->phase != GUARD_SAMPLES)
        return;
    if (!isfinite(m.received_host_monotonic_s) || m.received_host_monotonic_s < run->last_s ||
        m.received_host_monotonic_s - run->last_s > max_sample_gap_s) {
        fail_sample(run, CARTPOLE_POLE_DETAIL_GAP);
        return;
    }
    run->last_s = m.received_host_monotonic_s;
    if (!m.pole_valid) {
        if (run->phase == GUARD_SAMPLES) {
            fail_sample(run, CARTPOLE_POLE_DETAIL_INVALID_POLE);
        } else {
            run->sample_detail = CARTPOLE_POLE_DETAIL_INVALID_POLE;
            reset_samples(run);
        }
        return;
    }
    if (!angle_ok) {
        fail_sample(run, CARTPOLE_POLE_DETAIL_ANGLE);
        return;
    }
    if (!speed_ok) {
        fail_sample(run, CARTPOLE_POLE_DETAIL_MOTION);
        return;
    }
    if (run->phase == GUARD_SAMPLES) {
        if (fabs(wrap_urad(angle - run->last_angle)) > MAX_SPAN_URAD)
            fail_sample(run, CARTPOLE_POLE_DETAIL_SPAN);
        return;
    }
    if (!run->samples) {
        run->reference = angle;
        run->first_s = m.received_host_monotonic_s;
    }
    const double offset = wrap_urad(angle - run->reference);
    run->low = fmin(run->low, offset);
    run->high = fmax(run->high, offset);
    if (run->high - run->low > MAX_SPAN_URAD) {
        fail_sample(run, CARTPOLE_POLE_DETAIL_SPAN);
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
                                              cartpole_pole_settings_t *out)
{
    const cartpole_result_t result =
        cartpole_read_pole_settings(cartpole, out, transaction_timeout_s);
    return run->error != CARTPOLE_OK ? run->error : result;
}

static int pole_settings_valid(const cartpole_pole_settings_t *settings)
{
    return settings->zero_urad >= -HALF_TURN_URAD && settings->zero_urad <= HALF_TURN_URAD &&
           (settings->inverted == 0 || settings->inverted == 1);
}

static cartpole_result_t collect_samples(cartpole_t *cartpole, calibration_t *run)
{
    run->last_s = cartpole_get_monotonic_time_s(cartpole);
    const double deadline = run->last_s + acquisition_timeout_s;
    run->phase = COLLECT_SAMPLES;
    for (;;) {
        if (cartpole_is_stop_requested(cartpole))
            return CARTPOLE_CANCELLED;
        const cartpole_receive_result_t received = cartpole_receive(cartpole);
        if (run->error != CARTPOLE_OK) {
            if (run->error == CARTPOLE_MEASUREMENT_UNAVAILABLE && run->sample_detail)
                run->output->detail = run->sample_detail;
            return run->error;
        }
        if (received == CARTPOLE_RECEIVE_BACKLOG) {
            run->output->detail = CARTPOLE_POLE_DETAIL_BACKLOG;
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        if (received == CARTPOLE_RECEIVE_ERROR) {
            const cartpole_result_t error = cartpole_get_stream_result(cartpole);
            return error == CARTPOLE_OK ? CARTPOLE_TRANSPORT_ERROR : error;
        }
        const double now = cartpole_get_monotonic_time_s(cartpole);
        if (now - run->last_s > max_sample_gap_s) {
            run->output->detail = CARTPOLE_POLE_DETAIL_GAP;
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        if (run->samples && run->last_s - run->first_s >= 1.0 && now <= deadline) {
            if (run->samples >= MIN_SAMPLES) {
                run->sample_detail = CARTPOLE_POLE_DETAIL_NONE;
                return CARTPOLE_OK;
            }
            run->output->detail = CARTPOLE_POLE_DETAIL_COUNT;
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        if (now >= deadline) {
            run->output->detail = run->sample_detail ? run->sample_detail : CARTPOLE_POLE_DETAIL_COUNT;
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        const cartpole_result_t sleep = cartpole_sleep_until(cartpole, fmin(now + 0.001, deadline));
        if (sleep != CARTPOLE_OK)
            return sleep;
    }
}

static cartpole_result_t verify_down_angle(cartpole_t *cartpole, calibration_t *run)
{
    run->output->detail = CARTPOLE_POLE_DETAIL_VERIFY_STREAM;
    run->phase = VERIFY_SAMPLES;
    run->verify_from = cartpole_get_monotonic_time_s(cartpole);
    const double end = run->verify_from + verify_timeout_s;
    for (;;) {
        if (cartpole_is_stop_requested(cartpole))
            return CARTPOLE_CANCELLED;
        const cartpole_receive_result_t received = cartpole_receive(cartpole);
        if (run->error != CARTPOLE_OK)
            return run->error;
        if (received == CARTPOLE_RECEIVE_BACKLOG) {
            run->output->detail = CARTPOLE_POLE_DETAIL_BACKLOG;
            return CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        if (received == CARTPOLE_RECEIVE_ERROR) {
            const cartpole_result_t error = cartpole_get_stream_result(cartpole);
            return error == CARTPOLE_OK ? CARTPOLE_TRANSPORT_ERROR : error;
        }
        const double now = cartpole_get_monotonic_time_s(cartpole);
        if (run->verified && now <= end) {
            run->output->detail = CARTPOLE_POLE_DETAIL_NONE;
            return CARTPOLE_OK;
        }
        if (now >= end) {
            run->output->detail = run->saw_valid_verification ? CARTPOLE_POLE_DETAIL_VERIFY_ANGLE
                                                              : CARTPOLE_POLE_DETAIL_VERIFY_MISSING;
            return run->saw_valid_verification ? CARTPOLE_REPLY_MISMATCH
                                                : CARTPOLE_MEASUREMENT_UNAVAILABLE;
        }
        const cartpole_result_t sleep = cartpole_sleep_until(cartpole, fmin(now + .001, end));
        if (sleep != CARTPOLE_OK)
            return sleep;
    }
}

static cartpole_result_t guard_write(cartpole_configure_stage_t stage, void *context)
{
    calibration_t *run = context;
    if (stage == CARTPOLE_CONFIGURE_ATTEMPTED) {
        /* 发送后固件可能切换角度基准，此时仅继续监督模式。 */
        run->phase = WRITE_SETTING;
        run->output->write_attempted = 1;
        return CARTPOLE_OK;
    }
    if (run->error == CARTPOLE_OK &&
        cartpole_get_monotonic_time_s(run->cartpole) - run->last_s > max_sample_gap_s)
        fail_sample(run, CARTPOLE_POLE_DETAIL_GAP);
    if (run->sample_detail)
        run->output->detail = run->sample_detail;
    return run->error;
}

static cartpole_result_t calibrate(cartpole_t *cartpole, void *context)
{
    calibration_t *run = context;
    run->output->detail = CARTPOLE_POLE_DETAIL_INITIAL_MODE;
    cartpole_result_t result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;

    cartpole_pole_settings_t original, actual;
    run->output->detail = CARTPOLE_POLE_DETAIL_INITIAL_SETTINGS;
    result = settings_transaction(cartpole, run, &original);
    if (result != CARTPOLE_OK)
        return result;
    if (!pole_settings_valid(&original))
        return CARTPOLE_PROTOCOL_ERROR;

    run->output->detail = CARTPOLE_POLE_DETAIL_SAMPLING_STREAM;
    result = collect_samples(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;
    const int64_t old_zero = original.zero_urad;
    const int inverted = (int)original.inverted;
    const double mean = atan2(run->sin_sum, run->cos_sum) / angle_scale;
    const int64_t new_zero = (int64_t)wrap_urad(round((double)old_zero + (inverted ? -mean : mean)
                                                     + HALF_TURN_URAD));
    run->phase = GUARD_SAMPLES;
    run->output->detail = CARTPOLE_POLE_DETAIL_PREWRITE_SETTINGS;
    result = settings_transaction(cartpole, run, &actual);
    if (result != CARTPOLE_OK) {
        if (run->sample_detail)
            run->output->detail = run->sample_detail;
        return result;
    }
    if (actual.zero_urad != old_zero || actual.inverted != inverted)
        return CARTPOLE_CONFIGURATION_CHANGED;
    run->output->detail = CARTPOLE_POLE_DETAIL_PREWRITE_MODE;
    result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK) {
        if (run->sample_detail)
            run->output->detail = run->sample_detail;
        return result;
    }
    if (cartpole_get_monotonic_time_s(cartpole) - run->last_s > max_sample_gap_s) {
        run->output->detail = CARTPOLE_POLE_DETAIL_GAP;
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    }
    if (cartpole_is_stop_requested(cartpole))
        return CARTPOLE_CANCELLED;

    *run->output = (cartpole_pole_calibration_t){.old_zero_urad = old_zero,
                                                .new_zero_urad = new_zero,
                                                .pole_inverted = inverted,
                                                .detail = CARTPOLE_POLE_DETAIL_WRITE};
    result = cartpole_configure_checked(cartpole, CARTPOLE_SETTING_POLE_ZERO, (long)new_zero,
                                         transaction_timeout_s, guard_write, run);
    if (run->error != CARTPOLE_OK || result != CARTPOLE_OK)
        return run->error != CARTPOLE_OK ? run->error : result;
    run->phase = WATCH_MODE;
    run->output->detail = CARTPOLE_POLE_DETAIL_READBACK;
    result = settings_transaction(cartpole, run, &actual);
    if (result != CARTPOLE_OK)
        return result;
    if (actual.zero_urad != new_zero || actual.inverted != inverted)
        return CARTPOLE_REPLY_MISMATCH;
    run->output->write_confirmed = 1;
    run->output->detail = CARTPOLE_POLE_DETAIL_POSTWRITE_MODE;
    result = mode_transaction(cartpole, run);
    if (result != CARTPOLE_OK)
        return result;
    return verify_down_angle(cartpole, run);
}

const char *cartpole_pole_calibration_detail_string(cartpole_pole_calibration_detail_t detail)
{
    switch (detail) {
    case CARTPOLE_POLE_DETAIL_INITIAL_MODE: return "未确认固件处于 DISABLED NONE";
    case CARTPOLE_POLE_DETAIL_INITIAL_SETTINGS: return "未能读取有效的摆杆零点和方向";
    case CARTPOLE_POLE_DETAIL_INVALID_POLE: return "摆杆测量无效，未取得完整稳定的采样窗口";
    case CARTPOLE_POLE_DETAIL_ANGLE: return "摆角超出协议支持的范围";
    case CARTPOLE_POLE_DETAIL_MOTION: return "摆杆角速度无效或超过 20 mrad/s";
    case CARTPOLE_POLE_DETAIL_GAP: return "摆杆测量间隔超过 100 ms 或接收时间无效";
    case CARTPOLE_POLE_DETAIL_SPAN: return "摆角变化超过 0.5°，请让摆杆自然下垂并静止";
    case CARTPOLE_POLE_DETAIL_COUNT: return "稳定的一秒内有效摆杆测量不足 200 帧";
    case CARTPOLE_POLE_DETAIL_SAMPLING_STREAM: return "采集摆杆测量时中断";
    case CARTPOLE_POLE_DETAIL_PREWRITE_SETTINGS: return "写入前未能复核摆杆设置";
    case CARTPOLE_POLE_DETAIL_PREWRITE_MODE: return "写入前未能复核 DISABLED NONE";
    case CARTPOLE_POLE_DETAIL_WRITE: return "未收到预期的零点写入回执";
    case CARTPOLE_POLE_DETAIL_READBACK: return "未确认摆杆零点和方向的读回值";
    case CARTPOLE_POLE_DETAIL_POSTWRITE_MODE: return "写入后未能复核 DISABLED NONE";
    case CARTPOLE_POLE_DETAIL_VERIFY_MISSING: return "写入后未收到新的有效摆角";
    case CARTPOLE_POLE_DETAIL_VERIFY_ANGLE: return "新的摆杆测量未显示静止且摆角接近 ±π";
    case CARTPOLE_POLE_DETAIL_VERIFY_STREAM: return "核验写入后的摆角时中断";
    case CARTPOLE_POLE_DETAIL_BACKLOG: return "串口接收积压，无法确认摆杆测量时效";
    case CARTPOLE_POLE_DETAIL_NONE: return NULL;
    }
    return NULL;
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
