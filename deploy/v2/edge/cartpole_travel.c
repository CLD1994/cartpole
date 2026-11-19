#include "cartpole_travel.h"
#include "config.h"
#include <math.h>
#include <stddef.h>

static const double keepalive_s = .020;
static const double settle_timeout_s = 2.;
static const double stationary_s = .200;
static const double poll_s = .001;
/* 500Hz 名义出帧相位余量，不是设备到主机的链路延迟上界。 */
static const double observation_period_s = .002;

typedef enum { PREFLIGHT, STARTING, RUNNING, STOPPING } phase_t;

typedef struct {
    double since, last, low_um, high_um;
    int active, confirmed;
} stillness_t;

typedef struct {
    cartpole_t *cartpole;
    cartpole_travel_options_t options;
    cartpole_travel_report_t *report;
    cartpole_configuration_t config;
    phase_t phase;
    cartpole_result_t failure;
    double velocity, direction, physical_um, soft_um, target_um, extreme_um;
    double inflight_s, stop_not_before, recovery_outer_um;
    double first_write_done;
    int physical_entered, soft_entered, braking, has_complete_nonzero;
    stillness_t still;
} travel_t;

static void remember_failure(travel_t *t, cartpole_result_t failure)
{
    if (t->failure == CARTPOLE_OK)
        t->failure = failure;
}

static cartpole_result_t stream_failure(cartpole_t *c, cartpole_result_t fallback)
{
    cartpole_result_t error = cartpole_get_stream_result(c);
    return error == CARTPOLE_OK ? fallback : error;
}

static cartpole_result_t check_session(cartpole_t *c, cartpole_session_t session)
{
    if (session.has_received_fault_mode ||
        (session.is_mode_received && session.reported_mode == CARTPOLE_MODE_FAULT))
        return CARTPOLE_FAULT;
    if (session.has_stream_failure_latch)
        return stream_failure(c, CARTPOLE_SESSION_UNAVAILABLE);
    if (!session.connection_open || !session.is_mode_received ||
        session.reported_mode != CARTPOLE_MODE_READY ||
        session.reported_mode_reason != CARTPOLE_REASON_NONE)
        return CARTPOLE_SESSION_UNAVAILABLE;
    return CARTPOLE_OK;
}

/* STATE 的位置字段单位为整数微米。恢复微米格点，避免边界比较被单位转换误差扰动。 */
static double position_um(cartpole_measurement_t m)
{
    return round(m.cart_position_m * 1e6);
}

static int cart_state_is_fresh(cartpole_t *c, cartpole_measurement_t m)
{
    double age = cartpole_calculate_measurement_age_s(c, m);
    return m.measurement_id != 0 && m.cart_valid && isfinite(position_um(m)) &&
           isfinite(m.cart_velocity_m_s) && age >= 0 && age <= CFG_STATE_TIMEOUT_S;
}

static cartpole_result_t check_position(travel_t *t, cartpole_measurement_t m)
{
    double x = position_um(m);
    double delta = t->direction * (x - t->extreme_um);
    if (!t->physical_entered) {
        const double recovery_noise_um = 100.;
        if (fabs(x) > t->recovery_outer_um || delta < -recovery_noise_um)
            return CARTPOLE_REJECTED;
        if (t->direction * x >= -t->physical_um + recovery_noise_um)
            t->physical_entered = 1;
    }
    if ((t->physical_entered && fabs(x) > t->physical_um) ||
        t->direction * x > t->soft_um ||
        (t->soft_entered && t->direction * x < -t->soft_um) || delta < -3000.)
        return CARTPOLE_REJECTED;
    if (fabs(x) <= t->soft_um)
        t->soft_entered = 1;
    if (delta > 0)
        t->extreme_um = x;
    return CARTPOLE_OK;
}

static cartpole_result_t check_motion(travel_t *t, cartpole_measurement_t m,
                                     double *budget_mm, double *watchdog_mm)
{
    if (!cart_state_is_fresh(t->cartpole, m))
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    double velocity_mm_s = fmax(fabs(t->velocity), round(fabs(m.cart_velocity_m_s) * 1000.));
    if (velocity_mm_s > t->config.velocity_limit_mm_s)
        return CARTPOLE_REJECTED;
    cartpole_result_t r = check_position(t, m);
    if (r != CARTPOLE_OK)
        return r;

    double age = cartpole_calculate_measurement_age_s(t->cartpole, m);
    if (!(age >= 0 && age <= CFG_STATE_TIMEOUT_S))
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    double normal = velocity_mm_s * (fmax(age, observation_period_s) + t->inflight_s) +
                    velocity_mm_s * velocity_mm_s / (2. * t->config.acceleration_limit_mm_s2);
    double watchdog = normal + velocity_mm_s * (t->config.watchdog_timeout_ms / 1000.);
    if (!isfinite(normal) || !isfinite(watchdog))
        return CARTPOLE_INVALID_ARGUMENT;
    if (budget_mm)
        *budget_mm = normal;
    if (watchdog_mm)
        *watchdog_mm = watchdog;
    if (t->direction * (t->target_um - position_um(m)) / 1000. <= normal)
        t->braking = 1;
    return CARTPOLE_OK;
}

static void update_stillness(stillness_t *s, cartpole_measurement_t m, double not_before)
{
    double now = m.received_host_monotonic_s, x = position_um(m);
    if (m.cart_velocity_m_s != 0 || now < not_before) {
        *s = (stillness_t){0};
        return;
    }
    if (!s->active || now < s->last || now - s->last > CFG_STATE_TIMEOUT_S) {
        *s = (stillness_t){.since = now, .last = now, .low_um = x, .high_um = x, .active = 1};
        return;
    }
    s->last = now;
    s->low_um = fmin(s->low_um, x);
    s->high_um = fmax(s->high_um, x);
    if (s->high_um - s->low_um > 100.) {
        *s = (stillness_t){.since = now, .last = now, .low_um = x, .high_um = x, .active = 1};
        return;
    }
    s->confirmed = now - s->since >= stationary_s;
}

static void observe_speed(travel_t *t, cartpole_measurement_t m)
{
    if (!t->has_complete_nonzero)
        return;
    double elapsed = m.received_host_monotonic_s - t->first_write_done;
    double speed = round(m.cart_velocity_m_s * 1000.);
    if (elapsed < 0 || t->direction * speed <= 0.)
        return;
    if (!t->report->observed_peak || fabs(speed) > fabs(t->report->peak_velocity_mm_s)) {
        t->report->observed_peak = 1;
        t->report->peak_velocity_mm_s = speed;
        t->report->peak_at_s = elapsed;
    }
    if (!t->report->observed_target_95 && fabs(speed) * 100. >= fabs(t->velocity) * 95.) {
        t->report->observed_target_95 = 1;
        t->report->target_95_s = elapsed;
    }
}

static void observe(const cartpole_event_t *event, void *context)
{
    travel_t *t = context;
    switch (event->kind) {
    case CARTPOLE_EVENT_UNRECOGNIZED_LINE:
        if (t->phase != PREFLIGHT)
            remember_failure(t, CARTPOLE_PROTOCOL_ERROR);
        if (t->phase == STOPPING)
            t->still = (stillness_t){0};
        break;
    case CARTPOLE_EVENT_FIRMWARE_ERROR:
        remember_failure(t, CARTPOLE_REJECTED);
        break;
    case CARTPOLE_EVENT_SESSION:
        if (t->phase != PREFLIGHT)
            remember_failure(t, check_session(t->cartpole, event->data.session));
        break;
    case CARTPOLE_EVENT_CONFIGURATION:
        if (t->phase != PREFLIGHT)
            remember_failure(t, CARTPOLE_CONFIGURATION_CHANGED);
        break;
    case CARTPOLE_EVENT_STEP_LOSS:
        /* 事件仅用于报告；实际 MODE FAULT 仍通过会话门禁终止。 */
        break;
    case CARTPOLE_EVENT_MEASUREMENT: {
        cartpole_measurement_t m = event->data.measurement;
        if (t->phase == STARTING || t->phase == RUNNING) {
            remember_failure(t, check_motion(t, m, NULL, NULL));
            if (t->failure == CARTPOLE_OK) {
                if (t->phase == STARTING)
                    update_stillness(&t->still, m, 0.);
                else
                    observe_speed(t, m);
            }
        } else if (t->phase == STOPPING) {
            if (!cart_state_is_fresh(t->cartpole, m)) {
                t->still = (stillness_t){0};
                break;
            }
            remember_failure(t, check_position(t, m));
            t->report->has_final_position = 1;
            t->report->final_position_mm = position_um(m) / 1000.;
            if (t->report->zero_requested)
                update_stillness(&t->still, m, t->stop_not_before);
        }
        break;
    }
    }
}

static cartpole_result_t make_plan(travel_t *t)
{
    cartpole_t *c = t->cartpole;
    cartpole_result_t r = cartpole_refresh_configuration(c, 1.);
    if (r == CARTPOLE_OK && t->failure == CARTPOLE_OK)
        r = cartpole_query_mode(c, 1.);
    if (r == CARTPOLE_OK && t->failure == CARTPOLE_OK)
        r = cartpole_wait_for_measurement(c, CFG_STATE_TIMEOUT_S);
    if (t->failure != CARTPOLE_OK)
        return t->failure;
    if (r != CARTPOLE_OK)
        return r;

    cartpole_configuration_t cfg = cartpole_get_configuration(c);
    if (!cfg.complete || cfg.has_legacy_watchdog_deceleration || cfg.calibration_state != CARTPOLE_CALIBRATION_READY ||
        !cfg.calibration_saved || !cfg.calibration_storage_ok || cfg.track_length_um <= 0 ||
        cfg.acceleration_limit_mm_s2 <= 0 || cfg.watchdog_timeout_ms <= 20 ||
        cfg.track_margin_um < 0 || fabs(t->velocity) > cfg.velocity_limit_mm_s)
        return CARTPOLE_REJECTED;

    t->config = cfg;
    t->physical_um = (double)(cfg.track_length_um / 2);
    t->soft_um = t->physical_um - cfg.track_margin_um;
    t->target_um = t->direction * (t->soft_um - t->options.reserve_mm * 1000.);
    if (!(t->direction * t->target_um > 0.))
        return CARTPOLE_REJECTED;

    cartpole_measurement_t m = cartpole_get_latest_measurement(c);
    if (!cart_state_is_fresh(c, m))
        return CARTPOLE_MEASUREMENT_UNAVAILABLE;
    t->extreme_um = position_um(m);
    t->physical_entered = fabs(t->extreme_um) <= t->physical_um;
    t->recovery_outer_um = t->physical_um + t->options.start_tolerance_mm * 1000.;
    if (fabs(t->extreme_um) > t->recovery_outer_um ||
        t->direction * (t->target_um - t->extreme_um) <= 0.)
        return CARTPOLE_REJECTED;

    r = check_motion(t, m, &t->report->braking_budget_mm, &t->report->watchdog_budget_mm);
    if (r != CARTPOLE_OK)
        return r;
    if (t->braking)
        return CARTPOLE_REJECTED;
    t->report->physical_mm = t->physical_um / 1000.;
    t->report->soft_mm = t->soft_um / 1000.;
    t->report->target_mm = t->target_um / 1000.;
    t->report->plan_valid = 1;
    return CARTPOLE_OK;
}

static cartpole_result_t poll_motion(travel_t *t)
{
    cartpole_t *c = t->cartpole;
    if (cartpole_is_stop_requested(c))
        return CARTPOLE_CANCELLED;
    if (t->failure != CARTPOLE_OK)
        return t->failure;

    cartpole_receive_result_t received = cartpole_receive(c);
    if (received == CARTPOLE_RECEIVE_ERROR)
        return stream_failure(c, CARTPOLE_TRANSPORT_ERROR);
    if (received == CARTPOLE_RECEIVE_BACKLOG)
        return CARTPOLE_CONTROL_DEADLINE;
    if (cartpole_is_stop_requested(c))
        return CARTPOLE_CANCELLED;
    if (t->failure != CARTPOLE_OK)
        return t->failure;
    cartpole_result_t r = check_session(c, cartpole_get_session_state(c));
    if (r == CARTPOLE_OK)
        r = check_motion(t, cartpole_get_latest_measurement(c), NULL, NULL);
    return r;
}

static cartpole_result_t wait_at_start(travel_t *t)
{
    cartpole_t *c = t->cartpole;
    cartpole_result_t r = check_session(c, cartpole_get_session_state(c));
    if (r != CARTPOLE_OK)
        return r;
    t->phase = STARTING;
    double end = cartpole_get_monotonic_time_s(c) + settle_timeout_s;
    while (cartpole_get_monotonic_time_s(c) < end) {
        r = poll_motion(t);
        if (r != CARTPOLE_OK)
            return r;
        if (t->braking)
            return CARTPOLE_REJECTED;
        if (t->still.confirmed)
            return CARTPOLE_OK;
        cartpole_sleep_until(c, fmin(end, cartpole_get_monotonic_time_s(c) + poll_s));
    }
    return cartpole_is_stop_requested(c) ? CARTPOLE_CANCELLED : CARTPOLE_TIMEOUT;
}

static cartpole_result_t drive(travel_t *t, double *last_write_done)
{
    cartpole_t *c = t->cartpole;
    t->phase = RUNNING;
    double start = cartpole_get_monotonic_time_s(c), next = start, last = start;
    double end = start + t->options.max_seconds;
    double max_gap = fmin(.050, t->config.watchdog_timeout_ms / 1000.);
    while (cartpole_get_monotonic_time_s(c) < end) {
        cartpole_result_t r = poll_motion(t);
        if (r != CARTPOLE_OK)
            return r;
        double now = cartpole_get_monotonic_time_s(c);
        if (now >= end)
            return CARTPOLE_TIMEOUT;
        if (t->braking)
            return CARTPOLE_OK;
        if (t->report->motion_attempted && now - last >= max_gap)
            return CARTPOLE_CONTROL_DEADLINE;
        if (now >= next) {
            double deadline = fmin(end, t->report->motion_attempted ? last + max_gap : now + max_gap);
            t->report->motion_attempted = 1;
            cartpole_send_result_t sent = cartpole_send_velocity(c, t->velocity, deadline, 1);
            *last_write_done = cartpole_get_monotonic_time_s(c);
            if (sent == CARTPOLE_SEND_COMPLETE && !t->has_complete_nonzero) {
                t->has_complete_nonzero = 1;
                t->first_write_done = *last_write_done;
            }
            if (sent != CARTPOLE_SEND_COMPLETE || *last_write_done >= deadline) {
                if (cartpole_is_stop_requested(c))
                    return CARTPOLE_CANCELLED;
                return stream_failure(c, CARTPOLE_CONTROL_DEADLINE);
            }
            last = now;
            next = *last_write_done + keepalive_s;
        }
        cartpole_sleep_until(c, fmin(end, cartpole_get_monotonic_time_s(c) + poll_s));
    }
    return cartpole_is_stop_requested(c) ? CARTPOLE_CANCELLED : CARTPOLE_TIMEOUT;
}

static void stop_and_observe(travel_t *t, double last_write_done)
{
    cartpole_t *c = t->cartpole;
    t->phase = STOPPING;
    t->still = (stillness_t){0};
    double next = cartpole_get_monotonic_time_s(c), end = next + settle_timeout_s;
    while (cartpole_get_monotonic_time_s(c) < end) {
        double now = cartpole_get_monotonic_time_s(c);
        if (now >= next) {
            cartpole_measurement_t m = cartpole_get_latest_measurement(c);
            cartpole_send_result_t sent = cartpole_send_velocity(c, 0., fmin(end, now + keepalive_s), 0);
            now = cartpole_get_monotonic_time_s(c);
            if (sent == CARTPOLE_SEND_COMPLETE && !t->report->zero_requested) {
                t->report->zero_position_mm = cart_state_is_fresh(c, m) ? position_um(m) / 1000. : NAN;
                t->report->zero_requested = 1;
                t->stop_not_before = fmax(now, last_write_done + t->inflight_s);
            } else if (sent != CARTPOLE_SEND_COMPLETE) {
                remember_failure(t, stream_failure(c, CARTPOLE_CONTROL_DEADLINE));
                t->still = (stillness_t){0};
            }
            next = now + keepalive_s;
        }
        cartpole_receive_result_t received = cartpole_receive(c);
        now = cartpole_get_monotonic_time_s(c);
        if (received == CARTPOLE_RECEIVE_BACKLOG) {
            remember_failure(t, CARTPOLE_CONTROL_DEADLINE);
            t->still = (stillness_t){0};
        } else if (received == CARTPOLE_RECEIVE_ERROR) {
            remember_failure(t, stream_failure(c, CARTPOLE_TRANSPORT_ERROR));
            t->still = (stillness_t){0};
        }
        remember_failure(t, check_session(c, cartpole_get_session_state(c)));
        if (cartpole_is_stop_requested(c))
            remember_failure(t, CARTPOLE_CANCELLED);
        if (now < end && received == CARTPOLE_RECEIVE_EMPTY && t->still.confirmed &&
            cart_state_is_fresh(c, cartpole_get_latest_measurement(c))) {
            t->report->stop_confirmed = 1;
            return;
        }
        /* 停车清理仍须有界等待，即使调用者已请求取消。 */
        cartpole_sleep_for_cleanup_until(c, fmin(end, now + poll_s));
    }
    remember_failure(t, CARTPOLE_TIMEOUT);
}

static cartpole_result_t run_travel(cartpole_t *c, void *context)
{
    (void)c;
    travel_t *t = context;
    cartpole_result_t r = make_plan(t);
    if (r != CARTPOLE_OK || t->options.plan_only)
        return r;

    r = wait_at_start(t);
    if (r != CARTPOLE_OK)
        return r;
    double last_write_done = 0;
    remember_failure(t, drive(t, &last_write_done));
    if (t->report->motion_attempted)
        stop_and_observe(t, last_write_done);
    else if (t->failure == CARTPOLE_OK)
        remember_failure(t, CARTPOLE_REJECTED); /* 尚无任何非零写入就触发制动。 */
    return t->failure;
}

cartpole_result_t cartpole_travel_velocity(cartpole_t *c, double velocity_mm_s,
                                         const cartpole_travel_options_t *options,
                                         cartpole_travel_report_t *report)
{
    if (!report)
        return CARTPOLE_INVALID_ARGUMENT;
    *report = (cartpole_travel_report_t){0};
    if (!c || !options || !isfinite(velocity_mm_s) || fabs(velocity_mm_s) > 1e9 ||
        !isfinite(options->reserve_mm) || options->reserve_mm < 0 ||
        !isfinite(options->inflight_ms) || options->inflight_ms < 0 ||
        !isfinite(options->start_tolerance_mm) || options->start_tolerance_mm < 0 ||
        options->start_tolerance_mm > 1. ||
        !isfinite(options->max_seconds) || options->max_seconds <= 0.)
        return CARTPOLE_INVALID_ARGUMENT;
    double velocity = round(velocity_mm_s);
    if (!velocity)
        return CARTPOLE_INVALID_ARGUMENT;
    travel_t t = {.cartpole = c, .options = *options, .report = report, .velocity = velocity,
                  .direction = velocity > 0 ? 1. : -1., .inflight_s = options->inflight_ms / 1000.};
    return cartpole_execute_operation(c, run_travel, observe, &t);
}
