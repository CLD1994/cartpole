#include "cartpole_travel.h"
#include "fake_cartpole_device.h"
#include <assert.h>
#include <math.h>

static const cartpole_travel_options_t options = {
    .reserve_mm = 20, .inflight_ms = 10, .start_tolerance_mm = .5, .max_seconds = 1
};
typedef enum { BRAKE, PEAK, RECOVERY, RECOVERY_EXCEEDED, STALE, CANCEL, CANCEL_DURING_STOP, INVALID, POLE_INVALID, ERROR, STEP_LOSS_REPORT, FAULT_MODE, BACKLOG, OUTSIDE, UNKNOWN, LATE, NO_STOP, STOP_READ_ERROR } scenario_kind_t;
typedef struct {
    scenario_kind_t kind;
    int nonzero, zero, polls;
    cartpole_t *cartpole;
} scenario_t;

static cartpole_receive_result_t receive(fake_device *device)
{
    scenario_t *s = device->scenario_context;
    if (++s->polls % 20)
        return CARTPOLE_RECEIVE_EMPTY;
    if (s->nonzero && !s->zero && s->kind == STALE)
        return CARTPOLE_RECEIVE_EMPTY;
    if (s->nonzero && !s->zero && s->kind == BACKLOG)
        return CARTPOLE_RECEIVE_BACKLOG;
    device->measurement.measurement_id++;
    device->measurement.received_host_monotonic_s = device->now;
    device->measurement.cart_position_m = s->nonzero ? .405 : 0;
    if (!s->nonzero && (s->kind == RECOVERY || s->kind == RECOVERY_EXCEEDED)) {
        if (s->polls == 40)
            device->measurement.cart_position_m = s->kind == RECOVERY ? -.450500 : -.450501;
        else
            device->measurement.cart_position_m = -.450480;
    }
    device->measurement.cart_velocity_m_s =
        s->nonzero && (!s->zero || s->kind == NO_STOP) ? (s->kind == PEAK ? .4 : .6) : 0;
    if (s->nonzero && !s->zero && s->kind == INVALID) {
        device->measurement.cart_valid = 0;
        fake_device_emit(device, CARTPOLE_EVENT_MEASUREMENT);
        device->measurement.cart_valid = 1;
    }
    if (s->kind == POLE_INVALID)
        device->measurement.pole_valid = 0;
    if (s->nonzero && !s->zero && s->kind == ERROR)
        fake_device_emit(device, CARTPOLE_EVENT_FIRMWARE_ERROR);
    if (s->nonzero && !s->zero && s->kind == STEP_LOSS_REPORT)
        fake_device_emit(device, CARTPOLE_EVENT_STEP_LOSS);
    if (s->nonzero && !s->zero && s->kind == FAULT_MODE) {
        device->session.reported_mode = CARTPOLE_MODE_FAULT;
        device->session.reported_mode_reason = CARTPOLE_REASON_STEP_LOSS;
        device->session.has_received_fault_mode = 1;
        fake_device_emit(device, CARTPOLE_EVENT_SESSION);
    }
    if (s->nonzero && !s->zero && s->kind == UNKNOWN)
        fake_device_emit(device, CARTPOLE_EVENT_UNRECOGNIZED_LINE);
    if (s->nonzero && !s->zero && s->kind == OUTSIDE)
        device->measurement.cart_position_m = .441;
    fake_device_emit(device, CARTPOLE_EVENT_MEASUREMENT);
    if (s->zero && s->kind == STOP_READ_ERROR && device->now > 1.43)
        return CARTPOLE_RECEIVE_ERROR;
    return CARTPOLE_RECEIVE_EMPTY;
}

static cartpole_send_result_t send(fake_device *device, double velocity, double end, int once)
{
    scenario_t *s = device->scenario_context;
    (void)end;
    (void)once;
    if (velocity != 0) {
        assert(s->zero == 0);
        assert(device->now >= 1.2);
        s->nonzero++;
        if (s->kind == CANCEL)
            cartpole_request_stop(s->cartpole);
        if (s->kind == LATE)
            return CARTPOLE_SEND_LATE;
    } else {
        s->zero++;
        if (s->kind == CANCEL_DURING_STOP)
            cartpole_request_stop(s->cartpole);
    }
    return CARTPOLE_SEND_COMPLETE;
}

static void plan_only(void)
{
    fake_device device;
    cartpole_t *c = cartpole_create(fake_device_prepare(&device));
    assert(c);
    device.session.reported_mode = CARTPOLE_MODE_DISABLED;
    device.measurement.pole_valid = 0;
    cartpole_travel_options_t plan = options;
    plan.plan_only = 1;
    cartpole_travel_report_t report;
    assert(cartpole_travel_velocity(c, 600, &plan, &report) == CARTPOLE_OK);
    assert(report.plan_valid && report.target_mm == 420 && report.physical_mm == 450);
    assert(fabs(report.braking_budget_mm - 25.2) < .001);
    assert(fabs(report.watchdog_budget_mm - 85.2) < .001);
    assert(!report.stop_confirmed && device.sends == 0 && device.call_count == 3);
    for (int i = 0; i < device.call_count; ++i)
        assert(device.calls[i].operation == FAKE_REFRESH ||
               device.calls[i].operation == FAKE_QUERY || device.calls[i].operation == FAKE_WAIT);
    cartpole_destroy(c);
}

static void plan_rejects_unsafe_start(void)
{
    fake_device device;
    cartpole_t *c = cartpole_create(fake_device_prepare(&device));
    assert(c);
    cartpole_travel_options_t plan = options;
    plan.plan_only = 1;
    cartpole_travel_report_t report;
    device.measurement.cart_position_m = .4;
    assert(cartpole_travel_velocity(c, 600, &plan, &report) == CARTPOLE_REJECTED);
    assert(!report.plan_valid && !device.sends);
    device.measurement.cart_position_m = -.450501;
    assert(cartpole_travel_velocity(c, 600, &plan, &report) == CARTPOLE_REJECTED);
    device.measurement.cart_position_m = -.450499;
    assert(cartpole_travel_velocity(c, 600, &plan, &report) == CARTPOLE_OK);
    assert(report.plan_valid && !device.sends);
    device.configuration.has_legacy_watchdog_deceleration = 1;
    assert(cartpole_travel_velocity(c, 600, &plan, &report) == CARTPOLE_REJECTED);
    assert(!report.plan_valid && !device.sends);
    cartpole_destroy(c);
}

static void run(scenario_kind_t kind, cartpole_result_t expected, int confirmed)
{
    fake_device device;
    cartpole_t *c = cartpole_create(fake_device_prepare(&device));
    assert(c);
    scenario_t scenario = {.kind = kind, .cartpole = c};
    if (kind == RECOVERY || kind == RECOVERY_EXCEEDED)
        device.measurement.cart_position_m = -.450499;
    device.scenario_context = &scenario;
    device.on_receive = receive;
    device.on_send = send;

    cartpole_travel_report_t report;
    assert(cartpole_travel_velocity(c, 600, &options, &report) == expected);
    assert(report.plan_valid);
    if (kind == RECOVERY_EXCEEDED) {
        assert(!report.motion_attempted && !scenario.nonzero && !scenario.zero);
        cartpole_destroy(c);
        return;
    }
    assert(report.motion_attempted && scenario.nonzero > 0 && scenario.zero > 0);
    assert(report.zero_requested && report.stop_confirmed == confirmed);
    if (kind == BRAKE)
        assert(report.observed_target_95 && report.target_95_s >= 0 && report.target_95_s <= .0201);
    if (kind == PEAK)
        assert(!report.observed_target_95 && report.observed_peak && report.peak_velocity_mm_s == 400 &&
               report.peak_at_s >= 0 && report.peak_at_s <= .0201);
    assert(device.now < 4.5); /* motion and cleanup are both bounded */
    cartpole_destroy(c);
}

int main(void)
{
    plan_only();
    plan_rejects_unsafe_start();
    run(BRAKE, CARTPOLE_OK, 1);
    run(PEAK, CARTPOLE_OK, 1);
    run(RECOVERY, CARTPOLE_OK, 1);
    run(RECOVERY_EXCEEDED, CARTPOLE_REJECTED, 0);
    run(STALE, CARTPOLE_MEASUREMENT_UNAVAILABLE, 1);
    run(CANCEL, CARTPOLE_CANCELLED, 1);
    run(CANCEL_DURING_STOP, CARTPOLE_CANCELLED, 1);
    run(INVALID, CARTPOLE_MEASUREMENT_UNAVAILABLE, 1);
    run(POLE_INVALID, CARTPOLE_OK, 1);
    run(ERROR, CARTPOLE_REJECTED, 1);
    run(STEP_LOSS_REPORT, CARTPOLE_OK, 1);
    run(FAULT_MODE, CARTPOLE_FAULT, 1);
    run(BACKLOG, CARTPOLE_CONTROL_DEADLINE, 1);
    run(OUTSIDE, CARTPOLE_REJECTED, 1);
    run(UNKNOWN, CARTPOLE_PROTOCOL_ERROR, 1);
    run(LATE, CARTPOLE_CONTROL_DEADLINE, 1);
    run(NO_STOP, CARTPOLE_TIMEOUT, 0);
    run(STOP_READ_ERROR, CARTPOLE_TRANSPORT_ERROR, 0);
    return 0;
}
