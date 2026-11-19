#include "cartpole_pole_calibration.h"
#include "fake_cartpole_device.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    fake_device device;
    int frames, settings_reads, writes, queries;
    int invalid_at, moving_at, angle_change_at, mode_change_at;
    int always_invalid, bad_after_write, invalid_after_write;
    int invalid_before_write;
    int backlog_at, backlog_after_write;
    double write_time;
    int change_settings_after_sampling, ignore_write, wrapping, mode_change_before_write;
    int sample_interval_ms;
    long original_zero;
    int original_inversion;
    long firmware_zero;
    cartpole_result_t write_result;
} fixture_t;

static fixture_t *fixture(void *d)
{
    return ((fake_device *)d)->scenario_context;
}

static cartpole_result_t query(void *context, double timeout)
{
    (void)timeout;
    fake_device *d = context;
    fixture(context)->queries++;
    fake_device_emit(d, CARTPOLE_EVENT_SESSION);
    return CARTPOLE_OK;
}

static cartpole_result_t read_pole_settings(void *context, cartpole_pole_settings_t *out, double timeout)
{
    (void)timeout;
    fixture_t *f = fixture(context);
    f->settings_reads++;
    *out = (cartpole_pole_settings_t){.zero_urad = f->firmware_zero,
                                      .inverted = f->original_inversion};
    if (f->change_settings_after_sampling && f->settings_reads == 2)
        out->zero_urad++;
    return CARTPOLE_OK;
}

static cartpole_result_t configure(void *context, cartpole_setting_t setting, long value,
                                   double timeout, cartpole_configure_observer_t observer,
                                   void *observer_context)
{
    (void)timeout;
    fixture_t *f = fixture(context);
    assert(setting == CARTPOLE_SETTING_POLE_ZERO);
    if (f->invalid_before_write) {
        f->device.measurement.pole_valid = 0;
        fake_device_emit(&f->device, CARTPOLE_EVENT_MEASUREMENT);
        f->device.measurement.pole_valid = 1;
        fake_device_emit(&f->device, CARTPOLE_EVENT_MEASUREMENT);
    }
    if (f->mode_change_before_write) {
        f->device.session.reported_mode = CARTPOLE_MODE_READY;
        fake_device_emit(&f->device, CARTPOLE_EVENT_SESSION);
        f->device.session.reported_mode = CARTPOLE_MODE_DISABLED;
        fake_device_emit(&f->device, CARTPOLE_EVENT_SESSION);
    }
    assert(observer);
    cartpole_result_t checked = observer(CARTPOLE_CONFIGURE_CHECK, observer_context);
    if (checked != CARTPOLE_OK)
        return checked;
    observer(CARTPOLE_CONFIGURE_ATTEMPTED, observer_context);
    f->writes++;
    f->write_time = f->device.now;
    if (!f->ignore_write)
        f->firmware_zero = value;
    return f->write_result;
}

static cartpole_receive_result_t receive(fake_device *d)
{
    fixture_t *f = fixture(d);
    if (d->now < 1. + (double)(f->frames + 1) * f->sample_interval_ms / 1000.)
        return CARTPOLE_RECEIVE_EMPTY;
    f->frames++;
    if (f->frames == f->mode_change_at) {
        d->session.reported_mode = CARTPOLE_MODE_READY;
        fake_device_emit(d, CARTPOLE_EVENT_SESSION);
        d->session.reported_mode = CARTPOLE_MODE_DISABLED;
        fake_device_emit(d, CARTPOLE_EVENT_SESSION);
    }
    d->measurement = (cartpole_measurement_t){
        .pole_valid = !f->always_invalid && f->frames != f->invalid_at &&
                      !(f->writes && f->invalid_after_write),
        .pole_angle_rad = f->writes ? (f->bad_after_write ? 0.123456 : 3.141592) :
                          f->wrapping ? (f->frames % 2 ? 3.1415 : -3.1415)
                                     : (f->frames >= f->angle_change_at ? 1.01 : 1.0),
        .pole_angular_velocity_rad_s = f->frames == f->moving_at ? 0.021 : 0.001,
        .received_host_monotonic_s = d->now,
        .measurement_id = (unsigned)f->frames,
    };
    fake_device_emit(d, CARTPOLE_EVENT_MEASUREMENT);
    if (f->frames == f->backlog_at || (f->writes && f->backlog_after_write))
        return CARTPOLE_RECEIVE_BACKLOG;
    return CARTPOLE_RECEIVE_EMPTY;
}

static cartpole_result_t run(fixture_t *f, cartpole_mode_t mode, cartpole_pole_calibration_t *out)
{
    cartpole_device_t device = fake_device_prepare(&f->device);
    cartpole_device_ops_t ops = *device.ops;
    ops.query_mode = query;
    ops.read_pole_settings = read_pole_settings;
    ops.configure = configure;
    device.ops = &ops;
    f->device.scenario_context = f;
    f->device.session.reported_mode = mode;
    f->device.on_receive = receive;
    f->firmware_zero = f->original_zero;
    if (!f->sample_interval_ms)
        f->sample_interval_ms = 2;
    if (!f->angle_change_at)
        f->angle_change_at = 1000000;
    cartpole_t *cartpole = cartpole_create(device);
    assert(cartpole);
    cartpole_result_t result = cartpole_calibrate_pole_down(cartpole, out);
    assert(f->device.sends == 0);
    assert(f->device.call_count == 0); /* 其他设备能力未被调用。 */
    cartpole_destroy(cartpole);
    return result;
}

static void stable_samples_preserve_inversion_and_verify_readback(void)
{
    fixture_t f = {.original_zero = 120, .original_inversion = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_OK);
    assert(out.old_zero_urad == 120 && out.new_zero_urad == 2141712);
    assert(out.pole_inverted == 1 && out.write_attempted == 1);
    assert(out.write_confirmed && out.detail == CARTPOLE_POLE_DETAIL_NONE);
    assert(out.verified_down_angle_rad > 3.14);
    assert(f.writes == 1 && f.firmware_zero == 2141712 && f.settings_reads == 3);
    assert(f.queries == 3);
}

static void wrapped_samples_do_not_average_to_upright(void)
{
    fixture_t f = {.original_zero = 1000, .wrapping = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_OK);
    assert(labs(out.new_zero_urad - 1000) <= 2);
    assert(out.pole_inverted == 0 && f.writes == 1);
}

static void failures_before_write_leave_firmware_untouched(void)
{
    struct {
        fixture_t f;
        cartpole_mode_t mode;
        cartpole_result_t result;
    } cases[] = {
        {.f = {.original_zero = 0}, .mode = CARTPOLE_MODE_READY, .result = CARTPOLE_REJECTED},
        {.f = {.original_zero = 0}, .mode = CARTPOLE_MODE_FAULT, .result = CARTPOLE_FAULT},
        {{.always_invalid = 1}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
        {{.moving_at = 200}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
        {{.angle_change_at = 200}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
        {{.sample_interval_ms = 6}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
        {{.sample_interval_ms = 110}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
        {{.mode_change_at = 7}, CARTPOLE_MODE_DISABLED, CARTPOLE_REJECTED},
        {{.change_settings_after_sampling = 1}, CARTPOLE_MODE_DISABLED, CARTPOLE_CONFIGURATION_CHANGED}
    };
    for (unsigned i = 0; i < sizeof cases / sizeof *cases; i++) {
        cartpole_pole_calibration_t out;
        assert(run(&cases[i].f, cases[i].mode, &out) == cases[i].result);
        assert(cases[i].f.writes == 0 && !out.write_attempted);
    }
}

static void isolated_invalid_frame_requires_another_full_window_before_write(void)
{
    fixture_t f = {.invalid_at = 200};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_OK);
    /* Fixture's invalid frame occurs at t=1.4; a completely new second is required. */
    assert(f.writes == 1 && f.write_time >= 2.4);
}

static void stored_offset_without_down_angle_is_not_reported_as_calibrated(void)
{
    fixture_t f = {.bad_after_write = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_REPLY_MISMATCH);
    assert(f.writes == 1 && out.write_attempted && out.write_confirmed);
    assert(out.detail == CARTPOLE_POLE_DETAIL_VERIFY_ANGLE);
}

static void invalid_telemetry_after_write_reports_storage_separately_from_verification(void)
{
    fixture_t f = {.invalid_after_write = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_MEASUREMENT_UNAVAILABLE);
    assert(f.writes == 1 && out.write_attempted && out.write_confirmed);
    assert(out.detail == CARTPOLE_POLE_DETAIL_VERIFY_MISSING);
}

static void failures_after_write_are_marked_without_retries(void)
{
    fixture_t no_readback = {.ignore_write = 1};
    fixture_t ambiguous_write = {.write_result = CARTPOLE_TIMEOUT};
    cartpole_pole_calibration_t out;
    assert(run(&no_readback, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_REPLY_MISMATCH);
    assert(no_readback.writes == 1 && out.write_attempted);
    assert(run(&ambiguous_write, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_TIMEOUT);
    assert(ambiguous_write.writes == 1 && out.write_attempted);
}

static void mode_change_in_write_transaction_cancels_before_actual_send(void)
{
    fixture_t f = {.mode_change_before_write = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_REJECTED);
    assert(f.writes == 0);
}

static void invalid_sample_in_write_transaction_prevents_any_write(void)
{
    fixture_t f = {.invalid_before_write = 1};
    cartpole_pole_calibration_t out;
    assert(run(&f, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_MEASUREMENT_UNAVAILABLE);
    assert(f.writes == 0 && !out.write_attempted);
    assert(out.detail == CARTPOLE_POLE_DETAIL_INVALID_POLE);
}

static void eligible_sample_in_backlogged_receive_does_not_validate(void)
{
    cartpole_pole_calibration_t out;
    fixture_t sampling = {.backlog_at = 501};
    assert(run(&sampling, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_MEASUREMENT_UNAVAILABLE);
    assert(!out.write_attempted && sampling.writes == 0);
    assert(out.detail == CARTPOLE_POLE_DETAIL_BACKLOG);
    fixture_t verification = {.backlog_after_write = 1};
    assert(run(&verification, CARTPOLE_MODE_DISABLED, &out) == CARTPOLE_MEASUREMENT_UNAVAILABLE);
    assert(out.write_confirmed && verification.writes == 1);
    assert(out.detail == CARTPOLE_POLE_DETAIL_BACKLOG);
}

static cartpole_result_t reentrant_configure_observer(cartpole_configure_stage_t stage,
                                                       void *context)
{
    assert(stage == CARTPOLE_CONFIGURE_CHECK);
    assert(cartpole_reset(context, .5) == CARTPOLE_CALLBACK_ERROR);
    return CARTPOLE_OK;
}

static void checked_configure_observer_cannot_mutate_the_session(void)
{
    fake_device f = {0};
    cartpole_t *c = cartpole_create(fake_device_prepare(&f));
    assert(c);
    assert(cartpole_configure_checked(c, CARTPOLE_SETTING_POLE_ZERO, 17, .5,
                                      reentrant_configure_observer, c) == CARTPOLE_CALLBACK_ERROR);
    assert(f.call_count == 0);
    cartpole_destroy(c);
}

int main(void)
{
    stable_samples_preserve_inversion_and_verify_readback();
    wrapped_samples_do_not_average_to_upright();
    isolated_invalid_frame_requires_another_full_window_before_write();
    stored_offset_without_down_angle_is_not_reported_as_calibrated();
    invalid_telemetry_after_write_reports_storage_separately_from_verification();
    failures_before_write_leave_firmware_untouched();
    failures_after_write_are_marked_without_retries();
    mode_change_in_write_transaction_cancels_before_actual_send();
    invalid_sample_in_write_transaction_prevents_any_write();
    eligible_sample_in_backlogged_receive_does_not_validate();
    checked_configure_observer_cannot_mutate_the_session();
    puts("pole calibration tests passed");
}
