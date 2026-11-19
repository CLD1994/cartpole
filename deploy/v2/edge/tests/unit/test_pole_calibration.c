#include "cartpole_pole_calibration.h"
#include "fake_cartpole_device.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    fake_device device;
    int frames, settings_reads, writes, queries;
    int invalid_at, moving_at, angle_change_at, mode_change_at;
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

static cartpole_result_t read_settings(void *context, cartpole_settings_t *out, double timeout)
{
    (void)timeout;
    fixture_t *f = fixture(context);
    f->settings_reads++;
    *out = (cartpole_settings_t){.values = {
        [CARTPOLE_SETTING_VELOCITY_LIMIT] = 900,
        [CARTPOLE_SETTING_ACCELERATION_LIMIT] = 10000,
        [CARTPOLE_SETTING_WATCHDOG_TIMEOUT] = 100,
        [CARTPOLE_SETTING_TRACK_MARGIN] = 10000,
        [CARTPOLE_SETTING_POLE_ZERO] = f->firmware_zero,
        [CARTPOLE_SETTING_POLE_INVERT] = f->original_inversion,
        [CARTPOLE_SETTING_CART_INVERT] = 0,
        [CARTPOLE_SETTING_STEP_LOSS_THRESHOLD] = 5000}};
    if (f->change_settings_after_sampling && f->settings_reads == 2)
        out->values[CARTPOLE_SETTING_POLE_ZERO]++;
    return CARTPOLE_OK;
}

static cartpole_result_t configure(void *context, cartpole_setting_t setting, long value,
                                   double timeout)
{
    (void)timeout;
    fixture_t *f = fixture(context);
    assert(setting == CARTPOLE_SETTING_POLE_ZERO);
    if (f->mode_change_before_write) {
        f->device.session.reported_mode = CARTPOLE_MODE_READY;
        fake_device_emit(&f->device, CARTPOLE_EVENT_SESSION);
        f->device.session.reported_mode = CARTPOLE_MODE_DISABLED;
        fake_device_emit(&f->device, CARTPOLE_EVENT_SESSION);
        /* 真实适配器在接收积压后、发送配置前检查取消。 */
        if (f->device.cancellation_probe(f->device.cancellation_context))
            return CARTPOLE_CANCELLED;
    }
    f->writes++;
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
        .pole_valid = f->frames != f->invalid_at,
        .pole_angle_rad = f->wrapping ? (f->frames % 2 ? 3.1415 : -3.1415)
                                     : (f->frames >= f->angle_change_at ? 1.01 : 1.0),
        .pole_angular_velocity_rad_s = f->frames == f->moving_at ? 0.021 : 0.001,
        .received_host_monotonic_s = d->now,
        .measurement_id = (unsigned)f->frames,
    };
    fake_device_emit(d, CARTPOLE_EVENT_MEASUREMENT);
    return CARTPOLE_RECEIVE_EMPTY;
}

static cartpole_result_t run(fixture_t *f, cartpole_mode_t mode, cartpole_pole_calibration_t *out)
{
    cartpole_device_t device = fake_device_prepare(&f->device);
    cartpole_device_ops_t ops = *device.ops;
    ops.query_mode = query;
    ops.read_settings = read_settings;
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
        {{.invalid_at = 200}, CARTPOLE_MODE_DISABLED, CARTPOLE_MEASUREMENT_UNAVAILABLE},
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

int main(void)
{
    stable_samples_preserve_inversion_and_verify_readback();
    wrapped_samples_do_not_average_to_upright();
    failures_before_write_leave_firmware_untouched();
    failures_after_write_are_marked_without_retries();
    mode_change_in_write_transaction_cancels_before_actual_send();
    puts("pole calibration tests passed");
}
