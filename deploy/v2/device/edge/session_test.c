#include "session.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

struct wire { char lines[256][64]; int count; bool fail; };
static bool send_line(void *context, const char *line) {
    struct wire *w = context;
    assert(w->count < 256);
    snprintf(w->lines[w->count++], 64, "%s", line);
    return !w->fail;
}
static device_session *create(struct wire *w) {
    device_limits limits = {1250, 10000, 10000, 100, 5000, 0.1, 0.3, 1.0, 1.0};
    return device_session_create(limits, send_line, w);
}
static void prepare(device_session *s) {
    const char *lines[] = {
        "MODE READY NONE", "OK CAL_STATE 2", "OK CAL_LENGTH_UM 853325",
        "OK CAL_SAVED 1", "OK CAL_STORAGE 0", "OK VEL_MAX 1250",
        "OK ACC_MAX 10000", "OK WDT_MS 100", "OK WDT_DEC 5000",
        "OK X_MARGIN_UM 10000", "VEL_VALID 1 1", "STATE 0 0 0 0"
    };
    for (size_t i=0; i<sizeof(lines)/sizeof(lines[0]); ++i)
        assert(device_session_feed(s, lines[i], 1.0));
}
static void test_create_is_inert(void) {
    struct wire w = {0}; device_session *s = create(&w);
    assert(s && w.count == 0); device_session_destroy(s);
}
static void test_refresh_only_queries(void) {
    struct wire w = {0}; device_session *s = create(&w);
    assert(device_session_refresh(s, 0));
    assert(w.count == 3 && !strcmp(w.lines[0], "QUERY\n") &&
        !strcmp(w.lines[1], "CALIB?\n") && !strcmp(w.lines[2], "CONFIG?\n"));
    device_session_destroy(s);
}
static void test_start_requires_complete_metadata(void) {
    struct wire w = {0}; device_session *s = create(&w);
    assert(!device_session_start(s, 0)); assert(w.count == 0);
    device_session_destroy(s);
}
static void test_start_is_not_enable(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(w.count == 0);
    device_session_destroy(s);
}
static void test_action_maps_profile_velocity(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_action(s, 1, 1.01));
    assert(!strcmp(w.lines[0], "VEL 1250\n")); device_session_destroy(s);
}
static void test_action_rounds_half_units_away_from_zero(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    const double actions[] = {0.0004, -0.0004, 0.002, -0.002, 0.00039, -0.00039};
    const char *expected[] = {"VEL 1\n", "VEL -1\n", "VEL 3\n", "VEL -3\n", "VEL 0\n", "VEL 0\n"};
    for (size_t i = 0; i < sizeof(actions) / sizeof(actions[0]); ++i) {
        assert(device_session_action(s, actions[i], 1));
        assert(!strcmp(w.lines[i], expected[i]));
    }
    device_session_destroy(s);
}
static void test_action_outward_at_boundary_is_zero(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(device_session_feed(s, "STATE 417000 0 0 0", 1)); assert(device_session_action(s, 1, 1));
    assert(!strcmp(w.lines[0], "VEL 0\n")); device_session_destroy(s);
}
static void test_action_inward_at_boundary_is_allowed(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(device_session_feed(s, "STATE 417000 0 0 0", 1)); assert(device_session_action(s, -1, 1));
    assert(!strcmp(w.lines[0], "VEL -1250\n")); device_session_destroy(s);
}
static void test_nonfinite_action_latches_failure(void) {
    const double actions[] = {NAN, INFINITY, -INFINITY};
    for (size_t i = 0; i < sizeof(actions) / sizeof(actions[0]); ++i) {
        struct wire w = {0}; device_session *s = create(&w); prepare(s);
        assert(device_session_start(s, 1)); assert(!device_session_action(s, actions[i], 1));
        assert(device_session_snapshot(s).error == DEVICE_INVALID_ACTION);
        assert(!strcmp(w.lines[0], "VEL 0\n")); device_session_destroy(s);
    }
}
static void test_invalid_measurement_clears_cache(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "VEL_VALID 0 1", 1));
    assert(!device_session_snapshot(s).state.available); device_session_destroy(s);
}
static void test_validity_recovery_requires_new_state(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "VEL_VALID 0 1", 1));
    assert(device_session_feed(s, "VEL_VALID 1 1", 1));
    assert(!device_session_start(s, 1)); device_session_destroy(s);
}
static void test_active_failure_cannot_resume(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(!device_session_feed(s, "VEL_VALID 0 1", 1));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1));
    assert(!device_session_start(s, 1)); assert(w.count == 1); device_session_destroy(s);
}
static void test_state_timeout_stops_once(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); device_session_tick(s, 1.101);
    device_session_tick(s, 2); assert(w.count <= 2);
    assert(device_session_snapshot(s).error == DEVICE_STALE); device_session_destroy(s);
}
static void test_heartbeat_timeout_rejects_recent_state(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "STATE 0 0 0 0", 1.29));
    assert(!device_session_start(s, 1.301)); device_session_destroy(s);
}
static void test_metadata_must_be_recent(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "VEL_VALID 1 1", 2.01));
    assert(device_session_feed(s, "STATE 0 0 0 0", 2.01));
    assert(!device_session_start(s, 2.01)); device_session_destroy(s);
}
static void test_fault_mode_cannot_be_recovered_by_ready(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(!device_session_feed(s, "MODE FAULT STEP_LOSS", 1));
    assert(!device_session_feed(s, "MODE READY NONE", 1));
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING); device_session_destroy(s);
}
static void test_malformed_state_invalidates(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "STATE 0 bad 0 0", 1));
    assert(!device_session_snapshot(s).state.available); device_session_destroy(s);
}
static void test_config_mismatch_blocks_start(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "OK ACC_MAX 5000", 1));
    assert(!device_session_start(s, 1)); device_session_destroy(s);
}
static void test_left_reference_required(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "OK CAL_STATE 3", 1));
    assert(!device_session_start(s, 1)); device_session_destroy(s);
}
static void test_send_failure_is_bounded(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); w.fail = true;
    assert(!device_session_action(s, 1, 1)); assert(w.count == 2);
    device_session_tick(s, 1.01); assert(w.count == 2); device_session_destroy(s);
}
static void test_stop_waits_for_fresh_zero_then_disabled(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1.01));
    assert(w.count == 1); assert(device_session_snapshot(s).phase == DEVICE_STOPPING);
    assert(device_session_feed(s, "STATE 0 0 0 0", 1.02));
    assert(!strcmp(w.lines[1], "DISABLE\n"));
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING);
    assert(device_session_feed(s, "MODE DISABLED NONE", 1.03));
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(device_session_snapshot(s).disabled_confirmed); device_session_destroy(s);
}
static void test_stop_fault_is_unconfirmed(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1)); assert(!device_session_feed(s, "MODE FAULT STEP_LOSS", 1));
    assert(device_session_snapshot(s).error == DEVICE_STOP_UNCONFIRMED); device_session_destroy(s);
}
static void test_stop_deadline_is_bounded(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1)); device_session_tick(s, 2);
    assert(device_session_snapshot(s).error == DEVICE_STOP_UNCONFIRMED); device_session_destroy(s);
}
static void test_clock_regression_rejects_action(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(!device_session_action(s, 1, 0.9));
    assert(device_session_snapshot(s).error == DEVICE_CLOCK_ERROR); device_session_destroy(s);
}
static void test_active_metadata_requires_periodic_query(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    for (int i=1; i<=20; ++i) {
        double t = 1.0 + 0.05 * i;
        assert(device_session_feed(s, "VEL_VALID 1 1", t));
        assert(device_session_feed(s, "STATE 0 0 0 0", t));
    }
    device_session_tick(s, 2.01);
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING); device_session_destroy(s);
}
static void test_active_configuration_change_latches(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(!device_session_feed(s, "OK CAL_LENGTH_UM 853300", 1));
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING); device_session_destroy(s);
}
static void test_start_at_boundary_is_rejected(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_feed(s, "STATE 417000 0 0 0", 1));
    assert(!device_session_start(s, 1)); assert(w.count == 0); device_session_destroy(s);
}
static void test_out_of_range_state_is_rejected(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "STATE 2147483648 0 0 0", 1));
    assert(!device_session_snapshot(s).state.available); device_session_destroy(s);
}
static void test_failed_control_can_only_finish_stopping(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(!device_session_feed(s, "VEL_VALID 0 1", 1.01));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1.02));
    assert(!device_session_feed(s, "STATE 1 0 0 0", 1.03));
    assert(!device_session_feed(s, "MODE DISABLED NONE", 1.04));
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(device_session_snapshot(s).disabled_confirmed);
    assert(device_session_snapshot(s).error == DEVICE_INVALID_MEASUREMENT);
    assert(!device_session_start(s, 1.04)); device_session_destroy(s);
}
static void test_zero_margin_rejected(void) {
    struct wire w = {0};
    device_limits limits = {1250, 10000, 0, 100, 5000, 0.1, 0.3, 1.0, 1.0};
    assert(device_session_create(limits, send_line, &w) == NULL);
    assert(w.count == 0);
}
static void test_negative_margin_rejected(void) {
    struct wire w = {0};
    device_limits limits = {1250, 10000, -1, 100, 5000, 0.1, 0.3, 1.0, 1.0};
    assert(device_session_create(limits, send_line, &w) == NULL);
    assert(w.count == 0);
}
static void test_signed_validity_literal_rejected(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "VEL_VALID +1 1", 1));
    assert(!device_session_snapshot(s).validity); device_session_destroy(s);
}
static void test_padded_validity_literal_rejected(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "VEL_VALID 1 01", 1));
    assert(!device_session_snapshot(s).validity); device_session_destroy(s);
}
static void test_backlogged_validity_cannot_hide_expiry(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1.4));
    assert(!device_session_feed(s, "STATE 0 0 0 0", 1.4));
    assert(!device_session_action(s, 1, 1.4));
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING);
    assert(device_session_snapshot(s).error == DEVICE_STALE); device_session_destroy(s);
}
static void test_late_disabled_cannot_confirm_expired_stop(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1.01));
    assert(device_session_feed(s, "STATE 0 0 0 0", 1.02));
    assert(!device_session_feed(s, "MODE DISABLED NONE", 2.11));
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(!device_session_snapshot(s).disabled_confirmed);
    assert(device_session_snapshot(s).error == DEVICE_STOP_UNCONFIRMED); device_session_destroy(s);
}
static void test_application_abort_is_inert_while_observing(void) {
    struct wire w = {0}; device_session *s = create(&w);
    device_session_abort(s, 1);
    assert(w.count == 0);
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(device_session_snapshot(s).error == DEVICE_APPLICATION_ERROR);
    device_session_destroy(s);
}
static void test_application_abort_stops_without_extending_deadline(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); device_session_abort(s, 1.01);
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING);
    assert(device_session_snapshot(s).error == DEVICE_APPLICATION_ERROR);
    assert(!strcmp(w.lines[0], "VEL 0\n"));
    device_session_abort(s, 1.5); device_session_tick(s, 2.02);
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(device_session_snapshot(s).error == DEVICE_APPLICATION_ERROR);
    device_session_destroy(s);
}
static void test_missing_calibration_value_revokes_readback(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "OK CAL_STATE", 1));
    assert(device_session_feed(s, "VEL_VALID 1 1", 1));
    assert(device_session_feed(s, "STATE 0 0 0 0", 1));
    assert(!device_session_start(s, 1)); device_session_destroy(s);
}
static void test_missing_config_value_revokes_readback(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(!device_session_feed(s, "OK ACC_MAX", 1));
    assert(device_session_feed(s, "VEL_VALID 1 1", 1));
    assert(device_session_feed(s, "STATE 0 0 0 0", 1));
    assert(!device_session_start(s, 1)); device_session_destroy(s);
}
static void test_missing_active_config_value_latches(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1));
    assert(!device_session_feed(s, "OK ACC_MAX", 1));
    assert(device_session_snapshot(s).phase == DEVICE_STOPPING);
    assert(device_session_snapshot(s).error == DEVICE_INVALID_LINE); device_session_destroy(s);
}
static void test_stopping_fault_survives_disabled_ack(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1));
    assert(!device_session_feed(s, "MODE FAULT STEP_LOSS", 1));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1.02));
    assert(!device_session_feed(s, "STATE 0 0 0 0", 1.03));
    assert(!device_session_feed(s, "MODE DISABLED NONE", 1.04));
    assert(device_session_snapshot(s).disabled_confirmed);
    assert(device_session_snapshot(s).error == DEVICE_STOP_UNCONFIRMED); device_session_destroy(s);
}
static void test_stopping_error_survives_disabled_ack(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1));
    assert(!device_session_feed(s, "ERR DISABLE", 1));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1.02));
    assert(!device_session_feed(s, "STATE 0 0 0 0", 1.03));
    assert(!device_session_feed(s, "MODE DISABLED NONE", 1.04));
    assert(device_session_snapshot(s).disabled_confirmed);
    assert(device_session_snapshot(s).error == DEVICE_FIRMWARE_FAULT); device_session_destroy(s);
}
static void test_stopping_invalid_measurement_survives_disabled_ack(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(device_session_stop(s, 1));
    assert(!device_session_feed(s, "VEL_VALID 0 1", 1));
    assert(!device_session_feed(s, "VEL_VALID 1 1", 1.04));
    assert(!device_session_feed(s, "STATE 0 0 0 0", 1.04));
    device_session_tick(s, 1.08);
    assert(!device_session_feed(s, "MODE DISABLED NONE", 1.08));
    assert(device_session_snapshot(s).disabled_confirmed);
    assert(device_session_snapshot(s).error == DEVICE_INVALID_MEASUREMENT);
    device_session_destroy(s);
}
static void test_invalid_measurement_preserves_existing_error_and_deadline(void) {
    struct wire w = {0}; device_session *s = create(&w); prepare(s);
    assert(device_session_start(s, 1)); assert(!device_session_action(s, 2, 1));
    assert(!device_session_feed(s, "VEL_VALID 0 1", 1.5));
    device_session_tick(s, 2.01);
    assert(device_session_snapshot(s).phase == DEVICE_FINISHED);
    assert(device_session_snapshot(s).error == DEVICE_INVALID_ACTION);
    assert(!device_session_snapshot(s).disabled_confirmed);
    device_session_destroy(s);
}
static void test_limits_match_firmware_ranges(void) {
    struct wire w = {0};
    device_limits l = {1250, 10000, 10000, 100, 5000, .1, .3, 1, 1};
    int *fields[] = {&l.speed_mm_s, &l.acceleration_mm_s2, &l.soft_margin_um,
                    &l.watchdog_ms, &l.watchdog_decel_mm_s2};
    const int lows[] = {1, 100, 1000, 10, 100};
    const int highs[] = {2000, 30000, 50000, 1000, 30000};
    for (int i = 0; i < 5; ++i) {
        int saved = *fields[i];
        *fields[i] = lows[i] - 1; assert(!device_session_create(l, send_line, &w));
        *fields[i] = highs[i] + 1; assert(!device_session_create(l, send_line, &w));
        *fields[i] = lows[i]; device_session *s = device_session_create(l, send_line, &w);
        assert(s); device_session_destroy(s);
        *fields[i] = highs[i]; s = device_session_create(l, send_line, &w);
        assert(s); device_session_destroy(s); *fields[i] = saved;
    }
    assert(w.count == 0);
}
int main(void) {
    test_stopping_invalid_measurement_survives_disabled_ack();
    test_limits_match_firmware_ranges();
    test_invalid_measurement_preserves_existing_error_and_deadline();
    test_create_is_inert(); test_refresh_only_queries(); test_start_requires_complete_metadata();
    test_start_is_not_enable(); test_action_maps_profile_velocity();
    test_action_rounds_half_units_away_from_zero();
    test_action_outward_at_boundary_is_zero(); test_action_inward_at_boundary_is_allowed();
    test_nonfinite_action_latches_failure(); test_invalid_measurement_clears_cache();
    test_validity_recovery_requires_new_state(); test_active_failure_cannot_resume();
    test_state_timeout_stops_once(); test_heartbeat_timeout_rejects_recent_state();
    test_metadata_must_be_recent(); test_fault_mode_cannot_be_recovered_by_ready();
    test_malformed_state_invalidates(); test_config_mismatch_blocks_start();
    test_left_reference_required(); test_send_failure_is_bounded();
    test_stop_waits_for_fresh_zero_then_disabled(); test_stop_fault_is_unconfirmed();
    test_stop_deadline_is_bounded(); test_clock_regression_rejects_action();
    test_active_metadata_requires_periodic_query(); test_active_configuration_change_latches();
    test_start_at_boundary_is_rejected(); test_out_of_range_state_is_rejected();
    test_failed_control_can_only_finish_stopping();
    test_zero_margin_rejected(); test_negative_margin_rejected();
    test_signed_validity_literal_rejected(); test_padded_validity_literal_rejected();
    test_backlogged_validity_cannot_hide_expiry(); test_late_disabled_cannot_confirm_expired_stop();
    test_application_abort_is_inert_while_observing();
    test_application_abort_stops_without_extending_deadline();
    test_missing_calibration_value_revokes_readback();
    test_missing_config_value_revokes_readback(); test_missing_active_config_value_latches();
    test_stopping_fault_survives_disabled_ack(); test_stopping_error_survives_disabled_ack();
    puts("device session: 45 public-contract unit cases passed");
}
