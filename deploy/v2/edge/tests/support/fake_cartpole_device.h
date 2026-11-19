#ifndef TEST_FAKE_CARTPOLE_DEVICE_H
#define TEST_FAKE_CARTPOLE_DEVICE_H
#include "cartpole_device.h"
enum fake_operation { FAKE_QUERY, FAKE_WAIT, FAKE_ENABLE, FAKE_RESET, FAKE_REFRESH,
                      FAKE_MARK, FAKE_CONFIGURE, FAKE_STOP, FAKE_READ_SETTINGS,
                      FAKE_DISABLE, FAKE_CONFIRM_LEFT, FAKE_READ_CALIBRATION_STATUS };
typedef struct fake_device {
    double now;
    cartpole_clock_t clock;
    cartpole_measurement_t measurement;
    cartpole_session_t session;
    cartpole_configuration_t configuration;
    cartpole_calibration_status_t calibration_status;
    int64_t confirmed_left_position_um;
    cartpole_settings_t settings;
    cartpole_event_handler_t observer;
    void *observer_context;
    cartpole_result_t error;
    int reads, sends, attaches, detaches;
    void *scenario_context;
    cartpole_cancellation_probe_t cancellation_probe;
    void *cancellation_context;
    cartpole_result_t attach_result, transaction_result;
    cartpole_stop_result_t stop_result;
    cartpole_receive_result_t (*on_receive)(struct fake_device *);
    cartpole_send_result_t (*on_send)(struct fake_device *, double, double, int);
    struct { int operation; double timeout; long value; int argument; } calls[64];
    int call_count;
} fake_device;
cartpole_device_t fake_device_prepare(fake_device *);
void fake_device_emit(fake_device *, cartpole_event_kind_t);
#endif
