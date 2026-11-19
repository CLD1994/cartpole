#include <stddef.h>
#include <string.h>
#include "cartpole_format.h"

const char *cartpole_result_to_string(cartpole_result_t result)
{
    switch (result) {
    case CARTPOLE_OK:
        return CARTPOLE_TEXT_OK;
    case CARTPOLE_CANCELLED:
        return CARTPOLE_TEXT_CANCELLED;
    case CARTPOLE_TIMEOUT:
        return CARTPOLE_TEXT_TIMEOUT;
    case CARTPOLE_TRANSPORT_ERROR:
        return CARTPOLE_TEXT_TRANSPORT_ERROR;
    case CARTPOLE_PROTOCOL_ERROR:
        return CARTPOLE_TEXT_PROTOCOL_ERROR;
    case CARTPOLE_CALIBRATION_REFERENCE_ERROR:
        return CARTPOLE_TEXT_CALIBRATION_REFERENCE_ERROR;
    case CARTPOLE_STORAGE_ERROR:
        return CARTPOLE_TEXT_STORAGE_ERROR;
    case CARTPOLE_REJECTED:
        return CARTPOLE_TEXT_REJECTED;
    case CARTPOLE_BUSY:
        return CARTPOLE_TEXT_BUSY;
    case CARTPOLE_POWER_UNAVAILABLE:
        return CARTPOLE_TEXT_POWER_UNAVAILABLE;
    case CARTPOLE_FAULT:
        return CARTPOLE_TEXT_FAULT;
    case CARTPOLE_REPLY_MISMATCH:
        return CARTPOLE_TEXT_REPLY_MISMATCH;
    case CARTPOLE_INVALID_ARGUMENT:
        return CARTPOLE_TEXT_INVALID_ARGUMENT;
    case CARTPOLE_CALLBACK_ERROR:
        return CARTPOLE_TEXT_CALLBACK_ERROR;
    case CARTPOLE_MEASUREMENT_UNAVAILABLE:
        return CARTPOLE_TEXT_MEASUREMENT_UNAVAILABLE;
    case CARTPOLE_SESSION_UNAVAILABLE:
        return CARTPOLE_TEXT_SESSION_UNAVAILABLE;
    case CARTPOLE_CONTROL_DEADLINE:
        return CARTPOLE_TEXT_CONTROL_DEADLINE;
    case CARTPOLE_CONFIGURATION_CHANGED:
        return CARTPOLE_TEXT_CONFIGURATION_CHANGED;
    }
    return CARTPOLE_TEXT_UNKNOWN_RESULT;
}

const char *cartpole_mode_to_string(cartpole_mode_t mode)
{
    switch (mode) {
    case CARTPOLE_MODE_READY:
        return CARTPOLE_TEXT_READY;
    case CARTPOLE_MODE_FAULT:
        return CARTPOLE_TEXT_FAULT;
    case CARTPOLE_MODE_DISABLED:
        return CARTPOLE_TEXT_DISABLED;
    }
    return CARTPOLE_TEXT_UNKNOWN;
}
const char *cartpole_reason_to_string(cartpole_reason_t reason)
{
    switch (reason) {
    case CARTPOLE_REASON_NONE:
        return CARTPOLE_TEXT_NONE;
    case CARTPOLE_REASON_STEP_LOSS:
        return CARTPOLE_TEXT_STEP_LOSS;
    case CARTPOLE_REASON_AS5600:
        return CARTPOLE_TEXT_AS5600;
    case CARTPOLE_REASON_MT6835:
        return CARTPOLE_TEXT_MT6835;
    case CARTPOLE_REASON_POWER:
        return CARTPOLE_TEXT_POWER;
    }
    return CARTPOLE_TEXT_UNKNOWN;
}

const char *cartpole_setting_to_string(cartpole_setting_t setting)
{
    switch (setting) {
    case CARTPOLE_SETTING_VELOCITY_LIMIT:
        return CARTPOLE_TEXT_VEL_MAX;
    case CARTPOLE_SETTING_ACCELERATION_LIMIT:
        return CARTPOLE_TEXT_ACC_MAX;
    case CARTPOLE_SETTING_WATCHDOG_TIMEOUT:
        return CARTPOLE_TEXT_WDT_MS;
    case CARTPOLE_SETTING_TRACK_MARGIN:
        return CARTPOLE_TEXT_X_MARGIN_UM;
    case CARTPOLE_SETTING_POLE_ZERO:
        return CARTPOLE_TEXT_POLE_ZERO_URAD;
    case CARTPOLE_SETTING_POLE_INVERT:
        return CARTPOLE_TEXT_POLE_INV;
    case CARTPOLE_SETTING_CART_INVERT:
        return CARTPOLE_TEXT_X_INV;
    case CARTPOLE_SETTING_STEP_LOSS_THRESHOLD:
        return CARTPOLE_TEXT_STEP_LOSS_UM;
    case CARTPOLE_SETTING_COUNT:
        break;
    }
    return NULL;
}
int cartpole_parse_setting(const char *name, cartpole_setting_t *setting)
{
    if (!name || !setting)
        return -1;
    for (int i = CARTPOLE_SETTING_VELOCITY_LIMIT; i < CARTPOLE_SETTING_COUNT; i++) {
        if (!strcmp(name, cartpole_setting_to_string((cartpole_setting_t)i))) {
            *setting = (cartpole_setting_t)i;
            return 0;
        }
    }
    return -1;
}
