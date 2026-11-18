#include "cartpole_format.h"

const char *cartpole_result_to_string(cartpole_result_t result)
{
    switch (result) {
    case CARTPOLE_OK:
        return "OK";
    case CARTPOLE_CANCELLED:
        return "CANCELLED";
    case CARTPOLE_TIMEOUT:
        return "TIMEOUT";
    case CARTPOLE_TRANSPORT_ERROR:
        return "TRANSPORT_ERROR";
    case CARTPOLE_PROTOCOL_ERROR:
        return "PROTOCOL_ERROR";
    case CARTPOLE_REJECTED:
        return "REJECTED";
    case CARTPOLE_FAULT:
        return "FAULT";
    case CARTPOLE_REPLY_MISMATCH:
        return "REPLY_MISMATCH";
    case CARTPOLE_INVALID_ARGUMENT:
        return "INVALID_ARGUMENT";
    case CARTPOLE_CALLBACK_ERROR:
        return "CALLBACK_ERROR";
    case CARTPOLE_MEASUREMENT_UNAVAILABLE:
        return "MEASUREMENT_UNAVAILABLE";
    case CARTPOLE_SESSION_UNAVAILABLE:
        return "SESSION_UNAVAILABLE";
    case CARTPOLE_CONTROL_DEADLINE:
        return "CONTROL_DEADLINE";
    case CARTPOLE_CONFIGURATION_CHANGED:
        return "CONFIGURATION_CHANGED";
    }
    return "UNKNOWN_RESULT";
}

const char *cartpole_mode_to_string(cartpole_mode_t mode)
{
    switch (mode) {
    case CARTPOLE_MODE_READY:
        return "READY";
    case CARTPOLE_MODE_FAULT:
        return "FAULT";
    case CARTPOLE_MODE_DISABLED:
        return "DISABLED";
    }
    return "UNKNOWN";
}
const char *cartpole_reason_to_string(cartpole_reason_t reason)
{
    switch (reason) {
    case CARTPOLE_REASON_NONE:
        return "NONE";
    case CARTPOLE_REASON_STEP_LOSS:
        return "STEP_LOSS";
    case CARTPOLE_REASON_AS5600:
        return "AS5600";
    case CARTPOLE_REASON_MT6835:
        return "MT6835";
    }
    return "UNKNOWN";
}
