#include "cartpole_format.h"
#include <assert.h>
#include <string.h>
int main(void)
{
    assert(strcmp(cartpole_mode_to_string(CARTPOLE_MODE_READY), CARTPOLE_TEXT_READY) == 0);
    assert(strcmp(cartpole_reason_to_string(CARTPOLE_REASON_MT6835), CARTPOLE_TEXT_MT6835) == 0);
    assert(strcmp(cartpole_result_to_string(CARTPOLE_CONFIGURATION_CHANGED),
                  CARTPOLE_TEXT_CONFIGURATION_CHANGED) == 0);
    assert(strcmp(cartpole_mode_to_string((cartpole_mode_t)999), CARTPOLE_TEXT_UNKNOWN) == 0);
    assert(strcmp(cartpole_reason_to_string((cartpole_reason_t)999), CARTPOLE_TEXT_UNKNOWN) == 0);
    assert(strcmp(cartpole_result_to_string((cartpole_result_t)999), CARTPOLE_TEXT_UNKNOWN_RESULT) == 0);
    cartpole_setting_t setting = CARTPOLE_SETTING_POLE_ZERO;
    assert(strcmp(cartpole_setting_to_string(CARTPOLE_SETTING_ACCELERATION_LIMIT), CARTPOLE_TEXT_ACC_MAX) == 0);
    assert(cartpole_parse_setting(CARTPOLE_TEXT_VEL_MAX, &setting) == 0);
    assert(setting == CARTPOLE_SETTING_VELOCITY_LIMIT);
    assert(cartpole_parse_setting("invalid", &setting) == -1);
    assert(setting == CARTPOLE_SETTING_VELOCITY_LIMIT);
    assert(cartpole_setting_to_string((cartpole_setting_t)999) == NULL);
    assert(cartpole_parse_setting(NULL, &setting) == -1);
    assert(cartpole_parse_setting(CARTPOLE_TEXT_ACC_MAX, NULL) == -1);
    return 0;
}
