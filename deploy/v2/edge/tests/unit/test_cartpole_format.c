#include "cartpole_format.h"
#include <assert.h>
#include <string.h>
int main(void)
{
    assert(strcmp(cartpole_mode_to_string(CARTPOLE_MODE_READY), "READY") == 0);
    assert(strcmp(cartpole_reason_to_string(CARTPOLE_REASON_MT6835), "MT6835") == 0);
    assert(strcmp(cartpole_result_to_string(CARTPOLE_CONFIGURATION_CHANGED),
                  "CONFIGURATION_CHANGED") == 0);
    assert(strcmp(cartpole_mode_to_string((cartpole_mode_t)999), "UNKNOWN") == 0);
    assert(strcmp(cartpole_reason_to_string((cartpole_reason_t)999), "UNKNOWN") == 0);
    assert(strcmp(cartpole_result_to_string((cartpole_result_t)999), "UNKNOWN_RESULT") == 0);
    return 0;
}
