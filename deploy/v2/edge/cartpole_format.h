#ifndef EDGE_CARTPOLE_FORMAT_H
#define EDGE_CARTPOLE_FORMAT_H
#include "cartpole.h"

/* 枚举的展示文本；未知值返回稳定的 UNKNOWN 文本，不进行设备访问。 */
const char *cartpole_mode_to_string(cartpole_mode_t);
const char *cartpole_reason_to_string(cartpole_reason_t);
const char *cartpole_result_to_string(cartpole_result_t);
#endif
