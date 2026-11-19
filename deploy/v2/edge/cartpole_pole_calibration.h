#ifndef EDGE_CARTPOLE_POLE_CALIBRATION_H
#define EDGE_CARTPOLE_POLE_CALIBRATION_H
#include "cartpole.h"

typedef struct {
    int64_t old_zero_urad, new_zero_urad;
    int pole_inverted;
    /* 调用配置写入后置位，即使回执未确认。成功时其余字段有效。 */
    int write_attempted;
} cartpole_pole_calibration_t;

/* 操作者先让摆杆自然下垂且静止。仅在 DISABLED NONE 标定，不使能或移动小车。
 * 非 OK 且 write_attempted 时，查询固件以确认实际值，不自动回滚。 */
cartpole_result_t cartpole_calibrate_pole_down(cartpole_t *, cartpole_pole_calibration_t *);
#endif
