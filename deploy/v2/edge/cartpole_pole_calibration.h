#ifndef EDGE_CARTPOLE_POLE_CALIBRATION_H
#define EDGE_CARTPOLE_POLE_CALIBRATION_H
#include "cartpole.h"

typedef enum {
    CARTPOLE_POLE_DETAIL_NONE,
    CARTPOLE_POLE_DETAIL_INITIAL_MODE,
    CARTPOLE_POLE_DETAIL_INITIAL_SETTINGS,
    CARTPOLE_POLE_DETAIL_INVALID_POLE,
    CARTPOLE_POLE_DETAIL_ANGLE,
    CARTPOLE_POLE_DETAIL_MOTION,
    CARTPOLE_POLE_DETAIL_GAP,
    CARTPOLE_POLE_DETAIL_SPAN,
    CARTPOLE_POLE_DETAIL_COUNT,
    CARTPOLE_POLE_DETAIL_SAMPLING_STREAM,
    CARTPOLE_POLE_DETAIL_PREWRITE_SETTINGS,
    CARTPOLE_POLE_DETAIL_PREWRITE_MODE,
    CARTPOLE_POLE_DETAIL_WRITE,
    CARTPOLE_POLE_DETAIL_READBACK,
    CARTPOLE_POLE_DETAIL_POSTWRITE_MODE,
    CARTPOLE_POLE_DETAIL_VERIFY_MISSING,
    CARTPOLE_POLE_DETAIL_VERIFY_ANGLE,
    CARTPOLE_POLE_DETAIL_VERIFY_STREAM,
    CARTPOLE_POLE_DETAIL_BACKLOG,
} cartpole_pole_calibration_detail_t;

typedef struct {
    int64_t old_zero_urad, new_zero_urad;
    int pole_inverted;
    /* 尝试写入后，保留原值和候选值。收到写入回执且设置读回一致时置 write_confirmed。 */
    int write_attempted;
    int write_confirmed;
    double verified_down_angle_rad; /* 仅成功时有效。 */
    cartpole_pole_calibration_detail_t detail;
} cartpole_pole_calibration_t;

const char *cartpole_pole_calibration_detail_string(cartpole_pole_calibration_detail_t);

/* 操作者先让摆杆自然下垂且静止。仅在 DISABLED NONE 标定，不使能或移动小车。
 * 非 OK 且 write_attempted 时，查询固件以确认实际值，不自动回滚。 */
cartpole_result_t cartpole_calibrate_pole_down(cartpole_t *, cartpole_pole_calibration_t *);
#endif
