#ifndef EDGE_CARTPOLE_INTERNAL_H
#define EDGE_CARTPOLE_INTERNAL_H
#include "cartpole.h"
#include "proto.h"
/* STATE 测量有效位；两路独立，无效路不得参与推理或运动判定。 */
enum { CARTPOLE_MEASUREMENT_CART_VALID = 1u << 0, CARTPOLE_MEASUREMENT_POLE_VALID = 1u << 1 };
/* 以下接收计数和锁存仅覆盖本次open会话；open/close清除，MODE
 * READY/RESET不清除。 MODE或OK更新不会刷新最后STATE时间、计数或测量有效位。 */
typedef struct {
    /* 本进程串口已打开，不证明设备在线或供电正常。 */
    int is_serial_open;
    /* 本会话至少解析过一条MODE，不表示当前READY。 */
    int is_mode_received;
    fw_mode_t reported_mode;
    fw_reason_t reported_mode_reason;
    /* 禁止继续非零驱动；零速/停机cleanup仍可用，事务enum保存失败分类。 */
    int has_stream_failure_latch;
    /* 本会话曾收到MODE FAULT，后续READY不会清除。 */
    int has_received_fault_mode;
    unsigned measurement_validity_bits;
    /* 单位由字段名标明。 */
    int64_t x_um, theta_urad, vx_mm_s, vth_mrad_s;
    /* 主机CLOCK_MONOTONIC接收时刻，非设备采样；未收到=-1。 */
    double last_state_received_host_monotonic_s;
    /* 成功解析的STATE累计数量，非设备序号。 */
    uint64_t received_state_count;
    /* 无法解析的行累计数量，含接收器识别的超长行、含NUL等异常行。 */
    uint64_t received_unknown_line_count;
} cartpole_snapshot_t;
typedef enum {
    CARTPOLE_PUMP_ERROR = -1,
    CARTPOLE_PUMP_EMPTY = 0,
    CARTPOLE_PUMP_BACKLOG = 1
} cartpole_pump_result_t;
typedef enum {
    CARTPOLE_SEND_ERROR = -1,
    CARTPOLE_SEND_COMPLETE = 0,
    CARTPOLE_SEND_SKIPPED = 1,
    CARTPOLE_SEND_LATE = 2
} cartpole_send_result_t;

cartpole_snapshot_t cartpole_snapshot(void);
double cartpole_state_age_s(cartpole_snapshot_t);
cartpole_pump_result_t cartpole_pump(void);
cartpole_send_result_t cartpole_send_vel(double);
cartpole_send_result_t cartpole_send_vel_until(double, double);
cartpole_send_result_t cartpole_try_send_vel(double, double);
int cartpole_cached_value(const char *, int64_t *);
void cartpole_set_update_hook(void (*)(proto_msg_type_t, const cartpole_snapshot_t *, void *),
                              void *);
int cartpole_callback_enter(void);
int cartpole_callback_leave(void);
cartpole_result_t cartpole_stream_result(void);
int cartpole_loop_begin(void);
void cartpole_loop_end(void);
void cartpole_sleep_to(double);
#endif
