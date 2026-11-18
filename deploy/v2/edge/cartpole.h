/* 单设备、调用方主动 pump 的会话。open/close 不发送停车命令。 */
#ifndef EDGE_CARTPOLE_H
#define EDGE_CARTPOLE_H
#include "proto.h"
#include <stdint.h>
/* STATE 测量有效位；两路独立，无效路不得参与推理或运动判定。 */
enum { CARTPOLE_MEASUREMENT_CART_VALID = 1u << 0, CARTPOLE_MEASUREMENT_POLE_VALID = 1u << 1 };
/* 以下接收计数和锁存仅覆盖本次open会话；open/close清除，MODE READY/RESET不清除。
 * MODE或OK更新不会刷新最后STATE时间、计数或测量有效位。 */
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
typedef enum {
    CARTPOLE_STOP_UNCONFIRMED = -1,
    CARTPOLE_STOP_DISABLED = 0,
    CARTPOLE_STOP_FAULT = 1
} cartpole_stop_result_t;
typedef enum {
    CARTPOLE_OK = 0,
    CARTPOLE_CANCELLED,
    CARTPOLE_TIMEOUT,
    CARTPOLE_TRANSPORT_ERROR,
    CARTPOLE_PROTOCOL_ERROR,
    CARTPOLE_REJECTED,
    CARTPOLE_FAULT,
    CARTPOLE_REPLY_MISMATCH,
    CARTPOLE_INVALID_ARGUMENT
} cartpole_result_t;
/* 稳定分类名，不读取会话状态、不分配内存。 */
const char *cartpole_result_name(cartpole_result_t result);
typedef enum { CARTPOLE_LEFT, CARTPOLE_RIGHT } cartpole_endpoint_t;
/* 单线程使用，按值复制，后续pump不会改变已取得的快照。close清为未连接/未知。 */
cartpole_snapshot_t cartpole_snapshot(void);
/* 从最后STATE主机接收至调用此函数的秒数；未收到STATE返回-1。 */
double cartpole_state_age_s(cartpole_snapshot_t snapshot);
/* 每条 STATE/MODE/OK 后同步通知，不能重入pump或发送。需保留值时自行复制。 */
typedef void (*cartpole_observer_t)(proto_msg_type_t event, const cartpole_snapshot_t *,
                                    void *context);
void cartpole_set_observer(cartpole_observer_t observer, void *context);
void cartpole_request_stop(void);
int cartpole_stop_requested(void);
int cartpole_open(const char *port, int baud);
void cartpole_close(void);
cartpole_pump_result_t cartpole_pump(void); /* 最多4096字节 */
cartpole_send_result_t cartpole_send_vel(double velocity_mm_s);
/* 单次非阻塞写；SKIPPED未写，LATE已完整写但过期须停止；ERROR可能半帧。
 * COMPLETE仅证明内核接受，并不证明设备收讫。 */
cartpole_send_result_t cartpole_try_send_vel(double velocity_mm_s,
                                             double deadline_host_monotonic_s);
/* 事务timeout涵盖清积压、发送、等回执，不按阶段重新计时；TIMEOUT表示执行结果未确认，不得自动重试事务。
 * sync_mode只证明新MODE，不保证READY；wait_state只证明新STATE，不保证有效。 */
cartpole_result_t cartpole_sync_mode(double timeout_s);
cartpole_result_t cartpole_wait_state(double timeout_s);
cartpole_result_t cartpole_enable(double timeout_s);
cartpole_result_t cartpole_reset(double timeout_s);
cartpole_result_t cartpole_mark(cartpole_endpoint_t endpoint, double timeout_s);
cartpole_result_t cartpole_configure(const char *name, long value, double timeout_s);
cartpole_result_t cartpole_refresh_metadata(double timeout_s);
int cartpole_cached_value(const char *name, int64_t *value); /* 无I/O，不存在返回-1 */
cartpole_stop_result_t cartpole_stop_and_confirm(double timeout_s);
/* 当前主机CLOCK_MONOTONIC秒，不是设备时间或墙上时钟。 */
double cartpole_host_monotonic_s(void);
#endif
