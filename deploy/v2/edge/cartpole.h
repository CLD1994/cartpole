/* 单设备、调用方主动 pump 的会话。open/close 不发送停车命令。 */
#ifndef EDGE_CARTPOLE_H
#define EDGE_CARTPOLE_H
#include "proto.h"
#include <stdint.h>
typedef struct {
    int connected, mode_known;
    fw_mode_t mode;
    fw_reason_t reason;
    int io_error; /* 会话不可继续驱动的锁存位；包含I/O、协议或维护发送超时，非错误分类 */
    int fault_seen;
    unsigned valid_flags;
    int64_t x_um, theta_urad, vx_mm_s, vth_mrad_s; /* 单位由字段名标明 */
    double state_received_at_s; /* CLOCK_MONOTONIC主机接收时刻，非设备采样；未收到=-1 */
    uint64_t state_sequence;
    long state_count, unknown_count; /* 会话累计，消费者自行取差 */
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
double cartpole_snapshot_age(cartpole_snapshot_t snapshot);
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
cartpole_send_result_t cartpole_try_send_vel(double velocity_mm_s, double deadline_s);
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
double cartpole_now_s(void);
#endif
