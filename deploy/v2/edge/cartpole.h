/* 倒立摆业务会话。只读 getter 返回最近已接收状态的副本，不发起 IO。
 * 设备能力与调度环境由装配层注入；销毁会话不等于停车。 */
#ifndef EDGE_CARTPOLE_H
#define EDGE_CARTPOLE_H
#include <stdint.h>
#include "cartpole_clock.h"
typedef enum { CARTPOLE_MODE_DISABLED, CARTPOLE_MODE_READY, CARTPOLE_MODE_FAULT } cartpole_mode_t;
typedef enum {
    CARTPOLE_REASON_NONE,
    CARTPOLE_REASON_STEP_LOSS,
    CARTPOLE_REASON_AS5600,
    CARTPOLE_REASON_MT6835,
    CARTPOLE_REASON_POWER
} cartpole_reason_t;
typedef enum {
    CARTPOLE_OK = 0,
    CARTPOLE_CANCELLED,
    CARTPOLE_TIMEOUT,
    CARTPOLE_TRANSPORT_ERROR,
    CARTPOLE_PROTOCOL_ERROR,
    CARTPOLE_REJECTED,
    CARTPOLE_STORAGE_ERROR,
    CARTPOLE_CALIBRATION_REFERENCE_ERROR,
    CARTPOLE_FAULT,
    CARTPOLE_REPLY_MISMATCH,
    CARTPOLE_INVALID_ARGUMENT,
    CARTPOLE_CALLBACK_ERROR,
    CARTPOLE_MEASUREMENT_UNAVAILABLE,
    CARTPOLE_SESSION_UNAVAILABLE,
    CARTPOLE_CONTROL_DEADLINE,
    CARTPOLE_CONFIGURATION_CHANGED,
    CARTPOLE_BUSY,
    CARTPOLE_POWER_UNAVAILABLE
} cartpole_result_t;
typedef enum {
    CARTPOLE_STOP_UNCONFIRMED = -1,
    CARTPOLE_STOP_DISABLED = 0,
    CARTPOLE_STOP_FAULT = 1
} cartpole_stop_result_t;
typedef enum { CARTPOLE_LEFT, CARTPOLE_RIGHT } cartpole_endpoint_t;
typedef struct {
    int cart_valid, pole_valid;
    double cart_position_m, pole_angle_rad, cart_velocity_m_s, pole_angular_velocity_rad_s;
    uint64_t measurement_id;          /* 本会话主机测量计数，非设备序号 */
    double received_host_monotonic_s; /* 最后测量接收时刻，非设备采样时刻；未收到=-1
                                       */
} cartpole_measurement_t;
typedef struct {
    /* 打开连接不等于设备健康；is_mode_received表示本次连接后曾收到模式。 */
    int connection_open, is_mode_received;
    cartpole_mode_t reported_mode;
    cartpole_reason_t reported_mode_reason;
    /* 禁止继续非零驱动；零速与停机清理仍可用。FAULT历史不被READY/RESET清除。 */
    int has_stream_failure_latch, has_received_fault_mode;
    uint64_t received_fault_mode_count; /* 本会话收到的 FAULT 帧数，退回 DISABLED 不清零。 */
} cartpole_session_t;
typedef enum {
    CARTPOLE_CALIBRATION_NOT_READY,
    CARTPOLE_CALIBRATION_READY,
    CARTPOLE_CALIBRATION_REFERENCE_REQUIRED
} cartpole_calibration_state_t;
typedef struct {
    int complete; /* 全部部署配置已获取；false时其他值不构成可运行配置 */
    cartpole_calibration_state_t calibration_state;
    int calibration_saved, calibration_storage_ok;
    int has_legacy_watchdog_deceleration;
    int64_t track_length_um;
    int64_t velocity_limit_mm_s, acceleration_limit_mm_s2, watchdog_timeout_ms, track_margin_um;
} cartpole_configuration_t;
/* 固件 CALIB? 的原始状态与存储错误码；查询成功不等于可使能。 */
typedef struct {
    int64_t state, length_um, saved, storage;
} cartpole_calibration_status_t;
typedef enum {
    CARTPOLE_SETTING_VELOCITY_LIMIT,
    CARTPOLE_SETTING_ACCELERATION_LIMIT,
    CARTPOLE_SETTING_WATCHDOG_TIMEOUT,
    CARTPOLE_SETTING_TRACK_MARGIN,
    CARTPOLE_SETTING_POLE_ZERO,
    CARTPOLE_SETTING_POLE_INVERT,
    CARTPOLE_SETTING_CART_INVERT,
    CARTPOLE_SETTING_STEP_LOSS_THRESHOLD,
    CARTPOLE_SETTING_COUNT
} cartpole_setting_t;
/* 固件设置的完整读回，按 cartpole_setting_t 索引。与控制预检摘要独立。 */
typedef struct {
    int64_t values[CARTPOLE_SETTING_COUNT];
} cartpole_settings_t;

typedef struct {
    int64_t zero_urad, inverted;
} cartpole_pole_settings_t;
typedef enum {
    CARTPOLE_EVENT_MEASUREMENT,
    CARTPOLE_EVENT_SESSION,
    CARTPOLE_EVENT_CONFIGURATION,
    CARTPOLE_EVENT_STEP_LOSS,
    CARTPOLE_EVENT_FIRMWARE_ERROR,
    CARTPOLE_EVENT_UNRECOGNIZED_LINE
} cartpole_event_kind_t;
typedef struct {
    cartpole_event_kind_t kind;
    union {
        cartpole_measurement_t measurement;
        cartpole_session_t session;
        cartpole_configuration_t configuration;
        struct {
            int64_t event_id, run_id, device_time_us;
        } step_loss;
    } data;
} cartpole_event_t;
/* 同步逐事件订阅，无队列。回调只读，可复制event或request_stop，不得调用修改会话的API。
 */
typedef void (*cartpole_event_handler_t)(const cartpole_event_t *, void *);
typedef struct cartpole cartpole_t;

cartpole_result_t cartpole_subscribe(cartpole_t *, cartpole_event_handler_t, void *);
cartpole_measurement_t cartpole_get_latest_measurement(cartpole_t *);
cartpole_session_t cartpole_get_session_state(cartpole_t *);
cartpole_configuration_t cartpole_get_configuration(cartpole_t *);
/* CLOCK_MONOTONIC接收年龄（秒），不是设备采样年龄；未接收返回负数。 */
double cartpole_calculate_measurement_age_s(cartpole_t *, cartpole_measurement_t);
void cartpole_request_stop(cartpole_t *);
int cartpole_is_stop_requested(cartpole_t *);
double cartpole_get_monotonic_time_s(cartpole_t *);
/* timeout_s必须有限且>0，覆盖整次事务，不是每个阶段各自计时。
 * TIMEOUT只表示执行未确认，不自动重试。只读事务在FAULT下可用。
 * query_mode/wait_for_measurement成功只表示收到新报告，不承诺READY/测量有效。
 * configure值单位按setting：mm/s、mm/s²、ms、µm、µrad或反向布尔0/1。 */
cartpole_result_t cartpole_query_mode(cartpole_t *, double timeout_s);
cartpole_result_t cartpole_wait_for_measurement(cartpole_t *, double timeout_s);
/* 在已收到 DISABLED NONE 后请求使能；发送时必须有新鲜、双路有效的 STATE。
 * 发送前发现新的 FAULT、模式转移或无效测量即中止，不自动重发已写入的 ENABLE。 */
cartpole_result_t cartpole_enable(cartpole_t *, double timeout_s);
/* 仅发送禁用请求，不减速；非零速时返回 BUSY，设备故障时保留 FAULT。 */
cartpole_result_t cartpole_disable(cartpole_t *, double timeout_s);
cartpole_result_t cartpole_reset(cartpole_t *, double timeout_s);
/* 仅成功才返回固件读数（左端位置或标定长度，单位 µm）。 */
cartpole_result_t cartpole_mark_endpoint(cartpole_t *, cartpole_endpoint_t,
                                                  int64_t *value_um, double timeout_s);
/* 确认保存行程的左参考点，不使能。成功才写入 position_um。 */
cartpole_result_t cartpole_confirm_left(cartpole_t *, int64_t *position_um, double timeout_s);
/* 只查询当前轨道标定，成功才写入 out。 */
cartpole_result_t cartpole_read_calibration_status(cartpole_t *, cartpole_calibration_status_t *out,
                                                 double timeout_s);
cartpole_result_t cartpole_configure(cartpole_t *, cartpole_setting_t, long value,
                                     double timeout_s);
cartpole_result_t cartpole_refresh_configuration(cartpole_t *, double timeout_s);
/* 重新读取全部设置；失败不修改 out。 */
cartpole_result_t cartpole_read_settings(cartpole_t *, cartpole_settings_t *out, double timeout_s);
/* 如超时，只返回本次已收到字段的掩码，不修改 out；位号对应 cartpole_setting_t。 */
cartpole_result_t cartpole_read_settings_observed(cartpole_t *, cartpole_settings_t *out,
                                                uint32_t *received_mask, double timeout_s);
cartpole_result_t cartpole_read_pole_settings(cartpole_t *, cartpole_pole_settings_t *, double timeout_s);
typedef enum { CARTPOLE_CONFIGURE_CHECK, CARTPOLE_CONFIGURE_ATTEMPTED } cartpole_configure_stage_t;
/* 在设备排空待收消息后、每次尝试写入前同步检查。ATTEMPTED 表示帧已由驱动接收
 * 或写入结果有疑义，不代表收到固件回执。回调不得调用设备 IO。 */
typedef cartpole_result_t (*cartpole_configure_observer_t)(cartpole_configure_stage_t, void *);
cartpole_result_t cartpole_configure_checked(cartpole_t *, cartpole_setting_t, long, double,
                                             cartpole_configure_observer_t, void *);
/* 清理忽略已请求取消与流失败；FAULT保留故障，不等同DISABLED。
 * UNCONFIRMED不证明设备仍在运动，也不证明已经停下。 */
cartpole_stop_result_t cartpole_stop_and_confirm(cartpole_t *, double timeout_s);
typedef enum {
    CARTPOLE_RECEIVE_ERROR = -1,
    CARTPOLE_RECEIVE_EMPTY = 0,
    CARTPOLE_RECEIVE_BACKLOG = 1
} cartpole_receive_result_t;
typedef enum {
    CARTPOLE_SEND_ERROR = -1,
    CARTPOLE_SEND_COMPLETE = 0,
    CARTPOLE_SEND_SKIPPED = 1,
    CARTPOLE_SEND_LATE = 2
} cartpole_send_result_t;

/* 会话借用设备及其时钟；一个设备仅绑定一个会话。destroy 不关闭设备、不停车。 */
typedef struct cartpole_device cartpole_device_t;
cartpole_t *cartpole_create(cartpole_device_t device);
void cartpole_destroy(cartpole_t *);
typedef int (*cartpole_cancellation_probe_t)(void *);
/* 取消源由调用者持有，必须活过会话；探针只读且非阻塞，不重入、不销毁会话，不调用任何会话 API。
 * NULL 解除外部取消源；只能在没有活动操作时修改绑定。 */
cartpole_result_t cartpole_set_cancellation_source(cartpole_t *, cartpole_cancellation_probe_t, void *);
/* 接收所有当前可处理报告，逐条通知订阅者；BACKLOG 要求稍后继续接收。 */
cartpole_receive_result_t cartpole_receive(cartpole_t *);
/* 绝对期限基于注入时钟；try_once 不等待重试；完整写入不是设备收讫。 */
cartpole_send_result_t cartpole_send_velocity(cartpole_t *, double velocity_mm_s,
                                            double deadline_s, int try_once);
cartpole_result_t cartpole_get_stream_result(cartpole_t *);
/* 非负有限绝对时间；可被取消中断，回调内禁止阻塞。 */
cartpole_result_t cartpole_sleep_until(cartpole_t *, double deadline_s);
/* 只在有界停车清理中使用；即使已取消也等待注入时钟到达期限。 */
cartpole_result_t cartpole_sleep_for_cleanup_until(cartpole_t *, double deadline_s);
typedef cartpole_result_t (*cartpole_operation_t)(cartpole_t *, void *);
/* 同步独占一个设备操作：禁止操作重入或销毁会话；observer 在显示订阅前
 * 收到每一条报告，不能修改会话。返回时自动解绑 observer 和解除独占。
 * body 可以使用基本设备能力；退出不代表停车，调用者仍负责停机确认。 */
cartpole_result_t cartpole_execute_operation(cartpole_t *, cartpole_operation_t,
                                            cartpole_event_handler_t observer, void *);
typedef int (*cartpole_measurement_evaluator_t)(void *);
/* 同步计算业务结果；只能读取快照或请求取消，不能修改会话/发起设备事务。
 * 用户返回非零或违反只读约定均返回 CALLBACK_ERROR。 */
cartpole_result_t cartpole_evaluate_measurement(cartpole_t *, cartpole_measurement_evaluator_t, void *);
#endif
