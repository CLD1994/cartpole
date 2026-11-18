/* 单设备、单线程业务会话；事件循环负责接收与调度，无后台线程。
 * 所有读取接口返回值副本，不驱动接收。open/close不等于停车。
 * open/close重置本地测量、统计、配置、订阅和故障锁存。 */
#ifndef EDGE_CARTPOLE_H
#define EDGE_CARTPOLE_H
#include <stdint.h>
typedef enum { CARTPOLE_MODE_DISABLED, CARTPOLE_MODE_READY, CARTPOLE_MODE_FAULT } cartpole_mode_t;
typedef enum {
    CARTPOLE_REASON_NONE,
    CARTPOLE_REASON_STEP_LOSS,
    CARTPOLE_REASON_AS5600,
    CARTPOLE_REASON_MT6835
} cartpole_reason_t;
typedef enum {
    CARTPOLE_OK = 0,
    CARTPOLE_CANCELLED,
    CARTPOLE_TIMEOUT,
    CARTPOLE_TRANSPORT_ERROR,
    CARTPOLE_PROTOCOL_ERROR,
    CARTPOLE_REJECTED,
    CARTPOLE_FAULT,
    CARTPOLE_REPLY_MISMATCH,
    CARTPOLE_INVALID_ARGUMENT,
    CARTPOLE_CALLBACK_ERROR,
    CARTPOLE_MEASUREMENT_UNAVAILABLE,
    CARTPOLE_SESSION_UNAVAILABLE,
    CARTPOLE_CONTROL_DEADLINE,
    CARTPOLE_CONFIGURATION_CHANGED
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
    double received_host_monotonic_s; /* 最后测量接收时刻，非设备采样时刻；未收到=-1 */
} cartpole_measurement_t;
typedef struct {
    /* 打开连接不等于设备健康；is_mode_received表示本次open后曾收到模式。 */
    int connection_open, is_mode_received;
    cartpole_mode_t reported_mode;
    cartpole_reason_t reported_mode_reason;
    /* 禁止继续非零驱动；零速与停机清理仍可用。FAULT历史不被READY/RESET清除。 */
    int has_stream_failure_latch, has_received_fault_mode;
} cartpole_session_t;
/* 累计主机接收统计；unknown含接收器发现的异常行，非设备丢包计数。 */
typedef struct {
    uint64_t received_measurements, unknown_received_lines;
} cartpole_statistics_t;
typedef enum {
    CARTPOLE_CALIBRATION_NOT_READY,
    CARTPOLE_CALIBRATION_READY,
    CARTPOLE_CALIBRATION_REFERENCE_REQUIRED
} cartpole_calibration_state_t;
typedef struct {
    int complete; /* 全部部署配置已获取；false时其他值不构成可运行配置 */
    cartpole_calibration_state_t calibration_state;
    int calibration_saved, calibration_storage_ok;
    int64_t track_length_um;
    int64_t velocity_limit_mm_s, acceleration_limit_mm_s2, watchdog_timeout_ms, track_margin_um;
} cartpole_configuration_t;
typedef enum {
    CARTPOLE_SETTING_VELOCITY_LIMIT,
    CARTPOLE_SETTING_ACCELERATION_LIMIT,
    CARTPOLE_SETTING_WATCHDOG_TIMEOUT,
    CARTPOLE_SETTING_TRACK_MARGIN,
    CARTPOLE_SETTING_POLE_ZERO,
    CARTPOLE_SETTING_POLE_INVERT,
    CARTPOLE_SETTING_CART_INVERT,
    CARTPOLE_SETTING_STEP_LOSS_THRESHOLD
} cartpole_setting_t;
typedef enum {
    CARTPOLE_EVENT_MEASUREMENT,
    CARTPOLE_EVENT_SESSION,
    CARTPOLE_EVENT_CONFIGURATION,
    CARTPOLE_EVENT_STEP_LOSS
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
/* 同步逐事件订阅，无队列。回调只读，可复制event或request_stop，不得调用修改会话的API。 */
typedef void (*cartpole_event_handler_t)(const cartpole_event_t *, void *);
cartpole_result_t cartpole_subscribe(cartpole_event_handler_t, void *);
cartpole_measurement_t cartpole_measurement(void);
cartpole_session_t cartpole_session(void);
cartpole_statistics_t cartpole_statistics(void);
cartpole_configuration_t cartpole_configuration(void);
/* CLOCK_MONOTONIC接收年龄（秒），不是设备采样年龄；未接收返回负数。 */
double cartpole_measurement_age_s(cartpole_measurement_t);
const char *cartpole_mode_name(cartpole_mode_t);
const char *cartpole_reason_name(cartpole_reason_t);
const char *cartpole_result_name(cartpole_result_t);
const char *cartpole_setting_name(cartpole_setting_t);
int cartpole_setting_from_name(const char *, cartpole_setting_t *);
int cartpole_open(const char *port, int baud);
void cartpole_close(void);
void cartpole_request_stop(void);
int cartpole_stop_requested(void);
double cartpole_host_monotonic_s(void);
/* timeout_s必须有限且>0，覆盖整次事务，不是每个阶段各自计时。
 * TIMEOUT只表示执行未确认，不自动重试。只读事务在FAULT下可用。
 * query_mode/wait_measurement成功只表示收到新报告，不承诺READY/测量有效。
 * configure值单位按setting：mm/s、mm/s²、ms、µm、µrad或反向布尔0/1。 */
cartpole_result_t cartpole_query_mode(double timeout_s);
cartpole_result_t cartpole_wait_measurement(double timeout_s);
cartpole_result_t cartpole_enable(double timeout_s);
cartpole_result_t cartpole_reset(double timeout_s);
cartpole_result_t cartpole_mark(cartpole_endpoint_t, double timeout_s);
cartpole_result_t cartpole_configure(cartpole_setting_t, long value, double timeout_s);
cartpole_result_t cartpole_refresh_configuration(double timeout_s);
/* 清理忽略已请求取消与流失败；FAULT保留故障，不等同DISABLED。
 * UNCONFIRMED不证明设备仍在运动，也不证明已经停下。 */
cartpole_stop_result_t cartpole_stop_and_confirm(double timeout_s);
/* 循环回调只给业务意图，不能发送、执行事务、修改订阅、重入循环或关闭连接。 */
typedef enum {
    CARTPOLE_TICK_OBSERVE,
    CARTPOLE_TICK_VELOCITY,
    CARTPOLE_TICK_FINISH
} cartpole_tick_kind_t;
typedef struct {
    cartpole_tick_kind_t kind;
    double velocity_mm_s;
} cartpole_tick_action_t;
typedef int (*cartpole_tick_handler_t)(double elapsed_s, cartpole_tick_action_t *, void *);
/* duration_s=0持续至FINISH/取消；period_s>0，均须有限。
 * 接收积压时跳过业务tick；错过节拍不补发历史目标。tick返回非0或非法重入
 * 为CALLBACK_ERROR，request_stop为CANCELLED，时长结束/FINISH为OK。
 * 所有循环退出均不等于停车；调用方必须随后stop_and_confirm。
 * 回调同步执行，不能抢占；超时后不再提交该回调产生的运动意图。 */
cartpole_result_t cartpole_run_loop(double period_s, double duration_s, cartpole_tick_handler_t,
                                    void *);
typedef struct {
    /* 支持400/500 Hz；动作倍率与部署限幅单位mm/s。 */
    double control_hz, action_scale_mm_s, velocity_limit_mm_s;
    cartpole_configuration_t expected_configuration;
} cartpole_controller_options_t;
/* 回调必须写入有限归一化动作且返回0；应用负责观察量排列及角度符号转换。
 * 同tick/event回调一样只读或request_stop，不得执行会话事务。 */
typedef int (*cartpole_infer_t)(const cartpole_measurement_t *, float *normalized_action, void *);
cartpole_result_t cartpole_run_controller(const cartpole_controller_options_t *, cartpole_infer_t,
                                          void *);
/* hold先查询模式/等待测量，duration为之后的保持时长；结束请求零速，
 * 不确认DISABLED。center总timeout包含预检，新零速测量才构成停稳证据。
 * 两者退出后仍由调用方统一stop_and_confirm。 */
cartpole_result_t cartpole_hold_velocity(double velocity_mm_s, double duration_s);
cartpole_result_t cartpole_move_to_center(double timeout_s);
#endif
