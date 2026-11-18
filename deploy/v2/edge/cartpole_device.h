#ifndef EDGE_CARTPOLE_DEVICE_H
#define EDGE_CARTPOLE_DEVICE_H
#include "cartpole.h"
#include "cartpole_runtime.h"
/* 业务需要的设备能力。实现负责协议/IO及逐条有序报告；不得丢弃批内中间状态。 */
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
typedef struct {
    cartpole_measurement_t (*get_latest_measurement)(void *);
    cartpole_session_t (*get_session_state)(void *);
    cartpole_configuration_t (*get_configuration)(void *);
    cartpole_receive_result_t (*receive)(void *);
    cartpole_result_t (*get_error)(void *);
    cartpole_result_t (*attach_observer)(void *, cartpole_event_handler_t, void *);
    /* 同步解除观察器，返回后不得使用旧context，不得失败。 */
    void (*detach_observer)(void *);
    void (*report_failure)(void *, cartpole_result_t);
    cartpole_result_t (*query_mode)(void *, double);
    cartpole_result_t (*wait_measurement)(void *, double);
    cartpole_result_t (*enable)(void *, double);
    cartpole_result_t (*reset)(void *, double);
    cartpole_result_t (*refresh_configuration)(void *, double);
    cartpole_result_t (*mark)(void *, cartpole_endpoint_t, double);
    cartpole_result_t (*configure)(void *, cartpole_setting_t, long, double);
    cartpole_stop_result_t (*stop_and_confirm)(void *, double);
    /* try_once非零：不等待重试；deadline为同一runtime的绝对单调时间。
     * COMPLETE只证明本机完整写入，不证明设备收讫。LATE表示写入完成但超过期限。 */
    cartpole_send_result_t (*send_velocity)(void *, double, double, int try_once);
} cartpole_device_ops_t;
typedef struct {
    const cartpole_device_ops_t *ops;
    void *context;
    cartpole_runtime_t *runtime;
} cartpole_device_t;
/* 借用device/runtime，二者必须活过会话。一个设备仅绑定一个会话；destroy解绑observer，
 * 不关闭设备、不发停车命令。适配器与业务必须使用同一runtime。 */
cartpole_t *cartpole_create(cartpole_device_t, cartpole_runtime_t *);
void cartpole_destroy(cartpole_t *);
#endif
