#ifndef DEVICE_SESSION_H
#define DEVICE_SESSION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct device_session device_session;
/* 返回 true 表示完整命令已接受发送；false 包括部分写入。 */
typedef bool (*device_send_fn)(void *context, const char *line);

typedef struct {
    int speed_mm_s;
    int acceleration_mm_s2;
    int soft_margin_um;
    int watchdog_ms;
    int watchdog_decel_mm_s2;
    double state_timeout_s;
    double validity_timeout_s;
    double metadata_timeout_s;
    double stop_timeout_s;
} device_limits;

typedef enum {
    DEVICE_OBSERVING, DEVICE_ACTIVE, DEVICE_STOPPING,
    DEVICE_FINISHED
} device_phase;

typedef enum {
    DEVICE_OK, DEVICE_NOT_READY, DEVICE_INVALID_LINE, DEVICE_STALE,
    DEVICE_INVALID_MEASUREMENT, DEVICE_FIRMWARE_FAULT, DEVICE_WRITE_FAILED,
    DEVICE_INVALID_ACTION, DEVICE_CLOCK_ERROR, DEVICE_STOP_UNCONFIRMED,
    DEVICE_APPLICATION_ERROR
} device_error;

typedef struct {
    int64_t x_um, theta_urad, vx_mm_s, vtheta_mrad_s;
    double received_s;
    bool available;
} device_state;

typedef struct {
    device_phase phase;
    device_error error;
    device_state state;
    bool validity;
    bool mode_ready;
    bool disabled_confirmed;
    int64_t travel_um;
} device_snapshot;

/* 不发送任何命令。limits 由部署配置提供；时钟必须是同一单调时间轴。 */
device_session *device_session_create(device_limits limits,
                                      device_send_fn send, void *context);
void device_session_destroy(device_session *session);
/* 仅 QUERY / CALIB? / CONFIG?；start 要求元数据在规定时间内收到。 */
bool device_session_refresh(device_session *session, double now_s);
/* line 不含换行符；过长/乱码STATE或VEL_VALID不能被当作有效测量。
 * false表示错误或已锁存故障，STOPPING期间仍须继续feed/tick至FINISHED。 */
bool device_session_feed(device_session *session, const char *line, double now_s);
/* 只改变本地会话状态，不 ENABLE、不 CONFIG、不自动定位。 */
bool device_session_start(device_session *session, double now_s);
/* [-1,1] 按 limits.speed_mm_s 映射；越软边界时只封锁向外方向。 */
bool device_session_action(device_session *session, double action, double now_s);
/* 进入有界停机流程，后续 tick/feed 驱动；不证明机械静止。 */
bool device_session_stop(device_session *session, double now_s);
/* 调用方检测到积压、推理异常等时锁存失败；不伪造协议行。 */
void device_session_abort(device_session *session, double now_s);
void device_session_tick(device_session *session, double now_s);
device_snapshot device_session_snapshot(const device_session *session);

#endif
