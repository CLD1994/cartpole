#ifndef EDGE_CARTPOLE_CONTROL_H
#define EDGE_CARTPOLE_CONTROL_H
#include "cartpole.h"
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
cartpole_result_t cartpole_run_loop(cartpole_t *, double period_s, double duration_s,
                                    cartpole_tick_handler_t, void *);
typedef struct {
    /* 支持400/500 Hz；动作倍率与部署限幅单位mm/s。 */
    double control_hz, action_scale_mm_s, velocity_limit_mm_s;
    cartpole_configuration_t expected_configuration;
} cartpole_controller_options_t;
/* 回调必须写入有限归一化动作且返回0；应用负责观察量排列及角度符号转换。
 * 同tick/event回调一样只读或request_stop，不得执行会话事务。 */
typedef int (*cartpole_infer_t)(const cartpole_measurement_t *, float *normalized_action, void *);
cartpole_result_t cartpole_run_controller(cartpole_t *, const cartpole_controller_options_t *,
                                          cartpole_infer_t, void *);
/* hold先查询模式/等待测量，duration为之后的保持时长；结束请求零速，
 * 不确认DISABLED。center总timeout包含预检，新零速测量才构成停稳证据。
 * 两者退出后仍由调用方统一stop_and_confirm。 */
cartpole_result_t cartpole_hold_velocity(cartpole_t *, double velocity_mm_s, double duration_s);
cartpole_result_t cartpole_move_to_center(cartpole_t *, double timeout_s);
#endif
