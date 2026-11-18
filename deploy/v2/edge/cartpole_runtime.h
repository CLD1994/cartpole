#ifndef EDGE_CARTPOLE_RUNTIME_H
#define EDGE_CARTPOLE_RUNTIME_H
#include <signal.h>
/* 同一设备与业务会话共用单调时钟。时钟实现与context由装配层持有。 */
typedef struct {
    double (*now)(void *);
    void (*sleep_until)(void *, double);
    void *context;
} cartpole_clock_t;
typedef struct {
    cartpole_clock_t clock;
    volatile sig_atomic_t stop_requested;
    /* 可选借用的外部取消标志；信号处理器只写该sig_atomic_t。 */
    const volatile sig_atomic_t *external_stop;
    int callback_depth, callback_violation, loop_active;
    int session_bound; /* 每个运行环境只允许一个业务会话，取消与重入状态不跨实例共享。
                        */
} cartpole_runtime_t;
/* clock=NULL使用平台CLOCK_MONOTONIC；自定义时钟须提供两个回调。 */
int cartpole_runtime_init(cartpole_runtime_t *, const cartpole_clock_t *);
double cartpole_runtime_get_monotonic_time_s(cartpole_runtime_t *);
void cartpole_runtime_sleep_until(cartpole_runtime_t *, double);
int cartpole_runtime_is_cancelled(cartpole_runtime_t *);
#endif
