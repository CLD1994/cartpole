#ifndef EDGE_CARTPOLE_CLOCK_H
#define EDGE_CARTPOLE_CLOCK_H
/* 纯时钟能力，可由多个设备/会话共享；不含任何会话状态。 */
typedef struct {
    double (*now)(void *);
    /* 可因信号提前返回；会话层负责按自身取消状态重试至期限。 */
    void (*sleep_until)(void *, double);
    void *context;
} cartpole_clock_t;
/* source=NULL 使用平台单调时钟。 */
int cartpole_clock_init(cartpole_clock_t *, const cartpole_clock_t *source);
double cartpole_clock_get_monotonic_time_s(const cartpole_clock_t *);
void cartpole_clock_sleep_until(const cartpole_clock_t *, double);
#endif
