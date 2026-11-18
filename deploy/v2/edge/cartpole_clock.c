#include "cartpole_clock.h"
#include <math.h>
#include <stddef.h>
#include <time.h>
static double monotonic_now(void *context)
{
    (void)context;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
static void monotonic_sleep(void *context, double deadline)
{
    (void)context;
    struct timespec ts = {(time_t)deadline, (long)((deadline-floor(deadline))*1e9)};
    /* 信号中断即返回，让会话检查自己的取消源。 */
    (void)clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}
int cartpole_clock_init(cartpole_clock_t *clock, const cartpole_clock_t *source)
{
    if (!clock || (source && (!source->now || !source->sleep_until))) return -1;
    *clock = source ? *source : (cartpole_clock_t){monotonic_now, monotonic_sleep, NULL};
    return 0;
}
double cartpole_clock_get_monotonic_time_s(const cartpole_clock_t *clock)
{ return clock->now(clock->context); }
void cartpole_clock_sleep_until(const cartpole_clock_t *clock, double deadline)
{ clock->sleep_until(clock->context, deadline); }
