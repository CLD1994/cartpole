#include "cartpole_runtime.h"
#include <errno.h>
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
    cartpole_runtime_t *runtime = context;
    struct timespec ts = {(time_t)deadline, (long)((deadline - floor(deadline)) * 1e9)};
    while (!cartpole_runtime_is_cancelled(runtime) &&
           clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
    }
}
int cartpole_runtime_init(cartpole_runtime_t *runtime, const cartpole_clock_t *clock)
{
    if (!runtime || (clock && (!clock->now || !clock->sleep_until)))
        return -1;
    *runtime = (cartpole_runtime_t){0};
    runtime->clock = clock ? *clock : (cartpole_clock_t){monotonic_now, monotonic_sleep, runtime};
    return 0;
}
double cartpole_runtime_now(cartpole_runtime_t *runtime)
{
    return runtime->clock.now(runtime->clock.context);
}
void cartpole_runtime_sleep_until(cartpole_runtime_t *runtime, double time)
{
    runtime->clock.sleep_until(runtime->clock.context, time);
}
int cartpole_runtime_is_cancelled(cartpole_runtime_t *runtime)
{
    return runtime->stop_requested != 0 || (runtime->external_stop && *runtime->external_stop);
}
