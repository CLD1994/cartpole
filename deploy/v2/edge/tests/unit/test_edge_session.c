#include "cartpole.h"
#include "uart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
static char input[16384];
static size_t used;
static int reads, broken, write_fail;
static const char *query_reply, *enable_reply, *disable_reply, *config_reply;
static int zero_state = 1;
static int config_queries, calib_queries;
static int try_result, try_calls, framing_newlines;
static double clock_s = 1., write_delay;
int clock_gettime(clockid_t id, struct timespec *t)
{
    assert(id == CLOCK_MONOTONIC);
    t->tv_sec = (time_t)clock_s;
    t->tv_nsec = (long)((clock_s - (double)t->tv_sec) * 1e9);
    clock_s += .0001;
    return 0;
}
int uart_try_write_all(uart_t *u, const uint8_t *data, size_t n)
{
    (void)u;
    (void)data;
    (void)n;
    try_calls++;
    clock_s += write_delay;
    return try_result;
}
static void queue(const char *s)
{
    size_t n = strlen(s);
    assert(used + n < sizeof input);
    memcpy(input + used, s, n);
    used += n;
}
int uart_open(uart_t *u, const char *p, int b)
{
    (void)p;
    (void)b;
    u->fd = 1;
    used = 0;
    reads = 0;
    broken = 0;
    write_fail = 0;
    query_reply = enable_reply = disable_reply = config_reply = NULL;
    return 0;
}
void uart_close(uart_t *u)
{
    u->fd = -1;
}
int uart_read(uart_t *u, uint8_t *d, size_t n)
{
    (void)u;
    reads++;
    if (broken)
        return -1;
    if (n > used)
        n = used;
    memcpy(d, input, n);
    memmove(input, input + n, used - n);
    used -= n;
    return (int)n;
}
int uart_write_all(uart_t *u, const uint8_t *d, size_t n)
{
    (void)u;
    if (write_fail)
        return -1;
    char cmd[128];
    assert(n < sizeof cmd);
    memcpy(cmd, d, n);
    cmd[n] = 0;
    if (!strcmp(cmd, "\n"))
        framing_newlines++;
    if (!strcmp(cmd, "VEL 0\n") && zero_state)
        queue("STATE 3 0 0 0 0\n");
    if (!strcmp(cmd, "QUERY\n") && query_reply)
        queue(query_reply);
    if (!strcmp(cmd, "ENABLE\n") && enable_reply)
        queue(enable_reply);
    if (!strcmp(cmd, "DISABLE\n") && disable_reply)
        queue(disable_reply);
    if (!strncmp(cmd, "CONFIG ", 7) && config_reply)
        queue(config_reply);
    if (!strcmp(cmd, "CONFIG?\n")) {
        config_queries++;
        queue("OK VEL_MAX 900\nOK ACC_MAX 10000\nOK WDT_MS 100\nOK X_MARGIN_UM 10000\n");
    }
    if (!strcmp(cmd, "CALIB?\n")) {
        calib_queries++;
        queue("OK CAL_STATE 2\nOK CAL_LENGTH_UM 894845\nOK CAL_SAVED 1\nOK CAL_STORAGE 0\n");
    }
    return 0;
}
static void open_ready(void)
{
    assert(rig_open("fake", 460800) == 0);
    query_reply = "MODE READY NONE\n";
    assert(rig_sync_mode(.02) == 0);
    queue("STATE 3 1 2 3 4\n");
    assert(rig_pump() == 0);
}
static int observed_flags[8], hook_calls;
static fw_mode_t observed_modes[8];
static void update_hook(void *context)
{
    assert(context == &hook_calls);
    assert(hook_calls < 8);
    observed_flags[hook_calls] = rig_valid_flags();
    observed_modes[hook_calls] = rig_mode(NULL, NULL);
    hook_calls++;
}

int main(void)
{
    open_ready();
    query_reply = "MODE FAULT MT6835\n";
    assert(rig_sync_mode(.02) == 0);
    assert(rig_mode(NULL, NULL) == FW_FAULT);
    rig_request_stop();
    rig_close();
    assert(rig_open("fake", 460800) == 0);
    assert(!rig_stop_requested());
    int known;
    rig_mode(&known, NULL);
    assert(!known);
    assert(rig_state_age_s() < 0);
    assert(rig_valid_flags() == 0);
    /* 每次QUERY必须等新响应。 */
    open_ready();
    query_reply = NULL;
    assert(rig_sync_mode(.003) == -1);
    /* ENABLE按新MODE而非OK确认，不能复用已有READY。 */
    open_ready();
    assert(rig_enable(.003) == -1);
    enable_reply = "MODE READY NONE\n";
    assert(rig_enable(.02) == 0);
    enable_reply = "ERR ENABLE\nMODE READY NONE\n";
    assert(rig_enable(.02) == -1);
    /* 无效小车不能变成零值有效测量。 */
    queue("STATE 2 0 20 0 2\n");
    assert(rig_pump() == 0);
    assert(rig_valid_flags() == 2);
    assert(rig_send_vel(10) == -1);
    assert(rig_send_vel(0) == 0);
    /* 不匹配回执不能满足配置，回显不同不能报成功。 */
    open_ready();
    config_reply = "OK ACC_MAX 5000\n";
    assert(rig_configure("VEL_MAX", 900, .003) == -1);
    config_reply = "OK VEL_MAX 800\n";
    assert(rig_configure("VEL_MAX", 900, .02) == -1);
    config_reply = "ERR VEL_MAX\n";
    assert(rig_configure("VEL_MAX", 900, .02) == -1);
    config_reply = "ERR VEL_MAX\nOK VEL_MAX 900\n";
    assert(rig_configure("VEL_MAX", 900, .02) == -1);
    config_reply = "OK ACC_MAX 5000\nOK VEL_MAX 900\n";
    assert(rig_configure("VEL_MAX", 900, .02) == 0);
    /* 一次刷新采集全部元数据，读缓存不再发查询。 */
    assert(rig_refresh_metadata(.02) == 0);
    int cq = config_queries, ca = calib_queries;
    int64_t v;
    assert(rig_get_config("CAL_LENGTH_UM", &v, 0) == 0 && v == 894845);
    assert(rig_get_config("VEL_MAX", &v, 0) == 0 && v == 900);
    assert(cq == config_queries && ca == calib_queries);
    /* pump有界且故障不能视为无数据。 */
    open_ready();
    for (int i = 0; i < 600; i++)
        queue("STATE 3 0 0 0 0\n");
    reads = 0;
    assert(rig_pump() == 1 && reads == 8 && used > 0);
    open_ready();
    broken = 1;
    assert(rig_pump() == -1 && rig_io_error());
    /* 信号之后仍确认退出，FAULT不冒充DISABLED。 */
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE DISABLED NONE\n";
    rig_request_stop();
    assert(rig_stop_and_confirm(.02) == 0);
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE FAULT MT6835\n";
    assert(rig_stop_and_confirm(.02) == 1);
    open_ready();
    query_reply = NULL;
    assert(rig_stop_and_confirm(.003) == -1);
    open_ready();
    queue("STATE 3 invalid\n");
    assert(rig_pump() == -1);
    open_ready();
    query_reply = NULL;
    queue("MODE DISABLED NONE\n");
    zero_state = 0;
    assert(rig_stop_and_confirm(.003) == -1);
    zero_state = 1;
    /* 一次pump中FAULT后READY仍保留故障历史。 */
    open_ready();
    queue("MODE FAULT MT6835\nMODE READY NONE\n");
    assert(rig_pump() == 0);
    assert(rig_fault_seen());
    open_ready();
    double received_at = rig_state_received_at_s();
    assert(received_at > 0 && received_at <= rig_now_s());
    rig_set_update_hook(update_hook, &hook_calls);
    queue("STATE 1 0 0 0 0\nSTATE 3 0 0 0 0\nMODE DISABLED NONE\nMODE READY NONE\nOK ACC_MAX "
          "10000\n");
    assert(rig_pump() == 0 && hook_calls == 5);
    assert(observed_flags[0] == 1 && observed_flags[1] == 3);
    assert(observed_modes[2] == FW_DISABLED && observed_modes[3] == FW_READY);
    assert(rig_state_received_at_s() >= received_at);
    open_ready();
    assert(hook_calls == 5); /* open clears hook */
    open_ready();
    try_result = 0;
    int old_calls = try_calls;
    assert(rig_try_send_vel(10, rig_now_s() - 1.) == 1 && try_calls == old_calls);
    assert(rig_try_send_vel(10, rig_now_s() + 1.) == 0);
    write_delay = .02;
    assert(rig_try_send_vel(10, rig_now_s() + .01) == 2);
    write_delay = 0.;
    try_result = 1;
    assert(rig_try_send_vel(10, rig_now_s() + 1.) == 1 && !rig_io_error());
    try_result = -2;
    assert(rig_try_send_vel(10, rig_now_s() + 1.) == -1 && rig_io_error());
    int old_newlines = framing_newlines;
    assert(rig_send_vel(0) == 0 && framing_newlines == old_newlines + 1);
    puts("edge session passed");
    return 0;
}
