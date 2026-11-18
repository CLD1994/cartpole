#include "cartpole.h"
#include "uart.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
static char input[16384];
static size_t used;
static int reads, broken, write_fail;
static const char *query_reply, *enable_reply, *disable_reply, *config_reply, *mark_reply,
    *reset_reply;
static int zero_state = 1;
static int config_queries, calib_queries;
static int try_result, try_calls, framing_newlines, cancel_once, enable_writes;
static int cancel_transport, metadata_missing;
static double clock_s = 1., write_delay, read_delay;
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
    if (cancel_transport) {
        cartpole_request_stop();
        return -1;
    }
    if (cancel_once) {
        cancel_once = 0;
        cartpole_request_stop();
        return 1;
    }
    clock_s += write_delay;
    return try_result ? try_result : uart_write_all(u, data, n);
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
    query_reply = enable_reply = disable_reply = config_reply = mark_reply = reset_reply = NULL;
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
    if (used)
        clock_s += read_delay;
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
    if (!strcmp(cmd, "MARK LEFT\n") && mark_reply)
        queue(mark_reply);
    if (!strcmp(cmd, "RESET\n") && reset_reply)
        queue(reset_reply);
    if (!strcmp(cmd, "ENABLE\n"))
        enable_writes++;
    if (!strcmp(cmd, "ENABLE\n") && enable_reply)
        queue(enable_reply);
    if (!strcmp(cmd, "DISABLE\n") && disable_reply)
        queue(disable_reply);
    if (!strncmp(cmd, "CONFIG ", 7) && config_reply)
        queue(config_reply);
    if (!strcmp(cmd, "CONFIG?\n")) {
        config_queries++;
        queue(metadata_missing
                  ? "ERR VEL_MAX\n"
                  : "OK VEL_MAX 900\nOK ACC_MAX 10000\nOK WDT_MS 100\nOK X_MARGIN_UM 10000\n");
    }
    if (!strcmp(cmd, "CALIB?\n")) {
        calib_queries++;
        queue("OK CAL_STATE 2\nOK CAL_LENGTH_UM 894845\nOK CAL_SAVED 1\nOK CAL_STORAGE 0\n");
    }
    return 0;
}
static void open_ready(void)
{
    assert(cartpole_open("fake", 460800) == 0);
    query_reply = "MODE READY NONE\n";
    assert(cartpole_sync_mode(.02) == 0);
    queue("STATE 3 1 2 3 4\n");
    assert(cartpole_pump() == 0);
}
static int observed_flags[8], hook_calls;
static fw_mode_t observed_modes[8];
static proto_msg_type_t observed_events[8];
static cartpole_snapshot_t observed_snapshots[8];
static void update_hook(proto_msg_type_t event, const cartpole_snapshot_t *snapshot, void *context)
{
    assert(event == PROTO_STATE || event == PROTO_MODE || event == PROTO_OK);
    assert(snapshot->state_sequence == cartpole_snapshot().state_sequence);
    assert(context == &hook_calls);
    assert(hook_calls < 8);
    observed_events[hook_calls] = event;
    observed_snapshots[hook_calls] = *snapshot;
    observed_flags[hook_calls] = snapshot->valid_flags;
    observed_modes[hook_calls] = snapshot->mode;
    hook_calls++;
}

int main(void)
{
    open_ready();
    query_reply = "MODE FAULT MT6835\n";
    assert(cartpole_sync_mode(.02) == CARTPOLE_OK);
    assert(cartpole_snapshot().mode == FW_FAULT);
    cartpole_request_stop();
    cartpole_close();
    assert(!cartpole_snapshot().connected && !cartpole_snapshot().mode_known &&
           cartpole_snapshot().valid_flags == 0);
    assert(cartpole_open("fake", 460800) == 0);
    assert(!cartpole_stop_requested());
    int known;
    (known = cartpole_snapshot().mode_known, cartpole_snapshot().mode);
    assert(!known);
    assert(cartpole_snapshot_age(cartpole_snapshot()) < 0);
    assert(cartpole_snapshot().valid_flags == 0);
    /* 事务预算不能在drain/send后重置；过期也不得接受已排队回执。 */
    open_ready();
    enable_reply = "MODE READY NONE\n";
    write_delay = .01;
    double started = cartpole_now_s();
    assert(cartpole_enable(.003) == CARTPOLE_TIMEOUT);
    assert(cartpole_now_s() - started < .014);
    write_delay = 0;
    /* 每次QUERY必须等新响应。 */
    open_ready();
    query_reply = NULL;
    assert(cartpole_sync_mode(.003) == CARTPOLE_TIMEOUT);
    /* ENABLE按新MODE而非OK确认，不能复用已有READY。 */
    open_ready();
    assert(cartpole_enable(.003) == CARTPOLE_TIMEOUT);
    enable_reply = "MODE READY NONE\n";
    assert(cartpole_enable(.02) == 0);
    enable_reply = "ERR ENABLE\nMODE READY NONE\n";
    assert(cartpole_enable(.02) == CARTPOLE_REJECTED);
    /* 无效小车不能变成零值有效测量。 */
    queue("STATE 2 0 20 0 2\n");
    assert(cartpole_pump() == 0);
    assert(cartpole_snapshot().valid_flags == 2);
    assert(cartpole_send_vel(10) == -1);
    assert(cartpole_send_vel(0) == 0);
    /* 不匹配回执不能满足配置，回显不同不能报成功。 */
    open_ready();
    config_reply = "OK ACC_MAX 5000\n";
    assert(cartpole_configure("VEL_MAX", 900, .003) == CARTPOLE_TIMEOUT);
    config_reply = "OK VEL_MAX 800\n";
    assert(cartpole_configure("VEL_MAX", 900, .02) == CARTPOLE_REPLY_MISMATCH);
    config_reply = "ERR VEL_MAX\n";
    assert(cartpole_configure("VEL_MAX", 900, .02) == CARTPOLE_REJECTED);
    config_reply = "ERR VEL_MAX\nOK VEL_MAX 900\n";
    assert(cartpole_configure("VEL_MAX", 900, .02) == CARTPOLE_REJECTED);
    config_reply = "OK ACC_MAX 5000\nOK VEL_MAX 900\n";
    assert(cartpole_configure("VEL_MAX", 900, .02) == 0);
    /* 一次刷新采集全部元数据，读缓存不再发查询。 */
    assert(cartpole_refresh_metadata(.02) == 0);
    int cq = config_queries, ca = calib_queries;
    int64_t v;
    assert(cartpole_cached_value("CAL_LENGTH_UM", &v) == 0 && v == 894845);
    assert(cartpole_cached_value("VEL_MAX", &v) == 0 && v == 900);
    assert(cq == config_queries && ca == calib_queries);
    /* pump有界且故障不能视为无数据。 */
    open_ready();
    for (int i = 0; i < 600; i++)
        queue("STATE 3 0 0 0 0\n");
    reads = 0;
    assert(cartpole_pump() == 1 && reads == 8 && used > 0);
    open_ready();
    broken = 1;
    assert(cartpole_pump() == -1 && cartpole_snapshot().io_error);
    /* 信号之后仍确认退出，FAULT不冒充DISABLED。 */
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE DISABLED NONE\n";
    cartpole_request_stop();
    assert(cartpole_stop_and_confirm(.02) == 0);
    open_ready();
    query_reply = NULL;
    disable_reply = "MODE FAULT MT6835\n";
    assert(cartpole_stop_and_confirm(.02) == 1);
    open_ready();
    query_reply = NULL;
    assert(cartpole_stop_and_confirm(.003) == -1);
    open_ready();
    queue("STATE 3 invalid\n");
    assert(cartpole_pump() == -1);
    open_ready();
    query_reply = NULL;
    queue("MODE DISABLED NONE\n");
    zero_state = 0;
    assert(cartpole_stop_and_confirm(.003) == -1);
    zero_state = 1;
    /* 一次pump中FAULT后READY仍保留故障历史。 */
    open_ready();
    queue("MODE FAULT MT6835\nMODE READY NONE\n");
    assert(cartpole_pump() == 0);
    assert(cartpole_snapshot().fault_seen);
    open_ready();
    double received_at = cartpole_snapshot().state_received_at_s;
    assert(received_at > 0 && received_at <= cartpole_now_s());
    cartpole_snapshot_t before = cartpole_snapshot();
    cartpole_set_observer(update_hook, &hook_calls);
    queue("STATE 1 0 0 0 0\nSTATE 3 0 0 0 0\nMODE DISABLED NONE\nMODE READY NONE\nOK ACC_MAX "
          "10000\n");
    assert(cartpole_pump() == 0 && hook_calls == 5);
    assert(observed_flags[0] == 1 && observed_flags[1] == 3);
    assert(before.state_sequence + 2 == cartpole_snapshot().state_sequence);
    assert(before.state_received_at_s == received_at);
    assert(observed_events[0] == PROTO_STATE && observed_events[2] == PROTO_MODE &&
           observed_events[4] == PROTO_OK);
    for (int i = 2; i < 5; i++) {
        assert(observed_snapshots[i].state_sequence == observed_snapshots[1].state_sequence);
        assert(observed_snapshots[i].state_received_at_s ==
               observed_snapshots[1].state_received_at_s);
    }
    assert(observed_modes[2] == FW_DISABLED && observed_modes[3] == FW_READY);
    assert(cartpole_snapshot().state_received_at_s >= received_at);
    open_ready();
    assert(hook_calls == 5); /* open clears hook */
    open_ready();
    try_result = 0;
    int old_calls = try_calls;
    assert(cartpole_try_send_vel(10, cartpole_now_s() - 1.) == 1 && try_calls == old_calls);
    assert(cartpole_try_send_vel(10, cartpole_now_s() + 1.) == 0);
    write_delay = .02;
    assert(cartpole_try_send_vel(10, cartpole_now_s() + .01) == 2);
    write_delay = 0.;
    try_result = 1;
    assert(cartpole_try_send_vel(10, cartpole_now_s() + 1.) == 1 && !cartpole_snapshot().io_error);
    try_result = -2;
    assert(cartpole_try_send_vel(10, cartpole_now_s() + 1.) == -1 && cartpole_snapshot().io_error);
    try_result = 0;
    int old_newlines = framing_newlines;
    assert(cartpole_send_vel(0) == 0 && framing_newlines == old_newlines + 1);
    open_ready();
    query_reply = "MODE DISABLED NONE\n";
    read_delay = .01;
    assert(cartpole_stop_and_confirm(.003) == CARTPOLE_STOP_UNCONFIRMED);
    read_delay = 0.;
    open_ready();
    assert(cartpole_refresh_metadata(.02) == 0);
    cartpole_close();
    assert(cartpole_cached_value("VEL_MAX", &v) == -1);
    open_ready();
    queue("OK MARK_LEFT -42\n");
    assert(cartpole_mark(CARTPOLE_LEFT, .003) == CARTPOLE_TIMEOUT); /* stale response discarded */
    mark_reply = "OK MARK_RIGHT 84\n";
    assert(cartpole_mark(CARTPOLE_LEFT, .003) == CARTPOLE_TIMEOUT); /* request correlation */
    mark_reply = "OK MARK_LEFT -42\n";
    assert(cartpole_mark(CARTPOLE_LEFT, .02) == 0);
    assert(cartpole_mark((cartpole_endpoint_t)99, .02) == CARTPOLE_INVALID_ARGUMENT);
    reset_reply = "MODE DISABLED NONE\n";
    assert(cartpole_reset(.02) == 0);
    assert(cartpole_snapshot().mode == FW_DISABLED);
    reset_reply = "ERR RESET\nMODE DISABLED NONE\n";
    assert(cartpole_reset(.02) == CARTPOLE_REJECTED);
    open_ready();
    int before_enable = enable_writes;
    cancel_once = 1;
    assert(cartpole_enable(.02) == CARTPOLE_CANCELLED);
    assert(enable_writes == before_enable);
    query_reply = "MODE DISABLED NONE\n";
    assert(cartpole_stop_and_confirm(.02) == CARTPOLE_STOP_DISABLED);
    open_ready();
    assert(cartpole_configure(NULL, 900, .02) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_configure("BAD\nNAME", 900, .02) == CARTPOLE_INVALID_ARGUMENT);
    assert(!cartpole_snapshot().io_error);
    assert(cartpole_sync_mode(0) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_wait_state(-1) == CARTPOLE_INVALID_ARGUMENT);
    broken = 1;
    assert(cartpole_sync_mode(.02) == CARTPOLE_TRANSPORT_ERROR);
    open_ready();
    queue("STATE invalid\n");
    assert(cartpole_wait_state(.02) == CARTPOLE_PROTOCOL_ERROR);
    open_ready();
    enable_reply = "MODE FAULT MT6835\nMODE READY NONE\n";
    assert(cartpole_enable(.02) == CARTPOLE_FAULT);
    open_ready();
    assert(cartpole_enable(NAN) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_reset(INFINITY) == CARTPOLE_INVALID_ARGUMENT);
    assert(cartpole_refresh_metadata(-.1) == CARTPOLE_INVALID_ARGUMENT);
    metadata_missing = 1;
    assert(cartpole_refresh_metadata(.003) == CARTPOLE_TIMEOUT);
    metadata_missing = 0;
    cancel_transport = 1;
    assert(cartpole_enable(.02) == CARTPOLE_TRANSPORT_ERROR);
    cancel_transport = 0;
    open_ready();
    query_reply = "MODE FAULT MT6835\n";
    assert(cartpole_sync_mode(.02) == CARTPOLE_OK);
    assert(cartpole_refresh_metadata(.02) == CARTPOLE_OK);
    reset_reply = "MODE DISABLED NONE\n";
    assert(cartpole_reset(.02) == CARTPOLE_OK); /* historicalFAULT isn't newFAULT */
    open_ready();
    assert(cartpole_send_vel(NAN) == CARTPOLE_SEND_ERROR && !cartpole_snapshot().io_error);
    try_result = 1;
    assert(cartpole_send_vel(10) == CARTPOLE_SEND_ERROR);
    assert(cartpole_snapshot().io_error);
    try_result = 0;
    int before_retry = try_calls;
    assert(cartpole_send_vel(10) == CARTPOLE_SEND_ERROR && try_calls == before_retry);
    assert(cartpole_send_vel(0) == CARTPOLE_SEND_COMPLETE);
    assert(cartpole_sync_mode(.02) ==
           CARTPOLE_TIMEOUT); /* original timeout, not invented IO error */
    puts("edge session passed");
    return 0;
}
