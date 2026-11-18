/* 交互控制台实现。协议契约见 docs/deployment/protocol.md。 */

#include "console.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "cartpole.h"
#include "config.h"

#define CONSOLE_LINE_MAX 128
#define DUMP_WINDOW_S 0.5 /* CONFIG? dump 的应答收集窗口 */

/* ---- 命令解析（纯函数） ---- */

static int parse_long_token(const char *s, long *out)
{
    char *end;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || errno != 0) {
        return -1;
    }
    *out = v;
    return 0;
}

static int parse_double_token(const char *s, double *out)
{
    char *end;
    errno = 0;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) {
        return -1;
    }
    *out = v;
    return 0;
}

static int valid_param_name(const char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n >= 16) { /* 与 console_req_t.name 同容量，含 NUL */
        return 0;
    }
    for (size_t i = 0; i < n; i++) {
        char c = s[i];
        int ok = (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
        if (!ok) {
            return 0;
        }
    }
    return 1;
}

int console_parse(const char *line, console_req_t *req)
{
    memset(req, 0, sizeof(*req));
    char buf[CONSOLE_LINE_MAX];
    if (strlen(line) >= sizeof(buf)) {
        snprintf(req->err, sizeof(req->err), "line too long");
        return -1;
    }
    strcpy(buf, line);

    char *tok[4];
    int n = 0;
    for (char *p = strtok(buf, " \t"); p != NULL; p = strtok(NULL, " \t")) {
        if (n == 4) {
            n++;
            break;
        }
        tok[n++] = p;
    }
    if (n > 4) {
        snprintf(req->err, sizeof(req->err), "too many arguments");
        return -1;
    }
    if (n == 0) {
        req->cmd = CONSOLE_CMD_NONE;
        return 0;
    }

    if (strcmp(tok[0], "quit") == 0 || strcmp(tok[0], "exit") == 0) {
        if (n != 1) {
            snprintf(req->err, sizeof(req->err), "quit takes no arguments");
            return -1;
        }
        req->cmd = CONSOLE_CMD_QUIT;
        return 0;
    }
    if (strcmp(tok[0], "help") == 0) {
        if (n != 1) {
            snprintf(req->err, sizeof(req->err), "help takes no arguments");
            return -1;
        }
        req->cmd = CONSOLE_CMD_HELP;
        return 0;
    }
    if (strcmp(tok[0], "calib") == 0 || strcmp(tok[0], "mark-left") == 0 ||
        strcmp(tok[0], "mark-right") == 0 || strcmp(tok[0], "enable") == 0 ||
        strcmp(tok[0], "center") == 0 || strcmp(tok[0], "state") == 0 ||
        strcmp(tok[0], "query") == 0 || strcmp(tok[0], "reset") == 0) {
        if (n != 1) {
            snprintf(req->err, sizeof(req->err), "%s takes no arguments", tok[0]);
            return -1;
        }
        req->cmd = strcmp(tok[0], "calib") == 0        ? CONSOLE_CMD_CALIB
                   : strcmp(tok[0], "mark-left") == 0  ? CONSOLE_CMD_MARK_LEFT
                   : strcmp(tok[0], "mark-right") == 0 ? CONSOLE_CMD_MARK_RIGHT
                   : strcmp(tok[0], "enable") == 0     ? CONSOLE_CMD_ENABLE
                   : strcmp(tok[0], "center") == 0     ? CONSOLE_CMD_CENTER
                   : strcmp(tok[0], "state") == 0      ? CONSOLE_CMD_STATE
                   : strcmp(tok[0], "query") == 0      ? CONSOLE_CMD_QUERY
                                                       : CONSOLE_CMD_RESET;
        return 0;
    }
    if (strcmp(tok[0], "vel") == 0) {
        if (n != 2) {
            snprintf(req->err, sizeof(req->err), "usage: vel <mm/s>");
            return -1;
        }
        if (parse_double_token(tok[1], &req->vel_mm_s) != 0) {
            snprintf(req->err, sizeof(req->err), "invalid velocity '%s'", tok[1]);
            return -1;
        }
        req->cmd = CONSOLE_CMD_VEL;
        return 0;
    }
    if (strcmp(tok[0], "config") == 0) {
        if (n == 1) {
            req->cmd = CONSOLE_CMD_CONFIG;
            req->dump = 1;
            return 0;
        }
        if (n != 3) {
            snprintf(req->err, sizeof(req->err), "usage: config [NAME VALUE]");
            return -1;
        }
        if (!valid_param_name(tok[1])) {
            snprintf(req->err, sizeof(req->err), "invalid param name '%s'", tok[1]);
            return -1;
        }
        if (parse_long_token(tok[2], &req->value) != 0) {
            snprintf(req->err, sizeof(req->err), "invalid value '%s'", tok[2]);
            return -1;
        }
        req->cmd = CONSOLE_CMD_CONFIG;
        strcpy(req->name, tok[1]);
        return 0;
    }
    snprintf(req->err, sizeof(req->err), "unknown command '%s'", tok[0]);
    return -1;
}

/* ---- 单命令执行（控制台与一次性子命令共用） ---- */

/* 输入缓存保留同一次 read 中尚未执行的后续命令。 */
typedef struct {
    char chunk[256], line[CONSOLE_LINE_MAX];
    size_t pos, size, length;
    int eof, overflow, ready;
} console_input;
static console_input input;
static int read_line_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    (void)context;
    action->kind = CARTPOLE_TICK_OBSERVE;
    int filled = 0;
    while (1) {
        if (input.pos == input.size) {
            if (filled)
                return 0;
            filled = 1;
            fd_set rf;
            FD_ZERO(&rf);
            FD_SET(STDIN_FILENO, &rf);
            struct timeval tv = {0, 0};
            int rc = select(STDIN_FILENO + 1, &rf, NULL, NULL, &tv);
            if (rc < 0)
                return errno == EINTR ? 0 : -1;
            if (!rc)
                return 0;
            ssize_t n = read(STDIN_FILENO, input.chunk, sizeof input.chunk);
            if (n < 0)
                return errno == EINTR || errno == EAGAIN ? 0 : -1;
            if (!n) {
                input.eof = 1;
                action->kind = CARTPOLE_TICK_FINISH;
                return 0;
            }
            input.pos = 0;
            input.size = (size_t)n;
        }
        char c = input.chunk[input.pos++];
        if (c == '\n') {
            input.line[input.length] = 0;
            input.ready = 1;
            action->kind = CARTPOLE_TICK_FINISH;
            return 0;
        }
        if (c == '\r')
            continue;
        if (input.length + 1 < sizeof input.line)
            input.line[input.length++] = c;
        else
            input.overflow = 1;
    }
}
static void consume_line(void)
{
    input.length = 0;
    input.ready = 0;
    input.overflow = 0;
}
static int wait_enter(void)
{
    cartpole_result_t rc = cartpole_run_loop(.020, 0, read_line_tick, NULL);
    int ok = rc == CARTPOLE_OK && input.ready && !input.eof;
    consume_line();
    return ok ? 0 : -1;
}

/* 统一结果提示，不将拒绝、取消或故障统称为超时。 */
int console_transaction_result(const char *operation, cartpole_result_t result)
{
    if (result == CARTPOLE_OK)
        return 0;
    fprintf(stderr, "[edge] %s: %s%s\n", operation, cartpole_result_name(result),
            result == CARTPOLE_TIMEOUT ? " (execution unconfirmed; do not automatically retry)"
                                       : "");
    return -1;
}
static int query_mode_for_command(void)
{
    return console_transaction_result("QUERY", cartpole_query_mode(1.0));
}

int console_exec(const console_req_t *req)
{
    switch (req->cmd) {
    case CONSOLE_CMD_CALIB: {
        if (query_mode_for_command() != 0)
            return -1;
        int known;
        cartpole_session_t snapshot = cartpole_session();
        known = snapshot.is_mode_received;
        cartpole_mode_t m = snapshot.reported_mode;
        if (known && m != CARTPOLE_MODE_DISABLED) {
            fprintf(stderr, "[edge] calib requires DISABLED (current %s)\n", cartpole_mode_name(m));
            return -1;
        }
        printf("[edge] push cart to LEFT end, then press Enter...");
        fflush(stdout);
        if (wait_enter() != 0) {
            return -1;
        }
        if (console_transaction_result("MARK LEFT", cartpole_mark(CARTPOLE_LEFT, 1.0)) != 0) {
            return -1;
        }
        printf("[edge] push cart to RIGHT end, then press Enter...");
        fflush(stdout);
        if (wait_enter() != 0) {
            return -1;
        }
        if (console_transaction_result("MARK RIGHT", cartpole_mark(CARTPOLE_RIGHT, 1.0)) != 0) {
            return -1;
        }
        if (console_transaction_result("ENABLE", cartpole_enable(2.0)) != 0) {
            return -1;
        }
        printf("[edge] calibrated & READY; use 'center' to move to mid-track\n");
        return 0;
    }
    case CONSOLE_CMD_MARK_LEFT:
        return console_transaction_result("MARK LEFT", cartpole_mark(CARTPOLE_LEFT, 1.0));
    case CONSOLE_CMD_MARK_RIGHT:
        return console_transaction_result("MARK RIGHT", cartpole_mark(CARTPOLE_RIGHT, 1.0));
    case CONSOLE_CMD_ENABLE:
        return console_transaction_result("ENABLE", cartpole_enable(2.0));
    case CONSOLE_CMD_CENTER:
        if (console_goto_center(15.0) != 0) {
            fprintf(stderr, "[edge] move-to-center failed\n");
            return -1;
        }
        return 0;
    case CONSOLE_CMD_STATE:
        console_print_state();
        return 0;
    case CONSOLE_CMD_QUERY: {
        if (console_transaction_result("QUERY", cartpole_query_mode(1.0)) != 0) {
            return -1;
        }
        cartpole_reason_t reason = cartpole_session().reported_mode_reason;
        cartpole_mode_t m = cartpole_session().reported_mode;
        printf("MODE %s %s\n", cartpole_mode_name(m), cartpole_reason_name(reason));
        return 0;
    }
    case CONSOLE_CMD_CONFIG: {
        if (req->dump) {
            if (console_transaction_result("metadata",
                                           cartpole_refresh_configuration(DUMP_WINDOW_S)))
                return -1;
            cartpole_configuration_t c = cartpole_configuration();
            printf("[edge] calibration=%s length=%" PRId64 " um; VEL_MAX=%" PRId64
                   " ACC_MAX=%" PRId64 " WDT_MS=%" PRId64 " X_MARGIN_UM=%" PRId64 "\n",
                   c.calibration_state == CARTPOLE_CALIBRATION_READY ? "ready"
                   : c.calibration_state == CARTPOLE_CALIBRATION_REFERENCE_REQUIRED
                       ? "reference-required"
                       : "not-ready",
                   c.track_length_um, c.velocity_limit_mm_s, c.acceleration_limit_mm_s2,
                   c.watchdog_timeout_ms, c.track_margin_um);
            return 0;
        }
        char operation[32];
        snprintf(operation, sizeof operation, "CONFIG %s", req->name);
        cartpole_setting_t setting;
        if (cartpole_setting_from_name(req->name, &setting) != 0) {
            fprintf(stderr, "[edge] unsupported setting %s\n", req->name);
            return -1;
        }
        int rc =
            console_transaction_result(operation, cartpole_configure(setting, req->value, 1.0));
        if (rc == 0)
            printf("[edge] %s = %ld\n", cartpole_setting_name(setting), req->value);
        return rc;
    }
    case CONSOLE_CMD_RESET: {
        if (query_mode_for_command() != 0)
            return -1;
        int known;
        cartpole_session_t snapshot = cartpole_session();
        known = snapshot.is_mode_received;
        cartpole_mode_t m = snapshot.reported_mode;
        if (known && m == CARTPOLE_MODE_DISABLED) {
            printf("[edge] already DISABLED\n");
            return 0;
        }
        if (m != CARTPOLE_MODE_FAULT) {
            fprintf(stderr, "[edge] RESET only clears FAULT (current %s)\n", cartpole_mode_name(m));
            return -1;
        }
        if (console_transaction_result("RESET", cartpole_reset(2.0)) != 0)
            return -1;
        return 0;
    }
    default:
        return -1;
    }
}

/* ---- 交互循环 ---- */

static void print_help(void)
{
    printf("commands:\n"
           "  vel <mm/s>            set held target velocity (kept while READY & STATE fresh)\n"
           "  calib                 interactive 2-point calibration (MARK LEFT/RIGHT + ENABLE)\n"
           "  mark-left             send MARK LEFT (cart at left end)\n"
           "  mark-right            send MARK RIGHT (cart at right end)\n"
           "  enable                ENABLE after calibration, wait READY\n"
           "  center                move cart to track center\n"
           "  state                 print latest STATE snapshot\n"
           "  query                 ask firmware mode\n"
           "  config [NAME VALUE]   set a firmware param, or dump all with no args\n"
           "  reset                 clear FAULT\n"
           "  help | quit\n");
}

typedef struct {
    int holding;
    double velocity;
} console_hold;
static int console_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    console_hold *hold = context;
    int rc = read_line_tick(elapsed, action, NULL);
    if (rc || action->kind == CARTPOLE_TICK_FINISH)
        return rc;
    if (hold->holding) {
        action->kind = CARTPOLE_TICK_VELOCITY;
        action->velocity_mm_s = hold->velocity;
    }
    return 0;
}
int console_run(void)
{
    console_hold hold = {0};
    memset(&input, 0, sizeof input);
    print_help();
    puts("[edge] console ready (quit/Ctrl+D to exit)");
    while (!cartpole_stop_requested()) {
        cartpole_result_t result = cartpole_run_loop(.020, 0, console_tick, &hold);
        if (result == CARTPOLE_CANCELLED)
            return 0;
        if (result != CARTPOLE_OK)
            return console_transaction_result("console", result);
        if (input.eof)
            break;
        if (!input.ready)
            continue;
        console_req_t req;
        int parsed = input.overflow ? -1 : console_parse(input.line, &req);
        if (input.overflow)
            puts("? line too long");
        else if (parsed)
            printf("? %s\n", req.err);
        consume_line();
        if (parsed)
            continue;
        if (req.cmd == CONSOLE_CMD_QUIT)
            break;
        if (req.cmd == CONSOLE_CMD_NONE)
            continue;
        if (req.cmd == CONSOLE_CMD_HELP) {
            print_help();
            continue;
        }
        if (req.cmd == CONSOLE_CMD_VEL) {
            if (query_mode_for_command() ||
                console_transaction_result("STATE",
                                           cartpole_wait_measurement(CFG_STATE_TIMEOUT_S))) {
                hold.holding = 0;
                continue;
            }
            hold.holding = 1;
            hold.velocity = req.vel_mm_s;
            continue;
        }
        if (req.cmd != CONSOLE_CMD_QUERY && req.cmd != CONSOLE_CMD_STATE)
            hold.holding = 0;
        (void)console_exec(&req);
    }
    return 0;
}
int console_goto_center(double timeout_s)
{
    return console_transaction_result("center", cartpole_move_to_center(timeout_s));
}
void console_print_state(void)
{
    cartpole_measurement_t m = cartpole_measurement();
    if (m.received_host_monotonic_s < 0) {
        puts("[edge] no STATE received");
        return;
    }
    if (m.cart_valid)
        printf("[edge] cart x=%.6f m velocity=%.6f m/s\n", m.cart_position_m, m.cart_velocity_m_s);
    else
        puts("[edge] cart measurement unavailable");
    if (m.pole_valid)
        printf("[edge] pole angle=%.6f rad velocity=%.6f rad/s\n", m.pole_angle_rad,
               m.pole_angular_velocity_rad_s);
    else
        puts("[edge] pole measurement unavailable");
    printf("[edge] measurement age=%.1f ms\n", cartpole_measurement_age_s(m) * 1000);
}
