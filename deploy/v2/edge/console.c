#include "cartpole_format.h"
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
#include "cartpole_control.h"
#include "cartpole_pole_calibration.h"
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

static int normalize_param_name(char *s)
{
    size_t n = strlen(s);
    if (n == 0 || n >= 16) { /* 与 console_req_t.name 同容量，含 NUL */
        return 0;
    }
    for (size_t i = 0; i < n; i++) {
        if (s[i] >= 'a' && s[i] <= 'z')
            s[i] = (char)(s[i] - 'a' + 'A');
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
    if (strcmp(tok[0], "enable") == 0) {
        req->timeout_s = 5.0;
        if (n != 1 && (n != 3 || strcmp(tok[1], "--timeout") != 0 ||
                       parse_double_token(tok[2], &req->timeout_s) != 0 ||
                       req->timeout_s <= 0)) {
            snprintf(req->err, sizeof req->err, "usage: enable [--timeout SECONDS]");
            return -1;
        }
        req->cmd = CONSOLE_CMD_ENABLE;
        return 0;
    }
    static const struct {
        const char *name;
        console_cmd_t command;
    } no_args[] = {
        {"calib", CONSOLE_CMD_CALIB}, {"mark-left", CONSOLE_CMD_MARK_LEFT},
        {"mark-right", CONSOLE_CMD_MARK_RIGHT}, {"confirm-left", CONSOLE_CMD_CONFIRM_LEFT},
        {"calib-status", CONSOLE_CMD_CALIB_STATUS},
        {"calibrate-pole-down", CONSOLE_CMD_CALIBRATE_POLE_DOWN},
        {"disable", CONSOLE_CMD_DISABLE}, {"center", CONSOLE_CMD_CENTER},
        {"state", CONSOLE_CMD_STATE}, {"query", CONSOLE_CMD_QUERY}, {"reset", CONSOLE_CMD_RESET},
    };
    for (size_t i = 0; i < sizeof no_args / sizeof no_args[0]; ++i) {
        if (strcmp(tok[0], no_args[i].name) == 0) {
            if (n != 1) {
                snprintf(req->err, sizeof req->err, "%s takes no arguments", tok[0]);
                return -1;
            }
            req->cmd = no_args[i].command;
            return 0;
        }
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
    if (strcmp(tok[0], "dump") == 0) {
        if (n != 1) {
            snprintf(req->err, sizeof(req->err), "dump takes no arguments");
            return -1;
        }
        req->cmd = CONSOLE_CMD_CONFIG;
        req->dump = 1;
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
        if (!normalize_param_name(tok[1])) {
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
static int wait_enter(cartpole_t *c)
{
    cartpole_result_t rc = cartpole_run_loop(c, .020, 0, read_line_tick, NULL);
    int ok = rc == CARTPOLE_OK && input.ready && !input.eof;
    consume_line();
    return ok ? 0 : -1;
}

/* 统一结果提示，不将拒绝、取消或故障统称为超时。 */
int console_transaction_result(const char *operation, cartpole_result_t result)
{
    if (result == CARTPOLE_OK)
        return 0;
    const char *detail = "";
    switch (result) {
    case CARTPOLE_TIMEOUT:
        detail = "（执行未确认，不要自动重试）";
        break;
    case CARTPOLE_MEASUREMENT_UNAVAILABLE:
        detail = "（未收到所需的新鲜有效测量）";
        break;
    case CARTPOLE_POWER_UNAVAILABLE:
        detail = "（固件未确认供电就绪，检查 20 V 供电）";
        break;
    case CARTPOLE_BUSY:
        detail = "（固件未确认小车停稳；先请求零速、观察停稳，再禁用）";
        break;
    case CARTPOLE_FAULT:
        detail = "（固件处于故障，故障状态保留）";
        break;
    case CARTPOLE_CALIBRATION_REFERENCE_ERROR:
        detail = "（标定参考点无效，先用 calib-status 查看状态，再恢复参考点或重做左右端标定）";
        break;
    case CARTPOLE_STORAGE_ERROR:
        detail = "（固件未确认保存成功）";
        break;
    default:
        break;
    }
    fprintf(stderr, "[ERR] %s: %s%s\n", operation, cartpole_result_to_string(result), detail);
    return -1;
}
static int query_mode_for_command(cartpole_t *c)
{
    return console_transaction_result("QUERY", cartpole_query_mode(c, 1.0));
}

static int enable_checked(cartpole_t *c, double timeout)
{
    const double end = cartpole_get_monotonic_time_s(c) + timeout;
    const uint64_t previous_faults = cartpole_get_session_state(c).received_fault_mode_count;
    if (console_transaction_result("ENABLE/QUERY", cartpole_query_mode(c, timeout)))
        return -1;
    cartpole_session_t session = cartpole_get_session_state(c);
    if (session.reported_mode == CARTPOLE_MODE_FAULT ||
        session.received_fault_mode_count != previous_faults)
        return console_transaction_result("ENABLE", CARTPOLE_FAULT);
    if (!session.connection_open || session.has_stream_failure_latch)
        return console_transaction_result("ENABLE", CARTPOLE_SESSION_UNAVAILABLE);
    if (session.is_mode_received && session.reported_mode == CARTPOLE_MODE_READY &&
        session.reported_mode_reason == CARTPOLE_REASON_NONE) {
        puts("[OK] ENABLE：已处于 READY，无需再次使能");
        return 0;
    }
    if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_DISABLED ||
        session.reported_mode_reason != CARTPOLE_REASON_NONE)
        return console_transaction_result("ENABLE", CARTPOLE_REJECTED);
    const double remaining = end - cartpole_get_monotonic_time_s(c);
    if (remaining <= 0)
        return console_transaction_result("ENABLE/STATE", CARTPOLE_MEASUREMENT_UNAVAILABLE);
    /* 适配器在每次真正写入前确认模式、双路测量和接收时效。 */
    const cartpole_result_t result = cartpole_enable(c, remaining);
    if (console_transaction_result(result == CARTPOLE_MEASUREMENT_UNAVAILABLE
                                       ? "ENABLE/STATE" : "ENABLE", result))
        return -1;
    puts("[OK] ENABLE：固件已确认进入 READY");
    return 0;
}

static int mark_endpoint(cartpole_t *c, cartpole_endpoint_t endpoint)
{
    int64_t value_um;
    const char *command = endpoint == CARTPOLE_LEFT ? "MARK LEFT" : "MARK RIGHT";
    if (console_transaction_result(command, cartpole_mark_endpoint(c, endpoint, &value_um, 1.0)))
        return -1;
    printf("[OK] %s = %" PRId64 " um\n", endpoint == CARTPOLE_LEFT ? "MARK_LEFT" : "MARK_RIGHT",
           value_um);
    return 0;
}

int console_exec(cartpole_t *c, const console_req_t *req)
{
    switch (req->cmd) {
    case CONSOLE_CMD_CALIB: {
        if (query_mode_for_command(c) != 0)
            return -1;
        int known;
        cartpole_session_t snapshot = cartpole_get_session_state(c);
        known = snapshot.is_mode_received;
        cartpole_mode_t m = snapshot.reported_mode;
        if (known && m != CARTPOLE_MODE_DISABLED) {
            fprintf(stderr, "[edge] calib requires DISABLED (current %s)\n", cartpole_mode_to_string(m));
            return -1;
        }
        printf("[edge] push cart to LEFT end, then press Enter...");
        fflush(stdout);
        if (wait_enter(c) != 0) {
            return -1;
        }
        if (mark_endpoint(c, CARTPOLE_LEFT) != 0) {
            return -1;
        }
        printf("[edge] push cart to RIGHT end, then press Enter...");
        fflush(stdout);
        if (wait_enter(c) != 0) {
            return -1;
        }
        if (mark_endpoint(c, CARTPOLE_RIGHT) != 0) {
            return -1;
        }
        if (enable_checked(c, 2.0) != 0) {
            return -1;
        }
        printf("[edge] calibrated & READY; use 'center' to move to mid-track\n");
        return 0;
    }
    case CONSOLE_CMD_MARK_LEFT:
        return mark_endpoint(c, CARTPOLE_LEFT);
    case CONSOLE_CMD_MARK_RIGHT:
        return mark_endpoint(c, CARTPOLE_RIGHT);
    case CONSOLE_CMD_CONFIRM_LEFT: {
        int64_t position_um;
        if (console_transaction_result("CONFIRM LEFT", cartpole_confirm_left(c, &position_um, 1.0)))
            return -1;
        printf("[OK] CONFIRM_LEFT = %" PRId64 " um（仍处于 DISABLED）\n", position_um);
        return 0;
    }
    case CONSOLE_CMD_CALIB_STATUS: {
        cartpole_calibration_status_t status;
        if (console_transaction_result("CALIB?", cartpole_read_calibration_status(c, &status, 1.0)))
            return -1;
        printf("[OK] CAL_STATE = %" PRId64 "\n[OK] CAL_LENGTH_UM = %" PRId64
               "\n[OK] CAL_SAVED = %" PRId64 "\n[OK] CAL_STORAGE = %" PRId64 "\n",
               status.state, status.length_um, status.saved, status.storage);
        return 0;
    }
    case CONSOLE_CMD_CALIBRATE_POLE_DOWN: {
        puts("请让摆杆自然下垂并松手，保持静止；仅在 DISABLED 下校准。");
        fflush(stdout);
        cartpole_pole_calibration_t result = {0};
        int rc = console_transaction_result("calibrate-pole-down",
                                           cartpole_calibrate_pole_down(c, &result));
        if (rc != 0) {
            const char *detail = cartpole_pole_calibration_detail_string(result.detail);
            if (detail)
                fprintf(stderr, "[ERR] %s\n", detail);
            if (result.write_confirmed)
                fputs("[ERR] 新零点已写入并读回，但下垂角验证未通过；未自动回滚。\n", stderr);
            else if (result.write_attempted)
                fputs("[ERR] 零点写入未确认；先用 dump 核对，不要直接重试。\n", stderr);
            return rc;
        }
        printf("[OK] POLE_ZERO_URAD: %" PRId64 " -> %" PRId64
               "（已读回）；POLE_INV = %d 不变\n",
               result.old_zero_urad, result.new_zero_urad, result.pole_inverted);
        printf("[OK] 下垂角验收通过：%.3f rad\n", result.verified_down_angle_rad);
        return 0;
    }
    case CONSOLE_CMD_ENABLE:
        return enable_checked(c, req->timeout_s);
    case CONSOLE_CMD_DISABLE:
        return console_transaction_result("DISABLE", cartpole_disable(c, 1.0));
    case CONSOLE_CMD_CENTER:
        if (console_goto_center(c, 15.0) != 0) {
            fprintf(stderr, "[edge] move-to-center failed\n");
            return -1;
        }
        return 0;
    case CONSOLE_CMD_STATE:
        console_print_state(c);
        return 0;
    case CONSOLE_CMD_QUERY:
        return query_mode_for_command(c); /* 统一的会话事件负责输出 MODE。 */
    case CONSOLE_CMD_CONFIG: {
        if (req->dump) {
            cartpole_settings_t settings;
            uint32_t received_mask;
            cartpole_result_t result =
                cartpole_read_settings_observed(c, &settings, &received_mask, DUMP_WINDOW_S);
            if (result != CARTPOLE_OK) {
                if (result == CARTPOLE_TIMEOUT) {
                    fputs("[ERR] CONFIG?: TIMEOUT（缺少回执字段：", stderr);
                    int shown = 0;
                    for (int i = 0; i < CARTPOLE_SETTING_COUNT; i++) {
                        if (!(received_mask & (UINT32_C(1) << i))) {
                            fprintf(stderr, "%s%s", shown++ ? "、" : "",
                                    cartpole_setting_to_string(i));
                        }
                    }
                    if (!shown)
                        fputs("无，收到完整回执时已超出期限", stderr);
                    fputs("）\n", stderr);
                    return -1;
                }
                return console_transaction_result("CONFIG?", result);
            }
            for (int i = 0; i < CARTPOLE_SETTING_COUNT; i++)
                printf("[OK] %s = %" PRId64 "\n", cartpole_setting_to_string(i), settings.values[i]);
            return 0;
        }
        char operation[32];
        snprintf(operation, sizeof operation, "CONFIG %s", req->name);
        cartpole_setting_t setting;
        if (cartpole_parse_setting(req->name, &setting) != 0) {
            fprintf(stderr, "[ERR] CONFIG：不支持字段 %s\n", req->name);
            return -1;
        }
        int rc =
            console_transaction_result(operation, cartpole_configure(c, setting, req->value, 1.0));
        if (rc == 0)
            printf("[OK] %s = %ld\n", cartpole_setting_to_string(setting), req->value);
        return rc;
    }
    case CONSOLE_CMD_RESET: {
        if (query_mode_for_command(c) != 0)
            return -1;
        int known;
        cartpole_session_t snapshot = cartpole_get_session_state(c);
        known = snapshot.is_mode_received;
        cartpole_mode_t m = snapshot.reported_mode;
        if (known && m == CARTPOLE_MODE_DISABLED) {
            puts("[OK] RESET：已处于 DISABLED，无需复位");
            return 0;
        }
        if (m != CARTPOLE_MODE_FAULT) {
            fprintf(stderr, "[ERR] RESET：仅可清除 FAULT（当前 %s）\n", cartpole_mode_to_string(m));
            return -1;
        }
        if (console_transaction_result("RESET", cartpole_reset(c, 2.0)) != 0)
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
           "  vel <mm/s>            set held target velocity (kept while READY & "
           "STATE fresh)\n"
           "  calib                 interactive 2-point calibration (MARK "
           "LEFT/RIGHT + ENABLE)\n"
           "  mark-left             send MARK LEFT (cart at left end)\n"
           "  mark-right            send MARK RIGHT (cart at right end)\n"
           "  confirm-left          restore the saved left reference, without enabling\n"
           "  calib-status          read track calibration status\n"
           "  calibrate-pole-down   calibrate the stationary, hanging pole (DISABLED)\n"
           "  enable [--timeout s]  ENABLE after calibration, wait READY\n"
           "  disable               disable only when the firmware is stopped\n"
           "  center                move cart to track center\n"
           "  state                 print latest STATE snapshot\n"
           "  query                 ask firmware mode\n"
           "  dump                  read all firmware settings\n"
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
int console_run(cartpole_t *c)
{
    console_hold hold = {0};
    memset(&input, 0, sizeof input);
    print_help();
    puts("[edge] console ready (quit/Ctrl+D to exit)");
    while (!cartpole_is_stop_requested(c)) {
        cartpole_result_t result = cartpole_run_loop(c, .020, 0, console_tick, &hold);
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
            if (query_mode_for_command(c) ||
                console_transaction_result("STATE",
                                           cartpole_wait_for_measurement(c, CFG_STATE_TIMEOUT_S))) {
                hold.holding = 0;
                continue;
            }
            hold.holding = 1;
            hold.velocity = req.vel_mm_s;
            continue;
        }
        if (req.cmd != CONSOLE_CMD_QUERY && req.cmd != CONSOLE_CMD_STATE)
            hold.holding = 0;
        (void)console_exec(c, &req);
    }
    return 0;
}
int console_goto_center(cartpole_t *c, double timeout_s)
{
    return console_transaction_result("center", cartpole_move_to_center(c, timeout_s));
}
void console_print_state(cartpole_t *c)
{
    cartpole_measurement_t m = cartpole_get_latest_measurement(c);
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
    printf("[edge] measurement age=%.1f ms\n", cartpole_calculate_measurement_age_s(c, m) * 1000);
}
