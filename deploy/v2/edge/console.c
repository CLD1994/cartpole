/* 交互控制台实现。协议契约见 deploy/v2/protocol.md。 */

#include "console.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "rig.h"

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
        req->cmd = strcmp(tok[0], "calib") == 0          ? CONSOLE_CMD_CALIB
                   : strcmp(tok[0], "mark-left") == 0    ? CONSOLE_CMD_MARK_LEFT
                   : strcmp(tok[0], "mark-right") == 0   ? CONSOLE_CMD_MARK_RIGHT
                   : strcmp(tok[0], "enable") == 0       ? CONSOLE_CMD_ENABLE
                   : strcmp(tok[0], "center") == 0       ? CONSOLE_CMD_CENTER
                   : strcmp(tok[0], "state") == 0        ? CONSOLE_CMD_STATE
                   : strcmp(tok[0], "query") == 0        ? CONSOLE_CMD_QUERY
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

static void nap_ms(int ms)
{
    struct timespec ts = {0, (long)ms * 1000 * 1000};
    nanosleep(&ts, NULL);
}

/* 等操作者推车到位后按回车（calib 交互引导用）。 */
static int wait_enter(void)
{
    int c = getchar();
    while (c != '\n' && c != EOF) {
        c = getchar();
    }
    return c == EOF ? -1 : 0;
}

/* 发命令并等 OK/ERR 回执（回执名经控制台回显核对）。 */
static int send_and_await(const char *cmd)
{
    if (rig_send_simple(cmd) != 0) {
        return -1;
    }
    if (rig_wait_reply(1.0) != 0) {
        fprintf(stderr, "[edge] %s rejected/failed\n", cmd);
        return -1;
    }
    return 0;
}

/* 模式未知时先 QUERY 同步（一次性命令是新进程、控制台首条命令同理）。
 * 返回 0 表示模式已知，-1 表示 QUERY 超时。 */
static int ensure_mode_known(void)
{
    int known;
    rig_mode(&known, NULL);
    if (known) {
        return 0;
    }
    return rig_sync_mode(1.0) == 0 ? 0 : -1;
}

int console_exec(const console_req_t *req)
{
    switch (req->cmd) {
    case CONSOLE_CMD_CALIB: {
        /* 同步失败时保持原行为：前置检查跳过，由固件对 MARK 的拒绝兜底 */
        (void)ensure_mode_known();
        int known;
        fw_mode_t m = rig_mode(&known, NULL);
        if (known && m != FW_DISABLED) {
            fprintf(stderr, "[edge] calib requires DISABLED (current %s)\n",
                    proto_mode_name(m));
            return -1;
        }
        printf("[edge] push cart to LEFT end, then press Enter...");
        fflush(stdout);
        if (wait_enter() != 0) {
            return -1;
        }
        if (send_and_await("MARK LEFT") != 0) {
            fprintf(stderr, "[edge] MARK LEFT rejected/failed\n");
            return -1;
        }
        printf("[edge] push cart to RIGHT end, then press Enter...");
        fflush(stdout);
        if (wait_enter() != 0) {
            return -1;
        }
        if (send_and_await("MARK RIGHT") != 0) {
            fprintf(stderr, "[edge] MARK RIGHT rejected (L invalid? no left mark?)\n");
            return -1;
        }
        if (rig_send_simple("ENABLE") != 0) {
            return -1;
        }
        if (rig_wait_mode(FW_READY, 2.0) != 0) {
            fprintf(stderr, "[edge] ENABLE failed (see MODE events)\n");
            return -1;
        }
        printf("[edge] calibrated & READY; use 'center' to move to mid-track\n");
        return 0;
    }
    case CONSOLE_CMD_MARK_LEFT:
        return send_and_await("MARK LEFT");
    case CONSOLE_CMD_MARK_RIGHT:
        return send_and_await("MARK RIGHT");
    case CONSOLE_CMD_ENABLE:
        return send_and_await("ENABLE");
    case CONSOLE_CMD_CENTER:
        if (rig_goto_center(15.0) != 0) {
            fprintf(stderr, "[edge] move-to-center failed\n");
            return -1;
        }
        return 0;
    case CONSOLE_CMD_STATE:
        rig_print_state();
        return 0;
    case CONSOLE_CMD_QUERY: {
        if (rig_sync_mode(1.0) != 0) {
            fprintf(stderr, "[edge] no MODE reply (QUERY timed out)\n");
            return -1;
        }
        int known;
        fw_reason_t reason;
        fw_mode_t m = rig_mode(&known, &reason);
        printf("MODE %s %s\n", proto_mode_name(m), proto_reason_name(reason));
        return 0;
    }
    case CONSOLE_CMD_CONFIG:
        if (req->dump) {
            if (rig_send_simple("CONFIG?") != 0) {
                return -1;
            }
            double t0 = rig_now_s();
            while (rig_now_s() - t0 < DUMP_WINDOW_S) {
                rig_pump();
                nap_ms(5);
            }
            return 0;
        }
        if (rig_send_config(req->name, req->value) != 0) {
            return -1;
        }
        return rig_wait_reply(1.0) == 0 ? 0 : -1;
    case CONSOLE_CMD_RESET: {
        if (ensure_mode_known() != 0) {
            fprintf(stderr, "[edge] no MODE reply (QUERY timed out)\n");
            return -1;
        }
        int known;
        fw_mode_t m = rig_mode(&known, NULL);
        if (known && m == FW_DISABLED) {
            printf("[edge] already DISABLED\n");
            return 0;
        }
        if (m != FW_FAULT) {
            fprintf(stderr, "[edge] RESET only clears FAULT (current %s)\n",
                    proto_mode_name(m));
            return -1;
        }
        if (rig_send_simple("RESET") != 0) {
            return -1;
        }
        if (rig_wait_mode(FW_DISABLED, 2.0) != 0) {
            fprintf(stderr, "[edge] no DISABLED event after RESET\n");
            return -1;
        }
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

/* 处理一行输入。返回 1 表示退出控制台。 */
static int handle_line(const char *line, int *holding, double *held_vel)
{
    console_req_t req;
    if (console_parse(line, &req) != 0) {
        printf("? %s\n", req.err);
        return 0;
    }
    switch (req.cmd) {
    case CONSOLE_CMD_QUIT:
        return 1;
    case CONSOLE_CMD_HELP:
        print_help();
        return 0;
    case CONSOLE_CMD_NONE:
        return 0;
    case CONSOLE_CMD_VEL:
        /* 保持前先等 STATE 流就位、模式已知（与一次性 vel 子命令同语义）：
         * tick_hold 对未知模式会立即释放保持，首条命令必须先 QUERY 同步 */
        if (rig_wait_state(CFG_STATE_TIMEOUT_S) != 0) {
            printf("[edge] WARNING: no STATE stream, ignoring vel\n");
            return 0;
        }
        if (ensure_mode_known() != 0) {
            printf("[edge] WARNING: no MODE reply, ignoring vel\n");
            return 0;
        }
        *holding = 1;
        *held_vel = req.vel_mm_s;
        return 0;
    case CONSOLE_CMD_CALIB:
    case CONSOLE_CMD_MARK_LEFT:
    case CONSOLE_CMD_MARK_RIGHT:
    case CONSOLE_CMD_ENABLE:
    case CONSOLE_CMD_CENTER:
    case CONSOLE_CMD_RESET:
    case CONSOLE_CMD_CONFIG:
        /* 模式变更类命令期间不保持速度，避免标定/使能/清障后意外续动 */
        *holding = 0;
        console_exec(&req);
        return 0;
    default: /* QUERY / STATE 只读，不打断保持 */
        console_exec(&req);
        return 0;
    }
}

/* 保持心跳：检查模式与 STATE 新鲜度并重发目标速度（喂固件看门狗）。
 * 语义不依赖输入节奏：每处理完一行输入、每个 select 超时各调用一次。 */
static void tick_hold(int *holding, double held_vel)
{
    if (!*holding) {
        return;
    }
    int known;
    fw_mode_t m = rig_mode(&known, NULL);
    double age = rig_state_age_s();
    if (!known || m != FW_READY) {
        *holding = 0;
        printf("[edge] hold released (firmware in %s)\n",
               known ? proto_mode_name(m) : "unknown mode");
    } else if (age < 0.0 || age > CFG_STATE_TIMEOUT_S) {
        *holding = 0;
        rig_send_vel(0.0);
        printf("[edge] WARNING: STATE stream stale, motor stopped\n");
    } else {
        rig_send_vel(held_vel);
    }
}

int console_run(void)
{
    char line[CONSOLE_LINE_MAX];
    size_t len = 0;
    int holding = 0;
    double held_vel = 0.0;

    print_help();
    printf("[edge] console ready (quit/Ctrl+D to exit)\n");
    for (;;) {
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(STDIN_FILENO, &rf);
        struct timeval tv = {0, 20 * 1000};
        int r = select(STDIN_FILENO + 1, &rf, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) {
                r = 0; /* 信号打断：走下面的停止与保持检查 */
            } else {
                fprintf(stderr, "[edge] select: %s\n", strerror(errno));
                break;
            }
        }
        if (r > 0) {
            char chunk[64];
            ssize_t got = read(STDIN_FILENO, chunk, sizeof(chunk));
            if (got < 0 && errno == EINTR) {
                got = 0;
            }
            if (got < 0) {
                fprintf(stderr, "[edge] read: %s\n", strerror(errno));
                break;
            }
            if (got == 0) { /* Ctrl+D */
                break;
            }
            for (ssize_t k = 0; k < got; k++) {
                char c = chunk[k];
                if (c == '\n') {
                    line[len] = '\0';
                    if (handle_line(line, &holding, &held_vel)) {
                        goto out;
                    }
                    len = 0;
                    tick_hold(&holding, held_vel);
                } else if (len + 1 < sizeof(line)) {
                    line[len++] = c;
                } /* 超长输入截断，与协议接收器同策略 */
            }
        }

        rig_pump();
        tick_hold(&holding, held_vel);

        if (rig_stop_requested()) {
            break;
        }
    }
out:
    return 0; /* 停机由 main 的会话退出路径统一发送 */
}
