/* 端侧控制程序（新装置 v2）：命令行入口与三种运行形态。
 *
 *   run                        策略模式：200 Hz 读遥测 -> 组观测 -> NPU 推理 -> 发速度命令。
 *   （无子命令）/ console      交互控制台（维护与板端联调，不接触 NPU）。
 *   home/center/vel/state/query/config/reset    一次性子命令。
 *
 * 会话（串口、遥测、模式、运动流程）见 rig.c；控制台命令见 console.c。
 * 职责与安全语义（与 deploy/v2/protocol.md、sim2real-spec.md 对齐）：
 *   - 驱动门控：仅当最后已知固件模式为 READY 且 STATE 流新鲜（< CFG_STATE_TIMEOUT_S）时驱动；
 *   - 摆杆门限：|theta| 超过 CFG_ENGAGE_ANGLE_RAD 时不驱动（人手扶到直立附近才接管）；
 *   - 轨道端头：|x| 超过 CFG_TRACK_LIMIT_M 时只封锁向外的速度，保留回程；
 *   - 退出路径统一：SIGINT/SIGTERM/致命错误 -> VEL 0 -> 释放资源。
 *   固件侧另有独立的看门狗与软限位兜底（见协议）。
 */

#include "config.h"
#include "console.h"
#include "policy.h"
#include "proto.h"
#include "rig.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void print_usage(const char *prog);

static void on_signal(int sig)
{
    (void)sig;
    rig_request_stop();
}

static void nap_ms(int ms)
{
    struct timespec ts = {0, (long)ms * 1000 * 1000};
    nanosleep(&ts, NULL);
}

static void sleep_to(double next)
{
    struct timespec ts;
    ts.tv_sec = (time_t)next;
    ts.tv_nsec = (long)((next - (double)ts.tv_sec) * 1e9);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

/* ---- 策略模式（run） ---- */

/* 返回 0：SIGINT/SIGTERM 正常退出；返回 -1：致命错误（推理句柄失效），
 * 调用方走统一清理路径（VEL 0 -> 释放资源）。 */
static int control_loop(void)
{
    const double period = 1.0 / CFG_CONTROL_HZ;
    double next = rig_now_s() + period;
    int state_stale_logged = 0;
    int limit_logged = 0;

    printf("[edge] policy loop at %.0f Hz. Ctrl+C to stop.\n", CFG_CONTROL_HZ);
    while (!rig_stop_requested()) {
        rig_pump();

        int64_t x_um, theta_urad, vx_mm_s, vth_mrad_s;
        rig_telemetry(&x_um, &theta_urad, &vx_mm_s, &vth_mrad_s);
        double x_m = (double)x_um * 1e-6;
        double th_rad = (double)theta_urad * 1e-6;

        /* 门控 1：固件必须处于 READY（否则固件本就不执行 VEL，这里只是不发并提示一次） */
        int mode_known;
        fw_mode_t mode = rig_mode(&mode_known, NULL);
        if (!mode_known || mode != FW_READY) {
            state_stale_logged = 0;
            sleep_to(next);
            next += period;
            continue;
        }
        /* 门控 2：STATE 流必须新鲜 */
        double state_age = rig_state_age_s();
        if (state_age < 0.0 || state_age > CFG_STATE_TIMEOUT_S) {
            rig_send_vel(0.0);
            if (!state_stale_logged) {
                state_stale_logged = 1;
                printf("[edge] WARNING: STATE stream stale, motor stopped\n");
            }
            sleep_to(next);
            next += period;
            continue;
        }
        state_stale_logged = 0;

        /* 门控 3：摆杆在接管角以内（人手扶到直立附近），否则不驱动 */
        if (fabs(th_rad) > CFG_ENGAGE_ANGLE_RAD) {
            rig_send_vel(0.0);
            sleep_to(next);
            next += period;
            continue;
        }

        /* 观测（顺序/单位与训练一致）与推理 */
        float obs[4] = {
            (float)x_m,
            (float)th_rad,
            (float)((double)vx_mm_s * 1e-3),
            (float)((double)vth_mrad_s * 1e-3),
        };
        float action;
        if (policy_forward(obs, &action) != 0) {
            /* 库契约：句柄已失效，不得继续推理；停机走统一退出路径 */
            fprintf(stderr, "[edge] policy inference failed, stopping\n");
            return -1;
        }
        if (action > 1.0f) {
            action = 1.0f;
        }
        if (action < -1.0f) {
            action = -1.0f;
        }
        double v_mm_s = (double)action * CFG_V_MAX_MPS * 1000.0;

        /* 轨道端头：只封向外，保留向内纠偏 */
        if (fabs(x_m) > CFG_TRACK_LIMIT_M && (x_m > 0.0) == (v_mm_s > 0.0)) {
            v_mm_s = 0.0;
            if (!limit_logged) {
                limit_logged = 1;
                printf("[edge] WARNING: cart at track end (%.3f m), blocking outward motion\n", x_m);
            }
        } else {
            limit_logged = 0;
        }

        rig_send_vel(v_mm_s);
        sleep_to(next);
        next += period;
    }
    return 0;
}

static int run_policy(int argc, char **argv, int first)
{
    const char *model = NULL;

    for (int k = first; k < argc; k++) {
        if (strcmp(argv[k], "--model") == 0 && k + 1 < argc) {
            model = argv[++k];
        } else {
            fprintf(stderr, "[edge] unknown run argument: %s\n", argv[k]);
            return 1;
        }
    }
    if (model == NULL) {
        fprintf(stderr, "[edge] run requires --model <path.om>\n");
        return 1;
    }

    if (policy_init(model) != 0) {
        fprintf(stderr, "[edge] policy init failed\n");
        return 1;
    }
    if (rig_sync_mode(1.0) != 0) {
        fprintf(stderr, "[edge] no MODE reply (QUERY timed out) - firmware not running?\n");
        return 1;
    }
    int known;
    fw_mode_t mode = rig_mode(&known, NULL);
    printf("[edge] firmware mode: %s\n", proto_mode_name(mode));

    if (rig_push_params() != 0) {
        fprintf(stderr, "[edge] config push failed (serial write error)\n");
        return 1;
    }

    if (mode != FW_READY) {
        fprintf(stderr, "[edge] firmware in %s; calibrate first "
                "(console 'calib'), or RESET a FAULT\n",
                proto_mode_name(mode));
        return 1;
    }
    if (rig_goto_center(15.0) != 0) {
        fprintf(stderr, "[edge] move-to-center failed\n");
        return 1;
    }
    printf("[edge] ready at track center; lift the pole near upright to engage the policy\n");

    return control_loop() == 0 ? 0 : 1;
}

/* ---- 控制台（默认形态） ---- */

static int run_console(void)
{
    if (rig_sync_mode(1.0) != 0) {
        fprintf(stderr, "[edge] no MODE reply (QUERY timed out) - firmware not running?\n");
        return 1;
    }
    int known;
    fw_mode_t mode = rig_mode(&known, NULL);
    printf("[edge] firmware mode: %s\n", proto_mode_name(mode));

    if (rig_push_params() != 0) {
        fprintf(stderr, "[edge] config push failed (serial write error)\n");
        return 1;
    }
    return console_run();
}

/* ---- 一次性子命令 ---- */

static int oneshot_state(void)
{
    if (rig_wait_state(1.0) != 0) {
        fprintf(stderr, "[edge] no STATE stream\n");
        return 1;
    }
    rig_print_state();
    return 0;
}

static int oneshot_vel(double v_mm_s, double hold_s)
{
    if (rig_sync_mode(1.0) != 0) {
        fprintf(stderr, "[edge] no MODE reply (QUERY timed out) - firmware not running?\n");
        return 1;
    }
    int known;
    fw_mode_t mode = rig_mode(&known, NULL);
    if (!known || mode != FW_READY) {
        fprintf(stderr, "[edge] VEL requires READY (current %s)\n",
                known ? proto_mode_name(mode) : "unknown");
        return 1;
    }
    /* 保持前先等到 STATE 流就位：从未收到与已过期是不同状态，不能把
     * 起始竞态当成流过期立即中止。 */
    if (rig_wait_state(CFG_STATE_TIMEOUT_S) != 0) {
        fprintf(stderr, "[edge] no STATE stream\n");
        return 1;
    }
    if (rig_send_vel(v_mm_s) != 0) {
        fprintf(stderr, "[edge] invalid velocity value\n");
        return 1;
    }
    double t0 = rig_now_s();
    int rc = 0;
    while (!rig_stop_requested() && rig_now_s() - t0 < hold_s) {
        rig_pump();
        mode = rig_mode(&known, NULL);
        if (known && mode != FW_READY) {
            fprintf(stderr, "[edge] firmware left READY during hold\n");
            rc = 1;
            break;
        }
        double age = rig_state_age_s();
        if (age < 0.0 || age > CFG_STATE_TIMEOUT_S) {
            fprintf(stderr, "[edge] WARNING: STATE stream stale during hold\n");
            rc = 1;
            break;
        }
        rig_send_vel(v_mm_s); /* 20 ms 重发，喂固件看门狗（WDT_MS 100） */
        nap_ms(20);
    }
    rig_send_vel(0.0);
    return rc;
}

/* 把 argv 参数重新拼成控制台命令行走 console_parse 校验（校验单源）。 */
static int parse_argv_as_console(const char *cmd, const char *a1, const char *a2,
                                 console_req_t *req)
{
    char line[128];
    int n = snprintf(line, sizeof(line), "%s %s %s", cmd,
                     a1 != NULL ? a1 : "", a2 != NULL ? a2 : "");
    if (n < 0 || (size_t)n >= sizeof(line)) {
        return -1;
    }
    return console_parse(line, req);
}

static int parse_duration(const char *s, double *out)
{
    char *end;
    double v = strtod(s, &end);
    if (end == s || *end != '\0' || !isfinite(v) || v < 0.0) {
        return -1;
    }
    *out = v;
    return 0;
}

/* ---- 命令行 ---- */

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [--port /dev/ttyXXXX] [command [args]]\n"
            "  (no command)                     interactive console (default)\n"
            "  console                          same as above\n"
            "  run --model path.om              policy control (model required; firmware must be READY)\n"
            "  calib                            one-shot: interactive 2-point calibration (MARK LEFT/RIGHT + ENABLE)\n"
            "  mark-left                        one-shot: send MARK LEFT (cart at left end)\n"
            "  mark-right                       one-shot: send MARK RIGHT (cart at right end)\n"
            "  enable                           one-shot: ENABLE after calibration, wait READY\n"
            "  center                           one-shot: move cart to track center (needs READY)\n"
            "  vel <mm/s> [--hold s]            one-shot: hold velocity for s seconds (default 1.0)\n"
            "  state                            one-shot: print latest STATE snapshot\n"
            "  query                            one-shot: print firmware mode\n"
            "  config [NAME VALUE]              one-shot: set one param, or dump all with no args\n"
            "  reset                            one-shot: clear FAULT\n",
            prog);
}

static int dispatch(const char *cmd, int argc, char **argv, int first)
{
    int nargs = argc - first;
    const char *prog = argv[0];

    if (cmd == NULL || strcmp(cmd, "console") == 0) {
        if (nargs != 0) {
            print_usage(prog);
            return 1;
        }
        return run_console();
    }
    if (strcmp(cmd, "run") == 0) {
        return run_policy(argc, argv, first);
    }
    if (strcmp(cmd, "calib") == 0 || strcmp(cmd, "mark-left") == 0 ||
        strcmp(cmd, "mark-right") == 0 || strcmp(cmd, "enable") == 0) {
        if (nargs != 0) {
            fprintf(stderr, "[edge] %s takes no arguments\n", cmd);
            return 1;
        }
        console_req_t req;
        if (parse_argv_as_console(cmd, NULL, NULL, &req) != 0) {
            print_usage(prog);
            return 1;
        }
        return console_exec(&req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "center") == 0 || strcmp(cmd, "query") == 0 || strcmp(cmd, "reset") == 0) {
        if (nargs != 0) {
            fprintf(stderr, "[edge] %s takes no arguments\n", cmd);
            return 1;
        }
        console_req_t req;
        if (parse_argv_as_console(cmd, NULL, NULL, &req) != 0) {
            print_usage(prog);
            return 1;
        }
        return console_exec(&req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "state") == 0) {
        if (nargs != 0) {
            fprintf(stderr, "[edge] state takes no arguments\n");
            return 1;
        }
        return oneshot_state();
    }
    if (strcmp(cmd, "vel") == 0) {
        double hold = 1.0;
        if (nargs == 3 && strcmp(argv[first + 1], "--hold") == 0) {
            if (parse_duration(argv[first + 2], &hold) != 0) {
                print_usage(prog);
                return 1;
            }
        } else if (nargs != 1) {
            print_usage(prog);
            return 1;
        }
        console_req_t req;
        if (parse_argv_as_console("vel", argv[first], NULL, &req) != 0) {
            print_usage(prog);
            return 1;
        }
        return oneshot_vel(req.vel_mm_s, hold);
    }
    if (strcmp(cmd, "config") == 0) {
        console_req_t req;
        if (nargs == 0 || nargs == 2) {
            if (parse_argv_as_console("config", nargs == 0 ? NULL : argv[first],
                                      nargs == 0 ? NULL : argv[first + 1], &req) != 0) {
                fprintf(stderr, "[edge] invalid config arguments\n");
                return 1;
            }
            return console_exec(&req) == 0 ? 0 : 1;
        }
        print_usage(prog);
        return 1;
    }
    print_usage(prog);
    return 1;
}

static int is_known_command(const char *cmd)
{
    static const char *const names[] = {
        "console", "run", "home", "center", "vel", "state", "query", "config", "reset",
    };
    for (size_t k = 0; k < sizeof(names) / sizeof(names[0]); k++) {
        if (strcmp(cmd, names[k]) == 0) {
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    const char *port = CFG_SERIAL_PORT;
    const char *cmd = NULL;
    int first = argc; /* 子命令参数起点 */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = argv[++i];
        } else {
            cmd = argv[i]; /* 第一个非 --port 参数即子命令 */
            first = i + 1;
            break;
        }
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (cmd != NULL && !is_known_command(cmd)) {
        print_usage(argv[0]); /* 命令行错误先于任何硬件访问被拒绝 */
        return 1;
    }

    if (rig_open(port, CFG_BAUD) != 0) {
        fprintf(stderr, "[edge] cannot open %s\n", port);
        return 1;
    }

    int rc = dispatch(cmd, argc, argv, first);

    /* 会话形态（run/console）退出即停机；一次性命令不干预电机状态，
     * 其中 vel 自身管理起停。 */
    if (cmd == NULL || strcmp(cmd, "console") == 0 || strcmp(cmd, "run") == 0) {
        rig_send_vel(0.0);
    }
    policy_deinit(); /* 未初始化时是幂等空操作 */
    rig_close();
    return rc;
}
