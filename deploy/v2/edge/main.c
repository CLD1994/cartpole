/* CV610 应用层：组织模型、用户命令与输出；设备循环由 cartpole 会话负责。 */
#include "cartpole.h"
#include "config.h"
#include "console.h"
#include "package.h"
#include "policy.h"
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void print_usage(const char *prog);
static int g_motion_session;
static void on_signal(int sig)
{
    (void)sig;
    cartpole_request_stop();
}
static void print_event(const cartpole_event_t *event, void *context)
{
    (void)context;
    if (event->kind == CARTPOLE_EVENT_SESSION) {
        const cartpole_session_t *s = &event->data.session;
        if (s->is_mode_received)
            printf("[edge] MODE %s %s\n", cartpole_mode_name(s->reported_mode),
                   cartpole_reason_name(s->reported_mode_reason));
    } else if (event->kind == CARTPOLE_EVENT_STEP_LOSS) {
        printf("[edge] STEP_LOSS event=%" PRId64 " run=%" PRId64 " device_us=%" PRId64 "\n",
               event->data.step_loss.event_id, event->data.step_loss.run_id,
               event->data.step_loss.device_time_us);
    }
}
static int infer_model(const cartpole_measurement_t *m, float *action, void *context)
{
    const edge_package *p = context;
    float obs[4] = {(float)m->cart_position_m, (float)(p->angle_sign * m->pole_angle_rad),
                    (float)m->cart_velocity_m_s,
                    (float)(p->angle_sign * m->pole_angular_velocity_rad_s)};
    return policy_forward(obs, action);
}
static int run_policy(int argc, char **argv, int first)
{
    const char *manifest = NULL, *backend_name = NULL;
    if (argc - first == 4)
        for (int i = first; i < argc; i += 2) {
            if (!strcmp(argv[i], "--package") && !manifest)
                manifest = argv[i + 1];
            else if (!strcmp(argv[i], "--backend") && !backend_name)
                backend_name = argv[i + 1];
            else {
                manifest = NULL;
                break;
            }
        }
    if (!manifest || !backend_name ||
        (strcmp(backend_name, "cpu") && strcmp(backend_name, "npu"))) {
        fprintf(stderr,
                "[edge] run requires --backend cpu|npu --package path/to/edge-package.json\n");
        return 1;
    }
    policy_backend backend = !strcmp(backend_name, "cpu") ? POLICY_CPU : POLICY_NPU;
    edge_package package;
    char error[256];
    if (edge_package_load(manifest, &package, error, sizeof(error)) != 0) {
        fprintf(stderr, "[edge] package: %s\n", error);
        return 1;
    }
    const char *model = backend == POLICY_CPU ? package.model_cpu_path : package.model_npu_path;
    if (!model[0]) {
        fprintf(stderr, "[edge] package has no %s model; no fallback\n", backend_name);
        return 1;
    }
    if (policy_init(backend, model) != 0)
        return 1;
    fprintf(stderr, "[edge] backend=%s\n", backend_name);
    if (console_transaction_result("preflight QUERY", cartpole_query_mode(1.0)) != 0 ||
        console_transaction_result("preflight metadata", cartpole_refresh_configuration(1.0)) !=
            0 ||
        console_transaction_result("preflight STATE", cartpole_wait_measurement(1.0)) != 0)
        return 1;
    cartpole_configuration_t config = cartpole_configuration();
    if (!config.complete || config.calibration_state != CARTPOLE_CALIBRATION_READY ||
        !config.calibration_saved || !config.calibration_storage_ok ||
        config.track_length_um < 100000 || config.track_length_um > 1000000 ||
        config.track_margin_um < 0 || config.track_margin_um >= config.track_length_um / 2 ||
        config.velocity_limit_mm_s != package.velocity_limit_mm_s ||
        config.acceleration_limit_mm_s2 != package.acceleration_mm_s2 ||
        config.watchdog_timeout_ms != 100) {
        fprintf(stderr, "[edge] preflight: calibration/config mismatch\n");
        return 1;
    }
    cartpole_session_t session = cartpole_session();
    cartpole_measurement_t measurement = cartpole_measurement();
    if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_READY ||
        session.reported_mode_reason != CARTPOLE_REASON_NONE || session.has_received_fault_mode ||
        session.has_stream_failure_latch || !measurement.cart_valid || !measurement.pole_valid ||
        cartpole_measurement_age_s(measurement) < 0 ||
        cartpole_measurement_age_s(measurement) > .020) {
        fprintf(stderr, "[edge] preflight requires READY NONE and fresh valid STATE\n");
        return 1;
    }
    cartpole_controller_options_t options = {.control_hz = package.control_hz,
                                             .action_scale_mm_s = package.action_scale_mm_s,
                                             .velocity_limit_mm_s = package.velocity_limit_mm_s,
                                             .expected_configuration = config};
    g_motion_session = 1;
    printf("[edge] %.0f Hz; scale %d mm/s; limit %d mm/s; angle pause 20/resume 18 deg\n",
           package.control_hz, package.action_scale_mm_s, package.velocity_limit_mm_s);
    cartpole_result_t result = cartpole_run_controller(&options, infer_model, &package);
    return result == CARTPOLE_OK || result == CARTPOLE_CANCELLED
               ? 0
               : (console_transaction_result("control", result), 1);
}

static int run_console(void)
{
    if (console_transaction_result("QUERY", cartpole_query_mode(1.0)))
        return 1;
    g_motion_session = 1;
    return console_run();
}
static int oneshot_state(void)
{
    if (console_transaction_result("STATE", cartpole_wait_measurement(1.0)))
        return 1;
    console_print_state();
    return 0;
}
typedef struct {
    double xmin, xmax, tmin, tmax;
    int cart, pole;
} state_window;
static int sample_window(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)elapsed;
    action->kind = CARTPOLE_TICK_OBSERVE;
    state_window *w = context;
    cartpole_measurement_t m = cartpole_measurement();
    if (m.cart_valid) {
        if (!w->cart || m.cart_position_m < w->xmin)
            w->xmin = m.cart_position_m;
        if (!w->cart || m.cart_position_m > w->xmax)
            w->xmax = m.cart_position_m;
        w->cart = 1;
    }
    if (m.pole_valid) {
        if (!w->pole || m.pole_angle_rad < w->tmin)
            w->tmin = m.pole_angle_rad;
        if (!w->pole || m.pole_angle_rad > w->tmax)
            w->tmax = m.pole_angle_rad;
        w->pole = 1;
    }
    return 0;
}
static int oneshot_state_window(double secs)
{
    if (console_transaction_result("STATE", cartpole_wait_measurement(1.0)))
        return 1;
    cartpole_statistics_t before = cartpole_statistics();
    state_window w = {0};
    double start = cartpole_host_monotonic_s();
    cartpole_result_t result = cartpole_run_loop(.020, secs, sample_window, &w);
    double dt = cartpole_host_monotonic_s() - start;
    cartpole_statistics_t after = cartpole_statistics();
    uint64_t count = after.received_measurements - before.received_measurements;
    uint64_t unknown = after.unknown_received_lines - before.unknown_received_lines;
    printf("[edge] window %.1f s: %" PRIu64 " STATE lines -> %.1f Hz (observed)\n", dt, count,
           dt > 0 ? count / dt : 0);
    printf("[edge] unknown lines: %" PRIu64 " (require 0)\n", unknown);
    if (w.cart)
        printf("[edge] sampled x range [%.0f, %.0f] um\n", w.xmin * 1e6, w.xmax * 1e6);
    else
        puts("[edge] cart samples unavailable");
    if (w.pole)
        printf("[edge] sampled theta range [%.0f, %.0f] urad\n", w.tmin * 1e6, w.tmax * 1e6);
    else
        puts("[edge] pole samples unavailable");
    return result == CARTPOLE_OK && count > 0 && unknown == 0 ? 0 : 1;
}
static int watchdog_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)context;
    action->kind = elapsed < 1.0 ? CARTPOLE_TICK_VELOCITY : CARTPOLE_TICK_OBSERVE;
    action->velocity_mm_s = 200;
    return 0;
}
static int oneshot_wdt_test(void)
{
    if (console_transaction_result("QUERY", cartpole_query_mode(1.0)) ||
        console_transaction_result("STATE", cartpole_wait_measurement(CFG_STATE_TIMEOUT_S)))
        return 1;
    cartpole_session_t s = cartpole_session();
    if (!s.is_mode_received || s.reported_mode != CARTPOLE_MODE_READY ||
        s.reported_mode_reason != CARTPOLE_REASON_NONE)
        return 1;
    g_motion_session = 1;
    if (console_transaction_result("wdt-test", cartpole_run_loop(.005, 2.0, watchdog_tick, NULL)))
        return 1;
    if (console_transaction_result("QUERY", cartpole_query_mode(1.0)))
        return 1;
    s = cartpole_session();
    return s.is_mode_received && s.reported_mode == CARTPOLE_MODE_READY &&
                   s.reported_mode_reason == CARTPOLE_REASON_NONE
               ? 0
               : 1;
}
static int oneshot_vel(double velocity, double hold)
{
    g_motion_session = 1;
    cartpole_result_t result = cartpole_hold_velocity(velocity, hold);
    return result == CARTPOLE_OK || result == CARTPOLE_CANCELLED
               ? 0
               : (console_transaction_result(result == CARTPOLE_MEASUREMENT_UNAVAILABLE
                                                 ? "VEL cart measurement invalid"
                                                 : "VEL",
                                             result),
                  1);
}

/* 把 argv 参数重新拼成控制台命令行走 console_parse 校验（校验单源）。 */
static int parse_argv_as_console(const char *cmd, const char *a1, const char *a2,
                                 console_req_t *req)
{
    char line[128];
    int n =
        snprintf(line, sizeof(line), "%s %s %s", cmd, a1 != NULL ? a1 : "", a2 != NULL ? a2 : "");
    if (n < 0 || (size_t)n >= sizeof(line)) {
        return -1;
    }
    return console_parse(line, req);
}

static int parse_duration(const char *s, double *out)
{
    char *end;
    if (s == NULL)
        return -1;
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
    fprintf(
        stderr,
        "usage: %s [--port /dev/ttyXXXX] [command [args]]\n"
        "  (no command)                     interactive console (default)\n"
        "  console                          same as above\n"
        "  run --backend cpu|npu --package edge-package.json       policy control (validated "
        "package; firmware must be READY)\n"
        "  calib                            one-shot: interactive 2-point calibration (MARK "
        "LEFT/RIGHT + ENABLE)\n"
        "  mark-left                        one-shot: send MARK LEFT (cart at left end)\n"
        "  mark-right                       one-shot: send MARK RIGHT (cart at right end)\n"
        "  enable                           one-shot: ENABLE after calibration, wait READY\n"
        "  center                           one-shot: move cart to track center (needs READY)\n"
        "  vel <mm/s> [--hold s]            one-shot: hold velocity for s seconds (default 1.0)\n"
        "  state [--secs s]                one-shot: STATE snapshot, or s-second window stats"
        " (rate/unknown/ranges)\n"
        "  query                            one-shot: print firmware mode\n"
        "  config [NAME VALUE]              one-shot: set one param, or dump all with no args\n"
        "  reset                           one-shot: clear FAULT\n"
        "  wdt-test                        one-shot: watchdog drop test (needs READY;"
        " cart moves ~200 mm/s for 1 s)\n",
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
        if (strcmp(cmd, "center") == 0)
            g_motion_session = 1;
        return console_exec(&req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "state") == 0) {
        double secs = 0.0;
        if (nargs == 2 && strcmp(argv[first], "--secs") == 0) {
            if (parse_duration(argv[first + 1], &secs) != 0 || secs <= 0.0) {
                print_usage(prog);
                return 1;
            }
        } else if (nargs != 0) {
            print_usage(prog);
            return 1;
        }
        return secs > 0.0 ? oneshot_state_window(secs) : oneshot_state();
    }
    if (strcmp(cmd, "wdt-test") == 0) {
        if (nargs != 0) {
            fprintf(stderr, "[edge] wdt-test takes no arguments\n");
            return 1;
        }
        return oneshot_wdt_test();
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
        "console", "run",   "calib", "mark-left", "mark-right", "enable",   "center",
        "vel",     "state", "query", "config",    "reset",      "wdt-test",
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

    if (cartpole_open(port, CFG_BAUD) != 0) {
        fprintf(stderr, "[edge] cannot open %s\n", port);
        return 1;
    }

    if (cartpole_subscribe(print_event, NULL) != CARTPOLE_OK) {
        cartpole_close();
        return 1;
    }
    int rc = dispatch(cmd, argc, argv, first);

    /* 会话形态（run/console）退出即停机；一次性命令不干预电机状态，
     * 其中 vel 自身管理起停。 */
    if (g_motion_session) {
        cartpole_stop_result_t stopped = cartpole_stop_and_confirm(2.0);
        if (stopped == CARTPOLE_STOP_DISABLED)
            printf("[edge] stop: DISABLED confirmed\n");
        else {
            fprintf(stderr, "[edge] stop: %s\n",
                    stopped == CARTPOLE_STOP_FAULT ? "FAULT retained; DISABLED not confirmed"
                                                   : "DISABLED not confirmed");
            rc = 1;
        }
    }
    policy_deinit(); /* 未初始化时是幂等空操作 */
    cartpole_close();
    return rc;
}
