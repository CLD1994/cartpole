#include "cartpole_format.h"
/* CV610 应用层：组织模型、用户命令与输出；设备循环由 cartpole 会话负责。 */
#include "cartpole.h"
#include "cartpole_control.h"
#include "cartpole_travel.h"
#include "cartpole_device.h"
#include "config.h"
#include "console.h"
#include "package.h"
#include "policy.h"
#include "serial_cartpole.h"
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void print_usage(const char *prog);
static int g_motion_session;
static volatile sig_atomic_t signal_stop;
static int cancellation_requested(void *context)
{ (void)context; return signal_stop != 0; }
static void on_signal(int sig)
{
    (void)sig;
    signal_stop = 1;
}
static void print_event(const cartpole_event_t *event, void *context)
{
    (void)context;
    if (event->kind == CARTPOLE_EVENT_SESSION) {
        const cartpole_session_t *s = &event->data.session;
        if (s->is_mode_received)
            printf("[MODE] %s %s\n", cartpole_mode_to_string(s->reported_mode),
                   cartpole_reason_to_string(s->reported_mode_reason));
    } else if (event->kind == CARTPOLE_EVENT_STEP_LOSS) {
        printf("[edge] STEP_LOSS event=%" PRId64 " run=%" PRId64 " device_us=%" PRId64 "\n",
               event->data.step_loss.event_id, event->data.step_loss.run_id,
               event->data.step_loss.device_time_us);
    }
}
typedef struct {
    int mode_seen, loss_seen;
    cartpole_event_t mode, first_loss;
} deferred_motion_events_t;

static void defer_motion_event(const cartpole_event_t *event, void *context)
{
    deferred_motion_events_t *pending = context;
    if (event->kind == CARTPOLE_EVENT_SESSION && event->data.session.is_mode_received) {
        pending->mode_seen = 1;
        pending->mode = *event;
    } else if (event->kind == CARTPOLE_EVENT_STEP_LOSS && !pending->loss_seen) {
        pending->loss_seen = 1;
        pending->first_loss = *event;
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
static int run_policy(cartpole_t *c, int argc, char **argv, int first)
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
        fprintf(stderr, "[edge] run requires --backend cpu|npu --package "
                        "path/to/edge-package.json\n");
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
    if (console_transaction_result("preflight QUERY", cartpole_query_mode(c, 1.0)) != 0 ||
        console_transaction_result("preflight metadata", cartpole_refresh_configuration(c, 1.0)) !=
            0 ||
        console_transaction_result("preflight STATE", cartpole_wait_for_measurement(c, 1.0)) != 0)
        return 1;
    cartpole_configuration_t config = cartpole_get_configuration(c);
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
    cartpole_session_t session = cartpole_get_session_state(c);
    cartpole_measurement_t measurement = cartpole_get_latest_measurement(c);
    if (!session.is_mode_received || session.reported_mode != CARTPOLE_MODE_READY ||
        session.reported_mode_reason != CARTPOLE_REASON_NONE || session.has_received_fault_mode ||
        session.has_stream_failure_latch || !measurement.cart_valid || !measurement.pole_valid ||
        cartpole_calculate_measurement_age_s(c, measurement) < 0 ||
        cartpole_calculate_measurement_age_s(c, measurement) > .020) {
        fprintf(stderr, "[edge] preflight requires READY NONE and fresh valid STATE\n");
        return 1;
    }
    cartpole_controller_options_t options = {.control_hz = package.control_hz,
                                             .action_scale_mm_s = package.action_scale_mm_s,
                                             .velocity_limit_mm_s = package.velocity_limit_mm_s,
                                             .expected_configuration = config};
    g_motion_session = 1;
    printf("[edge] %.0f Hz; scale %d mm/s; limit %d mm/s; angle pause 20/resume "
           "18 deg\n",
           package.control_hz, package.action_scale_mm_s, package.velocity_limit_mm_s);
    cartpole_result_t result = cartpole_run_controller(c, &options, infer_model, &package);
    return result == CARTPOLE_OK || result == CARTPOLE_CANCELLED
               ? 0
               : (console_transaction_result("control", result), 1);
}

static int run_console(cartpole_t *c)
{
    if (console_transaction_result("QUERY", cartpole_query_mode(c, 1.0)))
        return 1;
    g_motion_session = 1;
    return console_run(c);
}
typedef struct {
    double duration_s, elapsed_s;
    double xmin, xmax, tmin, tmax;
    uint64_t total, valid;
    int cart, pole;
} state_window;

static void collect_state_event(const cartpole_event_t *event, void *context)
{
    if (event->kind != CARTPOLE_EVENT_MEASUREMENT)
        return;
    state_window *w = context;
    const cartpole_measurement_t m = event->data.measurement;
    w->total++;
    if (m.cart_valid && m.pole_valid)
        w->valid++;
    if (m.cart_valid) {
        w->xmin = w->cart ? fmin(w->xmin, m.cart_position_m) : m.cart_position_m;
        w->xmax = w->cart ? fmax(w->xmax, m.cart_position_m) : m.cart_position_m;
        w->cart = 1;
    }
    if (m.pole_valid) {
        w->tmin = w->pole ? fmin(w->tmin, m.pole_angle_rad) : m.pole_angle_rad;
        w->tmax = w->pole ? fmax(w->tmax, m.pole_angle_rad) : m.pole_angle_rad;
        w->pole = 1;
    }
}

static cartpole_result_t read_state_window(cartpole_t *c, void *context)
{
    state_window *w = context;
    double start = cartpole_get_monotonic_time_s(c), end = start + w->duration_s;
    for (;;) {
        double now = cartpole_get_monotonic_time_s(c);
        w->elapsed_s = now - start;
        if (cartpole_is_stop_requested(c))
            return CARTPOLE_CANCELLED;
        if (now >= end)
            return CARTPOLE_OK;
        if (cartpole_receive(c) == CARTPOLE_RECEIVE_ERROR)
            return cartpole_get_stream_result(c);
        cartpole_result_t sleep = cartpole_sleep_until(c, fmin(end, now + .001));
        if (sleep != CARTPOLE_OK)
            return sleep;
    }
}

static int oneshot_state_window(cartpole_t *c, serial_cartpole_t *serial, double secs, double expected)
{
    if (console_transaction_result("STATE", cartpole_wait_for_measurement(c, 1.0)))
        return 1;
    uint64_t before = serial_cartpole_get_statistics(serial).unknown_received_lines;
    state_window w = {.duration_s = secs};
    cartpole_result_t result = cartpole_execute_operation(c, read_state_window, collect_state_event, &w);
    uint64_t unknown = serial_cartpole_get_statistics(serial).unknown_received_lines - before;
    double hz = w.elapsed_s > 0 ? w.valid / w.elapsed_s : 0;
    int rate_ok = !expected || fabs(hz - expected) <= 1.0;
    printf("时长 %.1f s，有效 STATE %" PRIu64 " 行 → %.1f Hz（接收总数 %" PRIu64 " 行）\n",
           w.elapsed_s, w.valid, hz, w.total);
    printf("UNKNOWN 行：%" PRIu64 "（要求 0）\n", unknown);
    if (w.cart)
        printf("x_um ∈ [%.0f, %.0f]（漂移 %.0f µm；x ≈ %.2f mm）\n",
               w.xmin * 1e6, w.xmax * 1e6, (w.xmax - w.xmin) * 1e6, (w.xmin + w.xmax) * 500);
    else
        puts("小车测量不可用");
    if (w.pole)
        printf("theta_urad ∈ [%.0f, %.0f]（θ ∈ [%.2f°, %.2f°]）\n",
               w.tmin * 1e6, w.tmax * 1e6, w.tmin * (180.0 / 3.14159265358979323846),
               w.tmax * (180.0 / 3.14159265358979323846));
    else
        puts("摆杆测量不可用");
    if (expected)
        printf("目标 %.9g Hz（容差 ±1 Hz）：%s\n", expected, rate_ok ? "通过" : "未通过");
    else
        puts("未指定目标频率，仅检查收到有效 STATE 且 UNKNOWN 为 0；不证明期间没有无效帧。");
    return !console_transaction_result("STATE", result) && w.valid > 0 && unknown == 0 && rate_ok ? 0 : 1;
}

static int watchdog_tick(double elapsed, cartpole_tick_action_t *action, void *context)
{
    (void)context;
    action->kind = elapsed < 1.0 ? CARTPOLE_TICK_VELOCITY : CARTPOLE_TICK_OBSERVE;
    action->velocity_mm_s = 200;
    return 0;
}
static int oneshot_wdt_test(cartpole_t *c)
{
    if (console_transaction_result("QUERY", cartpole_query_mode(c, 1.0)) ||
        console_transaction_result("STATE", cartpole_wait_for_measurement(c, CFG_STATE_TIMEOUT_S)))
        return 1;
    cartpole_session_t s = cartpole_get_session_state(c);
    if (!s.is_mode_received || s.reported_mode != CARTPOLE_MODE_READY ||
        s.reported_mode_reason != CARTPOLE_REASON_NONE)
        return 1;
    g_motion_session = 1;
    if (console_transaction_result("wdt-test",
                                   cartpole_run_loop(c, .005, 2.0, watchdog_tick, NULL)))
        return 1;
    if (console_transaction_result("QUERY", cartpole_query_mode(c, 1.0)))
        return 1;
    s = cartpole_get_session_state(c);
    const int ready = s.is_mode_received && s.reported_mode == CARTPOLE_MODE_READY &&
                      s.reported_mode_reason == CARTPOLE_REASON_NONE;
    if (ready)
        puts("[OK] 停止发送 VEL 后固件仍为 READY；随后执行端侧停机确认。");
    else
        fputs("[ERR] 停止发送 VEL 后未观察到 READY NONE。\n", stderr);
    return ready ? 0 : 1;
}
static int oneshot_vel(cartpole_t *c, double velocity, double hold)
{
    g_motion_session = 1;
    cartpole_result_t result = cartpole_hold_velocity(c, velocity, hold);
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
    fprintf(stderr,
            "usage: %s [--port /dev/ttyXXXX] [--baud RATE] [command [args]]\n"
            "  (no command)                     interactive console (default)\n"
            "  console                          same as above\n"
            "  run --backend cpu|npu --package edge-package.json       policy "
            "control (validated "
            "package; firmware must be READY)\n"
            "  calib                            one-shot: interactive 2-point "
            "calibration (MARK "
            "LEFT/RIGHT + ENABLE)\n"
            "  mark-left                        one-shot: send MARK LEFT (cart "
            "at left end)\n"
            "  mark-right                       one-shot: send MARK RIGHT (cart "
            "at right end)\n"
            "  confirm-left                     restore saved left reference without enabling\n"
            "  calib-status                     read track calibration status\n"
            "  calibrate-pole-down              calibrate the stationary, hanging pole (DISABLED)\n"
            "  enable [--timeout s]             one-shot: ENABLE after "
            "calibration, wait READY\n"
            "  disable                          disable only when the firmware is stopped\n"
            "  center                           one-shot: move cart to track "
            "center (needs READY)\n"
            "  vel <mm/s> [--hold s | --plan]   按行程制动；--hold 定时，--plan 只读\n"
            "  state [--secs s] [--expected-hz Hz]  观测有效 STATE（默认 10 秒）\n"
            "  query                            one-shot: print firmware mode\n"
            "  dump                             one-shot: read all firmware settings\n"
            "  config [NAME VALUE]              one-shot: set one param, or dump "
            "all with no args\n"
            "  reset                           one-shot: clear FAULT\n"
            "  wdt-test                        one-shot: watchdog drop test "
            "(needs READY;"
            " cart moves ~200 mm/s for 1 s)\n",
            prog);
}

static int dispatch(cartpole_t *c, serial_cartpole_t *serial, const char *cmd, int argc,
                    char **argv, int first)
{
    int nargs = argc - first;
    const char *prog = argv[0];

    if (cmd == NULL || strcmp(cmd, "console") == 0) {
        if (nargs != 0) {
            print_usage(prog);
            return 1;
        }
        return run_console(c);
    }
    if (strcmp(cmd, "run") == 0) {
        return run_policy(c, argc, argv, first);
    }
    if (strcmp(cmd, "enable") == 0) {
        console_req_t req;
        if ((nargs != 0 && nargs != 2) ||
            parse_argv_as_console(cmd, nargs ? argv[first] : NULL,
                                  nargs ? argv[first + 1] : NULL, &req) != 0) {
            print_usage(prog);
            return 1;
        }
        return console_exec(c, &req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "calib") == 0 || strcmp(cmd, "mark-left") == 0 ||
        strcmp(cmd, "mark-right") == 0 || strcmp(cmd, "confirm-left") == 0 ||
        strcmp(cmd, "calib-status") == 0 || strcmp(cmd, "calibrate-pole-down") == 0 ||
        strcmp(cmd, "disable") == 0 || strcmp(cmd, "center") == 0 ||
        strcmp(cmd, "query") == 0 || strcmp(cmd, "reset") == 0) {
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
        return console_exec(c, &req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "state") == 0) {
        double secs = 10.0, expected = 0.0;
        int have_secs = 0;
        for (int i = first; i + 1 < argc; i += 2) {
            if (!have_secs && strcmp(argv[i], "--secs") == 0 &&
                parse_duration(argv[i + 1], &secs) == 0 && secs > 0) {
                have_secs = 1;
            } else if (!expected && strcmp(argv[i], "--expected-hz") == 0 &&
                       parse_duration(argv[i + 1], &expected) == 0 && expected > 0) {
                continue;
            } else {
                print_usage(prog);
                return 1;
            }
        }
        if (nargs % 2 != 0) {
            print_usage(prog);
            return 1;
        }
        return oneshot_state_window(c, serial, secs, expected);
    }
    if (strcmp(cmd, "wdt-test") == 0) {
        if (nargs != 0) {
            fprintf(stderr, "[edge] wdt-test takes no arguments\n");
            return 1;
        }
        return oneshot_wdt_test(c);
    }
    if (strcmp(cmd, "vel") == 0) {
        cartpole_travel_options_t opts = {.reserve_mm = 20, .inflight_ms = 10,
                                         .start_tolerance_mm = .5, .max_seconds = 30};
        const char *speed = NULL;
        double hold = 0;
        unsigned seen = 0;
        for (int i = first; i < argc; ++i) {
            const char *arg = argv[i];
            if (!strcmp(arg, "--plan")) {
                if (opts.plan_only)
                    goto invalid_vel;
                opts.plan_only = 1;
                continue;
            }
            static const char *const names[] = {
                "--hold", "--reserve-mm", "--inflight-ms", "--start-tolerance-mm", "--max-seconds"
            };
            double *values[] = {&hold, &opts.reserve_mm, &opts.inflight_ms,
                                &opts.start_tolerance_mm, &opts.max_seconds};
            size_t k = 0;
            for (; k < sizeof names / sizeof names[0]; ++k)
                if (!strcmp(arg, names[k]))
                    break;
            if (k < sizeof names / sizeof names[0]) {
                if (seen & (1u << k) || ++i >= argc || parse_duration(argv[i], values[k]))
                    goto invalid_vel;
                seen |= 1u << k;
            } else if (!speed) {
                speed = arg;
            } else {
                goto invalid_vel;
            }
        }
        console_req_t req;
        if (!speed || parse_argv_as_console("vel", speed, NULL, &req) != 0)
            goto invalid_vel;
        if (seen & 1) {
            if (opts.plan_only || seen != 1 || hold <= 0)
                goto invalid_vel;
            return oneshot_vel(c, req.vel_mm_s, hold);
        }
        if (req.vel_mm_s == 0 || opts.start_tolerance_mm > 1.0 || opts.max_seconds <= 0)
            goto invalid_vel;
        cartpole_travel_report_t report = {0};
        deferred_motion_events_t events = {0};
        if (cartpole_subscribe(c, defer_motion_event, &events) != CARTPOLE_OK)
            return 1;
        cartpole_result_t result = cartpole_travel_velocity(c, req.vel_mm_s, &opts, &report);
        int fallback = report.motion_attempted && !report.stop_confirmed && !opts.plan_only;
        cartpole_stop_result_t stop_result = fallback ? cartpole_stop_and_confirm(c, 2.0)
                                                     : CARTPOLE_STOP_UNCONFIRMED;
        (void)cartpole_subscribe(c, print_event, NULL);
        /* 串口收尾前不做可能被终端/日志管道阻塞的输出。 */
        if (events.mode_seen)
            print_event(&events.mode, NULL);
        if (events.loss_seen)
            print_event(&events.first_loss, NULL);
        if (report.plan_valid)
            printf("物理端 ±%.3f mm；软件端 ±%.3f mm；目标 %.3f mm；"
                   "正常提前制动预算 %.3f mm；异常断链条件预算 %.3f mm\n",
                   report.physical_mm, report.soft_mm, report.target_mm,
                   report.braking_budget_mm, report.watchdog_budget_mm);
        if (report.zero_requested && isfinite(report.zero_position_mm))
            printf("零速请求位置：%.3f mm\n", report.zero_position_mm);
        if (report.has_final_position)
            printf("最终收到位置：%.3f mm；距目标：%.3f mm\n",
                   report.final_position_mm, report.target_mm - report.final_position_mm);
        if (report.observed_target_95)
            printf("达到同向目标速度的 95%%：%.1f ms（主机接收估计）\n", report.target_95_s * 1000);
        else if (report.observed_peak)
            printf("未观测到目标速度的 95%%；同向峰值 %.0f mm/s，耗时 %.1f ms（主机接收估计）\n",
                   report.peak_velocity_mm_s, report.peak_at_s * 1000);
        if (report.stop_confirmed)
            puts("收到停稳证据");
        if (fallback) {
            if (stop_result == CARTPOLE_STOP_DISABLED)
                puts("行程停稳未确认；收尾已确认 DISABLED");
            else
                fputs("[ERR] 停稳及禁用未确认；需现场确认停止。\n", stderr);
        }
        return console_transaction_result(opts.plan_only ? "VEL --plan" : "VEL", result) ? 1 : 0;
invalid_vel:
        print_usage(prog);
        return 1;
    }
    if (strcmp(cmd, "dump") == 0) {
        console_req_t req;
        if (nargs != 0 || parse_argv_as_console(cmd, NULL, NULL, &req) != 0) {
            print_usage(prog);
            return 1;
        }
        return console_exec(c, &req) == 0 ? 0 : 1;
    }
    if (strcmp(cmd, "config") == 0) {
        console_req_t req;
        if (nargs == 0 || nargs == 2) {
            if (parse_argv_as_console("config", nargs == 0 ? NULL : argv[first],
                                      nargs == 0 ? NULL : argv[first + 1], &req) != 0) {
                fprintf(stderr, "[edge] invalid config arguments\n");
                return 1;
            }
            return console_exec(c, &req) == 0 ? 0 : 1;
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
        "vel",     "state", "query", "config",    "dump",       "reset",      "wdt-test",
        "confirm-left", "calib-status", "calibrate-pole-down", "disable",
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
    int baud = CFG_BAUD;
    const char *cmd = NULL;
    int first = argc; /* 子命令参数起点 */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = argv[++i];
        } else if (strcmp(argv[i], "--baud") == 0) {
            if (i + 1 == argc) {
                print_usage(argv[0]);
                return 1;
            }
            const char *arg = argv[++i];
            char *end;
            errno = 0;
            long value = strtol(arg, &end, 10);
            if (errno != 0 || end == arg || *end || value <= 0 || value > INT_MAX) {
                fprintf(stderr, "[edge] invalid baud rate: %s\n", arg);
                return 1;
            }
            baud = (int)value;
        } else {
            cmd = argv[i]; /* 第一个非全局选项即子命令 */
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

    cartpole_clock_t clock;
    if (cartpole_clock_init(&clock, NULL))
        return 1;
    serial_cartpole_t *serial = serial_cartpole_open(port, baud, &clock);
    if (!serial) {
        fprintf(stderr, "[edge] cannot open %s at %d baud\n", port, baud);
        return 1;
    }

    cartpole_t *c = cartpole_create(serial_cartpole_get_device(serial));
    if (c) (void)cartpole_set_cancellation_source(c, cancellation_requested, NULL);
    if (!c) {
        serial_cartpole_close(serial);
        return 1;
    }

    if (cartpole_subscribe(c, print_event, NULL) != CARTPOLE_OK) {
        cartpole_destroy(c);
        serial_cartpole_close(serial);
        return 1;
    }
    int rc = dispatch(c, serial, cmd, argc, argv, first);

    /* 会话形态（run/console）退出即停机；一次性命令不干预电机状态，
     * 其中 vel 自身管理起停。 */
    if (g_motion_session) {
        cartpole_stop_result_t stopped = cartpole_stop_and_confirm(c, 2.0);
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
    cartpole_destroy(c);
    serial_cartpole_close(serial);
    return rc;
}
