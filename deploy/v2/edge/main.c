/* CV610 正式控制应用与显式维护命令。模型包、会话与控制决策各自拥有单一契约。 */
#include "cartpole.h"
#include "config.h"
#include "console.h"
#include "control.h"
#include "package.h"
#include "policy.h"
#include "proto.h"

#include <inttypes.h>
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
    cartpole_request_stop();
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

/* ---- 正式策略应用 ---- */
static int g_motion_session;

static control_input snapshot_input(const cartpole_snapshot_t *snapshot,
                                    cartpole_pump_result_t pump_result)
{
    control_input in = {0};
    in.ready = snapshot->is_mode_received && snapshot->reported_mode == FW_READY &&
               snapshot->reported_mode_reason == FW_REASON_NONE &&
               !snapshot->has_received_fault_mode;
    in.flags = snapshot->measurement_validity_bits;
    in.age_s = cartpole_state_age_s(*snapshot);
    in.theta_rad = (double)snapshot->theta_urad * 1e-6;
    in.backlog = pump_result == CARTPOLE_PUMP_BACKLOG;
    in.io_error = pump_result == CARTPOLE_PUMP_ERROR || snapshot->has_stream_failure_latch;
    return in;
}
static control_input current_input(cartpole_pump_result_t result)
{
    cartpole_snapshot_t snapshot = cartpole_snapshot();
    return snapshot_input(&snapshot, result);
}

static int active_metadata_matches(const edge_package *package, int64_t length, int64_t margin)
{
    const char *names[] = {"CAL_STATE", "CAL_LENGTH_UM", "CAL_SAVED", "CAL_STORAGE",
                           "VEL_MAX",   "ACC_MAX",       "WDT_MS",    "X_MARGIN_UM"};
    const int64_t expected[] = {2,
                                length,
                                1,
                                0,
                                (int64_t)package->velocity_limit_mm_s,
                                (int64_t)package->acceleration_mm_s2,
                                100,
                                margin};
    for (size_t i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
        int64_t value;
        if (cartpole_cached_value(names[i], &value) != 0 || value != expected[i])
            return 0;
    }
    return 1;
}

typedef struct {
    control_gate gate;
    const edge_package *package;
    int64_t length, margin;
} control_runtime;

static void observe_update(proto_msg_type_t event, const cartpole_snapshot_t *snapshot,
                           void *context)
{
    (void)event;
    control_runtime *runtime = context;
    control_input in = snapshot_input(snapshot, CARTPOLE_PUMP_EMPTY);
    if (!active_metadata_matches(runtime->package, runtime->length, runtime->margin))
        in.io_error = 1;
    (void)control_observe(&runtime->gate, &in, snapshot->last_state_received_host_monotonic_s);
}

static int control_end(int result)
{
    cartpole_set_observer(NULL, NULL);
    return result;
}

static int control_loop(const edge_package *package, int64_t length, int64_t margin)
{
    const double period = 1.0 / package->control_hz;
    const double track_limit_m = (length * .5 - margin) * 1e-6;
    double next = cartpole_host_monotonic_s();
    control_runtime runtime = {.package = package, .length = length, .margin = margin};
    runtime.gate.has_time = 1;
    runtime.gate.last_observed_at = cartpole_snapshot().last_state_received_host_monotonic_s;
    cartpole_set_observer(observe_update, &runtime);
    uint64_t consumed = 0;
    printf("[edge] %.0f Hz; scale %d mm/s; limit %d mm/s; angle pause 20/resume 18 deg\n",
           package->control_hz, package->action_scale_mm_s, package->velocity_limit_mm_s);
    while (!cartpole_stop_requested()) {
        sleep_to(next);
        int pumped = cartpole_pump();
        control_input in = current_input(pumped);
        if (!active_metadata_matches(package, length, margin))
            return control_end(-1);
        control_decision decision = control_check(&runtime.gate, &in);
        if (decision == CONTROL_STOP) {
            fprintf(stderr, "[edge] control stopped: mode, measurement or transport unavailable\n");
            return control_end(-1);
        }
        cartpole_snapshot_t sample = cartpole_snapshot();
        uint64_t seq = sample.received_state_count;
        if (decision == CONTROL_DRIVE && seq != consumed) {
            int64_t x, theta, vx, w;
            x = sample.x_um;
            theta = sample.theta_urad;
            vx = sample.vx_mm_s;
            w = sample.vth_mrad_s;
            float obs[4] = {(float)(x * 1e-6), (float)(package->angle_sign * theta * 1e-6),
                            (float)(vx * 1e-3), (float)(package->angle_sign * w * 1e-3)};
            double sample_at = sample.last_state_received_host_monotonic_s;
            unsigned long long generation = runtime.gate.generation;
            float action;
            consumed = seq;
            if (policy_forward(obs, &action) != 0) {
                fprintf(stderr, "[edge] inference failed\n");
                return control_end(-1);
            }
            /* 推理期间发生的模式/测量变化优先于动作；过期候选不补发。 */
            pumped = cartpole_pump();
            in = current_input(pumped);
            if (!active_metadata_matches(package, length, margin))
                return control_end(-1);
            decision = control_check(&runtime.gate, &in);
            if (decision == CONTROL_STOP)
                return control_end(-1);
            if (decision == CONTROL_DRIVE && generation == runtime.gate.generation &&
                !cartpole_stop_requested() && cartpole_host_monotonic_s() - sample_at <= .020) {
                x = cartpole_snapshot().x_um;
                double velocity =
                    control_velocity(action, package->action_scale_mm_s,
                                     package->velocity_limit_mm_s, x * 1e-6, track_limit_m);
                if (!isfinite(velocity))
                    return control_end(-1);
                cartpole_send_result_t sent = cartpole_try_send_vel(velocity, sample_at + .020);
                if (sent == CARTPOLE_SEND_ERROR || sent == CARTPOLE_SEND_LATE) {
                    fprintf(stderr, "[edge] %s\n",
                            sent == CARTPOLE_SEND_LATE ? "write completed after candidate deadline"
                                                       : "velocity write failed");
                    return control_end(-1);
                }
            }
        }
        next = control_next_tick(next, cartpole_host_monotonic_s(), period);
    }
    return control_end(0);
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
    if (console_transaction_result("preflight QUERY", cartpole_sync_mode(1.0)) != 0)
        return 1;
    if (console_transaction_result("preflight metadata", cartpole_refresh_metadata(1.0)) != 0)
        return 1;
    if (console_transaction_result("preflight STATE", cartpole_wait_state(1.0)) != 0)
        return 1;
    int64_t cal, length, saved, storage, velocity, acceleration, wdt, margin;
    if (cartpole_cached_value("CAL_STATE", &cal) != 0 ||
        cartpole_cached_value("CAL_LENGTH_UM", &length) != 0 ||
        cartpole_cached_value("CAL_SAVED", &saved) != 0 ||
        cartpole_cached_value("CAL_STORAGE", &storage) != 0 ||
        cartpole_cached_value("VEL_MAX", &velocity) != 0 ||
        cartpole_cached_value("ACC_MAX", &acceleration) != 0 ||
        cartpole_cached_value("WDT_MS", &wdt) != 0 ||
        cartpole_cached_value("X_MARGIN_UM", &margin) != 0)
        return 1;
    if (cal != 2 || saved != 1 || storage != 0 || length < 100000 || length > 1000000 ||
        margin < 0 || margin >= length / 2 || velocity != package.velocity_limit_mm_s ||
        acceleration != package.acceleration_mm_s2 || wdt != 100) {
        fprintf(stderr,
                "[edge] preflight: calibration/config mismatch; requires VEL_MAX=%d ACC_MAX=%d "
                "WDT_MS=100\n",
                package.velocity_limit_mm_s, package.acceleration_mm_s2);
        return 1;
    }
    control_input in = current_input(cartpole_pump());
    if (!in.ready || in.flags != 3 || in.age_s < 0 || in.age_s > .020 || in.io_error ||
        in.backlog) {
        fprintf(stderr, "[edge] preflight requires READY NONE and fresh valid STATE\n");
        return 1;
    }
    /* 模型启动不定位、不复位、不使能，也不覆盖现场配置。 */
    g_motion_session = 1;
    return control_loop(&package, length, margin) == 0 ? 0 : 1;
}

/* ---- 控制台（默认形态） ---- */

static int run_console(void)
{
    cartpole_result_t query = cartpole_sync_mode(1.0);
    if (query != CARTPOLE_OK) {
        console_transaction_result("QUERY", query);
        return 1;
    }
    fw_mode_t mode = cartpole_snapshot().reported_mode;
    printf("[edge] firmware mode: %s\n", proto_mode_name(mode));

    g_motion_session = 1;
    return console_run();
}

/* ---- 一次性子命令 ---- */

static int oneshot_state(void)
{
    if (console_transaction_result("STATE", cartpole_wait_state(1.0)) != 0) {
        return 1;
    }
    console_print_state();
    return 0;
}

/* 窗口统计（与 host 工具 state --secs 同口径）：行率、无法解析行数、
 * 只统计主机采集到的有效快照，不宣称覆盖每个设备样本或验证设备采样频率。 */
static int oneshot_state_window(double secs)
{
    if (console_transaction_result("STATE", cartpole_wait_state(1.0)) != 0) {
        return 1;
    }
    uint64_t states_before = cartpole_snapshot().received_state_count,
             unknown_before = cartpole_snapshot().received_unknown_line_count;
    int64_t x, th;
    int64_t xmin = INT64_MAX, xmax = INT64_MIN;
    int64_t thmin = INT64_MAX, thmax = INT64_MIN;
    double t0 = cartpole_host_monotonic_s();
    while (!cartpole_stop_requested() && cartpole_host_monotonic_s() - t0 < secs) {
        if (cartpole_pump() < 0)
            return 1;
        x = cartpole_snapshot().x_um;
        th = cartpole_snapshot().theta_urad;
        if (cartpole_snapshot().measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID) {
            if (x < xmin)
                xmin = x;
            if (x > xmax)
                xmax = x;
        }
        if (cartpole_snapshot().measurement_validity_bits & CARTPOLE_MEASUREMENT_POLE_VALID) {
            if (th < thmin)
                thmin = th;
            if (th > thmax)
                thmax = th;
        }
        nap_ms(20);
    }
    double dt = cartpole_host_monotonic_s() - t0;
    double rate =
        dt > 0.0 ? (double)(cartpole_snapshot().received_state_count - states_before) / dt : 0.0;
    uint64_t unknown = (cartpole_snapshot().received_unknown_line_count - unknown_before);
    printf("[edge] window %.1f s: %" PRIu64 " STATE lines -> %.1f Hz (observed)\n", dt,
           (cartpole_snapshot().received_state_count - states_before), rate);
    printf("[edge] unknown lines: %" PRIu64 " (require 0)\n", unknown);
    if (xmin != INT64_MAX)
        printf("[edge] sampled x range [%lld, %lld] um  (x = %.2f .. %.2f mm)\n", (long long)xmin,
               (long long)xmax, xmin / 1000.0, xmax / 1000.0);
    else
        puts("[edge] cart samples unavailable");
    if (thmin != INT64_MAX)
        printf("[edge] sampled theta range [%lld, %lld] urad  (theta = %.2f .. %.2f deg)\n",
               (long long)thmin, (long long)thmax, thmin / 17453.3, thmax / 17453.3);
    else
        puts("[edge] pole samples unavailable");
    return ((cartpole_snapshot().received_state_count - states_before) > 0 && unknown == 0) ? 0 : 1;
}

/* 看门狗断流验证（与 host 工具 wdt-test 同流程）：以 200 mm/s 持续喂狗
 * 1 s，随后停止发送观察 1 s——固件看门狗应减速停车但模式保持 READY。 */
static int oneshot_wdt_test(void)
{
    if (console_transaction_result("QUERY", cartpole_sync_mode(1.0)) != 0) {
        return 1;
    }
    int known;
    fw_reason_t reason;
    cartpole_snapshot_t snapshot = cartpole_snapshot();
    known = snapshot.is_mode_received;
    reason = snapshot.reported_mode_reason;
    fw_mode_t mode = snapshot.reported_mode;
    if (!known || mode != FW_READY) {
        fprintf(stderr, "[edge] wdt-test requires READY (current %s)\n",
                known ? proto_mode_name(mode) : "unknown");
        return 1;
    }
    if (console_transaction_result("STATE", cartpole_wait_state(CFG_STATE_TIMEOUT_S)) != 0) {
        return 1;
    }
    g_motion_session = 1;
    double t0 = cartpole_host_monotonic_s();
    while (!cartpole_stop_requested() && cartpole_host_monotonic_s() - t0 < 1.0) {
        if (cartpole_send_vel(200.0) != 0)
            return 1;
        cartpole_pump();
        nap_ms(5);
    }
    printf("[edge] feeding stopped, observing 1 s ...\n");
    t0 = cartpole_host_monotonic_s();
    while (!cartpole_stop_requested() && cartpole_host_monotonic_s() - t0 < 1.0) {
        cartpole_pump();
        nap_ms(20);
    }
    if (console_transaction_result("QUERY", cartpole_sync_mode(1.0)) != 0) {
        return 1;
    }
    snapshot = cartpole_snapshot();
    known = snapshot.is_mode_received;
    reason = snapshot.reported_mode_reason;
    mode = snapshot.reported_mode;
    printf("[edge] QUERY -> %s %s (expect READY NONE: watchdog is a transient guard)\n",
           known ? proto_mode_name(mode) : "unknown", known ? proto_reason_name(reason) : "-");
    return (known && mode == FW_READY && reason == FW_REASON_NONE) ? 0 : 1;
}

static int oneshot_vel(double v_mm_s, double hold_s)
{
    if (console_transaction_result("QUERY", cartpole_sync_mode(1.0)) != 0) {
        return 1;
    }
    int known;
    cartpole_snapshot_t snapshot = cartpole_snapshot();
    known = snapshot.is_mode_received;
    fw_mode_t mode = snapshot.reported_mode;
    if (!known || mode != FW_READY) {
        fprintf(stderr, "[edge] VEL requires READY (current %s)\n",
                known ? proto_mode_name(mode) : "unknown");
        return 1;
    }
    /* 保持前先等到 STATE 流就位：从未收到与已过期是不同状态，不能把
     * 起始竞态当成流过期立即中止。 */
    if (console_transaction_result("STATE", cartpole_wait_state(CFG_STATE_TIMEOUT_S)) != 0) {
        return 1;
    }
    g_motion_session = 1;
    snapshot = cartpole_snapshot();
    if (!(snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID)) {
        fprintf(stderr, "[edge] VEL blocked: cart measurement invalid\n");
        return 1;
    }
    if (cartpole_send_vel(v_mm_s) != CARTPOLE_SEND_COMPLETE) {
        fprintf(stderr, "[edge] VEL send failed or blocked by session checks\n");
        return 1;
    }
    double t0 = cartpole_host_monotonic_s();
    int rc = 0;
    while (!cartpole_stop_requested() && cartpole_host_monotonic_s() - t0 < hold_s) {
        cartpole_pump();
        snapshot = cartpole_snapshot();
        known = snapshot.is_mode_received;
        mode = snapshot.reported_mode;
        const char *hold_failure = NULL;
        if (!known || mode != FW_READY)
            hold_failure = "reported mode is not READY";
        else if (snapshot.has_received_fault_mode)
            hold_failure = "MODE FAULT observed in this session";
        else if (snapshot.has_stream_failure_latch)
            hold_failure = "session stream failure latched";
        else if (!(snapshot.measurement_validity_bits & CARTPOLE_MEASUREMENT_CART_VALID))
            hold_failure = "cart measurement invalid";
        if (hold_failure) {
            fprintf(stderr, "[edge] VEL hold stopped: %s\n", hold_failure);
            rc = 1;
            break;
        }
        double age = cartpole_state_age_s(snapshot);
        if (age < 0.0 || age > CFG_STATE_TIMEOUT_S) {
            fprintf(stderr, "[edge] WARNING: STATE stream stale during hold\n");
            rc = 1;
            break;
        }
        if (cartpole_send_vel(v_mm_s) != CARTPOLE_SEND_COMPLETE) {
            fprintf(stderr, "[edge] VEL hold send failed or blocked by session checks\n");
            rc = 1;
            break;
        } /* 喂固件看门狗 */
        nap_ms(20);
    }
    if (cartpole_send_vel(0.0) != 0)
        rc = 1;
    return rc;
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
