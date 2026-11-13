#include "session.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct metadata { int64_t value; double at; bool known; };
enum field { CAL_STATE, CAL_LENGTH, CAL_SAVED, CAL_STORAGE,
             VEL_MAX, ACC_MAX, WDT_MS, WDT_DEC, MARGIN, FIELD_COUNT };
static const char *const names[FIELD_COUNT] = {
    "CAL_STATE", "CAL_LENGTH_UM", "CAL_SAVED", "CAL_STORAGE",
    "VEL_MAX", "ACC_MAX", "WDT_MS", "WDT_DEC", "X_MARGIN_UM"
};
struct device_session {
    device_limits limits;
    device_send_fn send;
    void *context;
    device_snapshot view;
    struct metadata meta[FIELD_COUNT];
    double now, valid_at, mode_at, stop_at, last_stop_tx;
    bool clock_known, valid_known, mode_known, disable_sent;
};

static void invalidate(device_session *s) {
    s->view.state.available = false;
    s->view.validity = false;
    s->valid_known = false;
}
static void fail(device_session *s, device_error e) {
    device_phase previous = s->view.phase;
    invalidate(s);
    if (previous == DEVICE_FINISHED) return;
    if (e == DEVICE_WRITE_FAILED || e == DEVICE_CLOCK_ERROR) {
        s->view.error = s->view.error == DEVICE_OK ? e : s->view.error;
        s->view.phase = DEVICE_FINISHED;
        if (previous == DEVICE_ACTIVE || previous == DEVICE_STOPPING)
            (void)s->send(s->context, "VEL 0\n");
        return;
    }
    if (previous == DEVICE_ACTIVE || previous == DEVICE_STOPPING) {
        if (s->view.error == DEVICE_OK) s->view.error = e;
        if (previous == DEVICE_ACTIVE) {
            s->view.phase = DEVICE_STOPPING;
            s->stop_at = s->now; s->last_stop_tx = s->now; s->disable_sent = false;
            if (!s->send(s->context, "VEL 0\n")) s->view.phase = DEVICE_FINISHED;
        }
    }
}
static bool clock_update(device_session *s, double t) {
    if (!isfinite(t) || t < 0 || (s->clock_known && t < s->now)) {
        fail(s, DEVICE_CLOCK_ERROR);
        return false;
    }
    s->now = t;
    s->clock_known = true;
    return true;
}
static bool emit(device_session *s, const char *line) {
    if (s->send(s->context, line)) return true;
    fail(s, DEVICE_WRITE_FAILED);
    return false;
}
static bool fresh(double t, double now, double limit) {
    return now >= t && now - t <= limit;
}
static bool metadata_ready(const device_session *s, bool require_fresh) {
    for (int i = 0; i < FIELD_COUNT; ++i)
        if (!s->meta[i].known || (require_fresh && !fresh(s->meta[i].at, s->now, s->limits.metadata_timeout_s))) return false;
    return s->meta[CAL_STATE].value == 2 && s->meta[CAL_SAVED].value == 1 &&
        s->meta[CAL_STORAGE].value == 0 && s->meta[CAL_LENGTH].value >= 100000 &&
        s->meta[CAL_LENGTH].value <= 1000000 &&
        s->meta[CAL_LENGTH].value / 2 > s->limits.soft_margin_um &&
        s->meta[VEL_MAX].value == s->limits.speed_mm_s &&
        s->meta[ACC_MAX].value == s->limits.acceleration_mm_s2 &&
        s->meta[WDT_MS].value == s->limits.watchdog_ms &&
        s->meta[WDT_DEC].value == s->limits.watchdog_decel_mm_s2 &&
        s->meta[MARGIN].value == s->limits.soft_margin_um;
}
static bool ready(const device_session *s, bool require_fresh_metadata) {
    return s->mode_known && s->view.mode_ready &&
        (!require_fresh_metadata || fresh(s->mode_at, s->now, s->limits.metadata_timeout_s)) && metadata_ready(s, require_fresh_metadata) &&
        s->valid_known && s->view.validity &&
        fresh(s->valid_at, s->now, s->limits.validity_timeout_s) &&
        s->view.state.available &&
        fresh(s->view.state.received_s, s->now, s->limits.state_timeout_s);
}
static bool integer(const char *p, int64_t *value) {
    if (!*p) return false;
    char *end;
    errno = 0;
    long long v = strtoll(p, &end, 10);
    if (errno || *end) return false;
    *value = (int64_t)v;
    return true;
}
static bool finite_positive(double x) { return isfinite(x) && x > 0; }

device_session *device_session_create(device_limits l, device_send_fn send, void *context) {
    if (!send || l.speed_mm_s <= 0 || l.acceleration_mm_s2 <= 0 ||
        l.soft_margin_um <= 0 || l.watchdog_ms <= 0 || l.watchdog_decel_mm_s2 <= 0 ||
        !finite_positive(l.state_timeout_s) || !finite_positive(l.validity_timeout_s) ||
        !finite_positive(l.metadata_timeout_s) || !finite_positive(l.stop_timeout_s)) return NULL;
    device_session *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->limits = l; s->send = send; s->context = context;
    s->view.phase = DEVICE_OBSERVING;
    return s;
}
void device_session_destroy(device_session *s) { free(s); }
bool device_session_refresh(device_session *s, double now) {
    if (!clock_update(s, now) || s->view.phase == DEVICE_FINISHED) return false;
    return emit(s, "QUERY\n") && emit(s, "CALIB?\n") && emit(s, "CONFIG?\n");
}

bool device_session_feed(device_session *s, const char *line, double now) {
    if (!clock_update(s, now)) return false;
    /* 先检查旧快照截止，积压字节不能靠新的接收时间恢复过期会话。 */
    device_session_tick(s, now);
    /* 失败锁存：后来的有效行不能恢复同一会话。 */
    if (s->view.phase == DEVICE_FINISHED) return false;
    if (!line || strlen(line) >= 160) { fail(s, DEVICE_INVALID_LINE); return false; }
    char buffer[160]; strcpy(buffer, line);
    char *words[7]; size_t n = 0;
    for (char *p = strtok(buffer, " \t\r\n"); p; p = strtok(NULL, " \t\r\n")) {
        if (n == 7) { fail(s, DEVICE_INVALID_LINE); return false; }
        words[n++] = p;
    }
    if (!n) return true;
    if (!strcmp(words[0], "VEL_VALID")) {
        if (n != 3 ||
            (strcmp(words[1], "0") && strcmp(words[1], "1")) ||
            (strcmp(words[2], "0") && strcmp(words[2], "1"))) goto malformed;
        bool valid = !strcmp(words[1], "1") && !strcmp(words[2], "1");
        if (!valid) {
            invalidate(s);
            if (s->view.phase == DEVICE_ACTIVE) fail(s, DEVICE_INVALID_MEASUREMENT);
        } else {
            /* 无效后必须重新收到STATE，不能复用旧缓存。 */
            s->valid_known = true; s->view.validity = true; s->valid_at = now;
        }
    } else if (!strcmp(words[0], "STATE")) {
        int64_t v[4];
        if (n != 5) goto malformed;
        for (int i = 0; i < 4; ++i)
            if (!integer(words[i+1], &v[i]) || v[i] < INT32_MIN || v[i] > INT32_MAX) goto malformed;
        if (s->valid_known && s->view.validity && fresh(s->valid_at, now, s->limits.validity_timeout_s)) {
            s->view.state = (device_state){v[0], v[1], v[2], v[3], now, true};
        }
    } else if (!strcmp(words[0], "MODE")) {
        if (n != 3) goto malformed;
        bool disabled = !strcmp(words[1], "DISABLED") && !strcmp(words[2], "NONE");
        s->view.mode_ready = !strcmp(words[1], "READY") && !strcmp(words[2], "NONE");
        s->mode_known = true; s->mode_at = now;
        if (s->view.phase == DEVICE_ACTIVE && !s->view.mode_ready) fail(s, DEVICE_FIRMWARE_FAULT);
        else if (s->view.phase == DEVICE_STOPPING && disabled && s->disable_sent) {
            s->view.phase = DEVICE_FINISHED; s->view.disabled_confirmed = true;
        }
        else if (!s->view.mode_ready && !disabled) {
            invalidate(s);
            if (s->view.phase == DEVICE_STOPPING) fail(s, DEVICE_STOP_UNCONFIRMED);
        }
    } else if (!strcmp(words[0], "OK")) {
        if (n >= 2) {
            for (int i = 0; i < FIELD_COUNT; ++i) if (!strcmp(words[1], names[i])) {
                int64_t value;
                if (n != 3 || !integer(words[2], &value)) {
                    s->meta[i].known = false;
                    goto malformed;
                }
                if (s->view.phase == DEVICE_ACTIVE && s->meta[i].known && s->meta[i].value != value) {
                    fail(s, DEVICE_FIRMWARE_FAULT);
                    return false;
                }
                s->meta[i] = (struct metadata){value, now, true};
                if (i == CAL_LENGTH) s->view.travel_um = value;
            }
        }
    } else if (!strcmp(words[0], "ERR")) {
        if (s->view.phase == DEVICE_STOPPING && n == 2 && !strcmp(words[1], "BUSY")) s->disable_sent = false;
        else if (s->view.phase == DEVICE_ACTIVE || s->view.phase == DEVICE_STOPPING) fail(s, DEVICE_FIRMWARE_FAULT);
    }
    device_session_tick(s, now);
    return s->view.phase == DEVICE_OBSERVING || s->view.error == DEVICE_OK;
malformed:
    fail(s, DEVICE_INVALID_LINE);
    return false;
}

bool device_session_start(device_session *s, double now) {
    if (!clock_update(s, now) || s->view.phase != DEVICE_OBSERVING) return false;
    int64_t bound = s->meta[CAL_LENGTH].value / 2 - s->limits.soft_margin_um;
    if (!ready(s, true) || s->view.state.x_um <= -bound || s->view.state.x_um >= bound) {
        s->view.error = DEVICE_NOT_READY; return false;
    }
    s->view.phase = DEVICE_ACTIVE; s->view.error = DEVICE_OK;
    return true;
}
bool device_session_action(device_session *s, double action, double now) {
    if (!clock_update(s, now)) return false;
    device_session_tick(s, now);
    if (s->view.phase != DEVICE_ACTIVE) return false;
    if (!isfinite(action) || action < -1 || action > 1) { fail(s, DEVICE_INVALID_ACTION); return false; }
    int velocity = (int)lround(action * s->limits.speed_mm_s);
    int64_t bound = s->meta[CAL_LENGTH].value / 2 - s->limits.soft_margin_um;
    if ((s->view.state.x_um >= bound && velocity > 0) ||
        (s->view.state.x_um <= -bound && velocity < 0)) velocity = 0;
    char line[48]; snprintf(line, sizeof line, "VEL %d\n", velocity);
    return emit(s, line);
}
bool device_session_stop(device_session *s, double now) {
    if (!clock_update(s, now) || s->view.phase == DEVICE_FINISHED) return false;
    if (s->view.phase != DEVICE_ACTIVE && s->view.phase != DEVICE_STOPPING) return false;
    if (s->view.phase != DEVICE_STOPPING) {
        s->view.phase = DEVICE_STOPPING; s->stop_at = now;
        s->last_stop_tx = now; s->disable_sent = false;
        s->view.state.available = false;
        return emit(s, "VEL 0\n");
    }
    device_session_tick(s, now);
    return s->view.phase == DEVICE_OBSERVING || s->view.error == DEVICE_OK;
}
void device_session_abort(device_session *s, double now) {
    if (!clock_update(s, now) || s->view.phase == DEVICE_FINISHED) return;
    if (s->view.phase == DEVICE_OBSERVING) {
        invalidate(s);
        s->view.error = DEVICE_APPLICATION_ERROR;
        s->view.phase = DEVICE_FINISHED;
        return;
    }
    fail(s, DEVICE_APPLICATION_ERROR);
    device_session_tick(s, now);
}
void device_session_tick(device_session *s, double now) {
    if (!clock_update(s, now) || s->view.phase == DEVICE_FINISHED) return;
    if (s->view.phase == DEVICE_ACTIVE && !ready(s, true)) { fail(s, DEVICE_STALE); return; }
    if (s->view.phase == DEVICE_OBSERVING && s->view.state.available &&
        (!fresh(s->view.state.received_s, now, s->limits.state_timeout_s) ||
         !fresh(s->valid_at, now, s->limits.validity_timeout_s))) s->view.state.available = false;
    if (s->view.phase != DEVICE_STOPPING) return;
    if (now - s->stop_at >= s->limits.stop_timeout_s) {
        if (s->view.error == DEVICE_OK) s->view.error = DEVICE_STOP_UNCONFIRMED;
        s->view.phase = DEVICE_FINISHED; invalidate(s); return;
    }
    if (!s->disable_sent && s->view.state.available && s->view.state.received_s >= s->stop_at &&
        fresh(s->view.state.received_s, now, s->limits.state_timeout_s) &&
        s->view.validity && fresh(s->valid_at, now, s->limits.validity_timeout_s) &&
        s->view.state.vx_mm_s == 0) {
        s->disable_sent = true;
        if (!emit(s, "DISABLE\n")) return;
    }
    if (now - s->last_stop_tx >= 0.03) {
        s->last_stop_tx = now;
        (void)emit(s, "VEL 0\n");
    }
}
device_snapshot device_session_snapshot(const device_session *s) { return s->view; }
