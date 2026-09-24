/* 行协议实现。契约见 deploy/v2/protocol.md；本文件不依赖串口，可在主机上单测。 */

#include "proto.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 解析一个十进制整数字段。成功时推进 *p 并返回 true。 */
static bool parse_int(const char **p, int64_t *out)
{
    char *end;
    long long v;
    const char *s = *p;

    while (*s == ' ') {
        s++;
    }
    if (*s == '\0') {
        return false;
    }
    v = strtoll(s, &end, 10);
    if (end == s) {
        return false;
    }
    *p = end;
    *out = (int64_t)v;
    return true;
}

static bool parse_word(const char **p, const char **begin, size_t *len)
{
    const char *s = *p;

    while (*s == ' ') {
        s++;
    }
    *begin = s;
    while (*s != '\0' && *s != ' ') {
        s++;
    }
    *len = (size_t)(s - *begin);
    *p = s;
    return *len > 0;
}

static bool word_eq(const char *w, size_t len, const char *lit)
{
    return strlen(lit) == len && memcmp(w, lit, len) == 0;
}

static bool parse_mode_word(const char *w, size_t len, fw_mode_t *out)
{
    if (word_eq(w, len, "DISABLED")) {
        *out = FW_DISABLED;
    } else if (word_eq(w, len, "HOMING")) {
        *out = FW_HOMING;
    } else if (word_eq(w, len, "READY")) {
        *out = FW_READY;
    } else if (word_eq(w, len, "FAULT")) {
        *out = FW_FAULT;
    } else {
        return false;
    }
    return true;
}

static bool parse_reason_word(const char *w, size_t len, fw_reason_t *out)
{
    if (word_eq(w, len, "NONE")) {
        *out = FW_REASON_NONE;
    } else if (word_eq(w, len, "STEP_LOSS")) {
        *out = FW_REASON_STEP_LOSS;
    } else if (word_eq(w, len, "STALL")) {
        *out = FW_REASON_STALL;
    } else if (word_eq(w, len, "AS5600")) {
        *out = FW_REASON_AS5600;
    } else if (word_eq(w, len, "MT6835")) {
        *out = FW_REASON_MT6835;
    } else if (word_eq(w, len, "HOME_TIMEOUT")) {
        *out = FW_REASON_HOME_TIMEOUT;
    } else {
        return false;
    }
    return true;
}

static bool copy_param(proto_msg_t *msg, const char *w, size_t len)
{
    if (len == 0 || len >= sizeof(msg->param)) {
        return false;
    }
    memcpy(msg->param, w, len);
    msg->param[len] = '\0';
    return true;
}

proto_msg_type_t proto_parse_line(const char *line, proto_msg_t *msg)
{
    const char *p = line;
    const char *w;
    size_t wl;

    if (!parse_word(&p, &w, &wl)) {
        return PROTO_UNKNOWN;
    }

    proto_msg_type_t type;
    if (word_eq(w, wl, "STATE")) {
        type = PROTO_STATE;
    } else if (word_eq(w, wl, "MODE")) {
        type = PROTO_MODE;
    } else if (word_eq(w, wl, "OK")) {
        type = PROTO_OK;
    } else if (word_eq(w, wl, "ERR")) {
        type = PROTO_ERR;
    } else {
        return PROTO_UNKNOWN;
    }

    proto_msg_t m;
    memset(&m, 0, sizeof(m));
    m.type = type;

    switch (type) {
    case PROTO_STATE:
        if (!parse_int(&p, &m.x_um) || !parse_int(&p, &m.theta_urad) ||
            !parse_int(&p, &m.vx_mm_s) || !parse_int(&p, &m.vth_mrad_s)) {
            return PROTO_UNKNOWN;
        }
        break;
    case PROTO_MODE:
        if (!parse_word(&p, &w, &wl) || !parse_mode_word(w, wl, &m.mode)) {
            return PROTO_UNKNOWN;
        }
        if (!parse_word(&p, &w, &wl) || !parse_reason_word(w, wl, &m.reason)) {
            return PROTO_UNKNOWN;
        }
        break;
    case PROTO_OK:
        if (!parse_word(&p, &w, &wl) || !copy_param(&m, w, wl)) {
            return PROTO_UNKNOWN;
        }
        if (!parse_int(&p, &m.value)) {
            return PROTO_UNKNOWN;
        }
        break;
    case PROTO_ERR:
        if (!parse_word(&p, &w, &wl) || !copy_param(&m, w, wl)) {
            return PROTO_UNKNOWN;
        }
        break;
    default:
        return PROTO_UNKNOWN;
    }

    /* 行尾只允许空白：多余字段判为格式错误，整行丢弃 */
    while (*p == ' ') {
        p++;
    }
    if (*p != '\0') {
        return PROTO_UNKNOWN;
    }

    *msg = m;
    return type;
}

void proto_rx_init(proto_rx_t *rx)
{
    rx->len = 0;
}

void proto_rx_feed(proto_rx_t *rx, const uint8_t *data, size_t n, proto_line_cb cb, void *user)
{
    for (size_t i = 0; i < n; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            /* 去掉可选的 \r，终结一行 */
            if (rx->len > 0 && rx->buf[rx->len - 1] == '\r') {
                rx->len--;
            }
            rx->buf[rx->len] = '\0';
            cb(rx->buf, rx->len, user);
            rx->len = 0;
        } else if (rx->len + 1 < sizeof(rx->buf)) {
            rx->buf[rx->len++] = c;
        }
        /* 超长行：丢弃后续字节直到换行 */
    }
}

size_t proto_format_vel(char *buf, double v_mm_s)
{
    if (!isfinite(v_mm_s) || v_mm_s < -1.0e9 || v_mm_s > 1.0e9) {
        return 0;
    }
    int n = snprintf(buf, 64, "VEL %d\n", (int)lround(v_mm_s));
    return n > 0 ? (size_t)n : 0;
}

size_t proto_format_simple(char *buf, const char *cmd)
{
    int n = snprintf(buf, 64, "%s\n", cmd);
    return n > 0 ? (size_t)n : 0;
}

size_t proto_format_config(char *buf, const char *name, long value)
{
    int n = snprintf(buf, 64, "CONFIG %s %ld\n", name, value);
    return n > 0 ? (size_t)n : 0;
}

const char *proto_mode_name(fw_mode_t m)
{
    switch (m) {
    case FW_DISABLED:
        return "DISABLED";
    case FW_HOMING:
        return "HOMING";
    case FW_READY:
        return "READY";
    case FW_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

const char *proto_reason_name(fw_reason_t r)
{
    switch (r) {
    case FW_REASON_NONE:
        return "NONE";
    case FW_REASON_STEP_LOSS:
        return "STEP_LOSS";
    case FW_REASON_STALL:
        return "STALL";
    case FW_REASON_AS5600:
        return "AS5600";
    case FW_REASON_MT6835:
        return "MT6835";
    case FW_REASON_HOME_TIMEOUT:
        return "HOME_TIMEOUT";
    default:
        return "?";
    }
}
