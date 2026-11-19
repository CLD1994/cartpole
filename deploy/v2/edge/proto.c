/* 行协议实现。契约见 docs/deployment/protocol.md；本文件不依赖串口，可在主机上单测。 */

#include "proto.h"

#include <errno.h>
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
    errno = 0;
    v = strtoll(s, &end, 10);
    if (end == s || errno == ERANGE || (*end != ' ' && *end != '\0')) {
        return false;
    }
    *p = end;
    *out = (int64_t)v;
    return true;
}

/* STATE 的字段必须与 Python 正式解析器一样是规范十进制，不允许 +、前导零或额外空白。 */
static bool parse_state_number(const char **p, int64_t *out, bool last)
{
    const char *s = *p;
    if (*s++ != ' ')
        return false;
    const char *digits = s;
    if (*s == '-')
        s++;
    if (*s == '0') {
        if (s != digits)
            return false;
        s++;
    } else {
        if (*s < '1' || *s > '9')
            return false;
        while (*s >= '0' && *s <= '9')
            s++;
    }
    if ((last && *s != '\0') || (!last && *s != ' '))
        return false;
    char *end;
    errno = 0;
    long long v = strtoll(digits, &end, 10);
    if (errno == ERANGE || end != s || v < INT32_MIN || v > INT32_MAX)
        return false;
    *out = (int64_t)v;
    *p = s;
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
    } else if (word_eq(w, len, "AS5600")) {
        *out = FW_REASON_AS5600;
    } else if (word_eq(w, len, "MT6835")) {
        *out = FW_REASON_MT6835;
    } else if (word_eq(w, len, "POWER")) {
        *out = FW_REASON_POWER;
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
    } else if (word_eq(w, wl, "EVENT")) {
        type = PROTO_STEP_LOSS;
    } else {
        return PROTO_UNKNOWN;
    }

    proto_msg_t m;
    memset(&m, 0, sizeof(m));
    m.type = type;

    switch (type) {
    case PROTO_STATE: {
        int64_t flags;
        if (strncmp(line, "STATE ", 6) != 0)
            return PROTO_UNKNOWN;
        if (!parse_state_number(&p, &flags, false) || flags < 0 || flags > 3 ||
            !parse_state_number(&p, &m.x_um, false) ||
            !parse_state_number(&p, &m.theta_urad, false) ||
            !parse_state_number(&p, &m.vx_mm_s, false) ||
            !parse_state_number(&p, &m.vth_mrad_s, true)) {
            return PROTO_UNKNOWN;
        }
        m.valid_flags = (unsigned)flags;
        if (!(flags & 1) && (m.x_um || m.vx_mm_s))
            return PROTO_UNKNOWN;
        if (!(flags & 2) && (m.theta_urad || m.vth_mrad_s))
            return PROTO_UNKNOWN;
        if ((flags & 2) && (m.theta_urad < -3141592 || m.theta_urad > 3141592))
            return PROTO_UNKNOWN;
        break;
    }
    case PROTO_STEP_LOSS:
        if (!parse_word(&p, &w, &wl) || !word_eq(w, wl, "STEP_LOSS") ||
            !parse_int(&p, &m.event_id) || !parse_int(&p, &m.run_id) ||
            !parse_int(&p, &m.device_us) || m.event_id < 0 || m.run_id < 0 || m.device_us < 0)
            return PROTO_UNKNOWN;
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
    rx->overflow = false;
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
            cb(rx->overflow ? "!OVERLONG" : rx->buf, rx->overflow ? 9 : rx->len, user);
            rx->len = 0;
            rx->overflow = false;
        } else if (c == '\0') {
            rx->overflow = true;
        } else if (rx->len + 1 < sizeof(rx->buf)) {
            rx->buf[rx->len++] = c;
        } else {
            rx->overflow = true;
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
    if (cmd == NULL || strchr(cmd, '\n') || strchr(cmd, '\r'))
        return 0;
    int n = snprintf(buf, 64, "%s\n", cmd);
    return n > 0 && n < 64 ? (size_t)n : 0;
}

size_t proto_format_config(char *buf, const char *name, long value)
{
    if (name == NULL || *name == '\0')
        return 0;
    for (const char *p = name; *p; p++)
        if (!((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9') || *p == '_'))
            return 0;
    int n = snprintf(buf, 64, "CONFIG %s %ld\n", name, value);
    return n > 0 && n < 64 ? (size_t)n : 0;
}

const char *proto_mode_name(fw_mode_t m)
{
    switch (m) {
    case FW_DISABLED:
        return "DISABLED";
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
    case FW_REASON_AS5600:
        return "AS5600";
    case FW_REASON_MT6835:
        return "MT6835";
    case FW_REASON_POWER:
        return "POWER";
    default:
        return "?";
    }
}

/* 此解析器独立检查语法，不通过重新格式化来证明格式化器自身正确。 */
static bool parse_command_integer(const char *text, long *value)
{
    const char *digits = *text == '-' ? text + 1 : text;
    if (*digits < '0' || *digits > '9' ||
        (*digits == '0' && (digits[1] != '\n' || digits != text)))
        return false;
    const char *end = digits;
    while (*end >= '0' && *end <= '9')
        end++;
    if (*end != '\n' || end[1] != '\0')
        return false;
    errno = 0;
    char *parsed_end;
    long result = strtol(text, &parsed_end, 10);
    if (errno == ERANGE || parsed_end != end)
        return false;
    *value = result;
    return true;
}
bool proto_parse_command(const char *line, proto_command_t *out)
{
    if (!line || !out || strlen(line) >= 64)
        return false;
    proto_command_t parsed = {0};
    if (strncmp(line, "VEL ", 4) == 0) {
        parsed.kind = PROTO_COMMAND_VELOCITY;
        if (!parse_command_integer(line + 4, &parsed.value) ||
            parsed.value < -1000000000L || parsed.value > 1000000000L)
            return false;
    } else if (strncmp(line, "CONFIG ", 7) == 0) {
        parsed.kind = PROTO_COMMAND_CONFIGURE;
        const char *name = line + 7, *end = name;
        while ((*end >= 'A' && *end <= 'Z') || (*end >= '0' && *end <= '9') || *end == '_')
            end++;
        size_t length = (size_t)(end - name);
        if (!length || length >= sizeof parsed.parameter || *end != ' ' ||
            !parse_command_integer(end + 1, &parsed.value))
            return false;
        memcpy(parsed.parameter, name, length);
    } else {
        static const struct { const char *line; proto_command_kind_t kind; } simple[] = {
            {"QUERY\n", PROTO_COMMAND_QUERY}, {"CALIB?\n", PROTO_COMMAND_CALIBRATION_QUERY},
            {"CONFIG?\n", PROTO_COMMAND_CONFIGURATION_QUERY}, {"ENABLE\n", PROTO_COMMAND_ENABLE},
            {"DISABLE\n", PROTO_COMMAND_DISABLE}, {"RESET\n", PROTO_COMMAND_RESET},
            {"MARK LEFT\n", PROTO_COMMAND_MARK_LEFT}, {"MARK RIGHT\n", PROTO_COMMAND_MARK_RIGHT},
            {"CONFIRM LEFT\n", PROTO_COMMAND_CONFIRM_LEFT}
        };
        bool found = false;
        for (size_t i = 0; i < sizeof simple / sizeof *simple; i++) {
            if (strcmp(line, simple[i].line) == 0) {
                parsed.kind = simple[i].kind;
                found = true;
                break;
            }
        }
        if (!found)
            return false;
    }
    *out = parsed;
    return true;
}
