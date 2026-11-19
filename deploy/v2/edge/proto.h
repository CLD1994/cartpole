/* 行协议：解析固件发来的 STATE/MODE/OK/ERR 行，构造发往固件的命令行。
 *
 * 纯字符串处理，不接触串口 —— 解析与格式化可在主机上单测（proto_test.c）。
 * 契约：docs/deployment/protocol.md。
 */
#ifndef EDGE_PROTO_H
#define EDGE_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 固件模式（MODE 行第一个字段） */
typedef enum {
    FW_DISABLED = 0,
    FW_READY,
    FW_FAULT,
} fw_mode_t;

/* MODE 行第二个字段：进入当前模式的原因 */
typedef enum {
    FW_REASON_NONE = 0,
    FW_REASON_STEP_LOSS,
    FW_REASON_AS5600,
    FW_REASON_MT6835,
} fw_reason_t;

/* 解析结果类型 */
typedef enum {
    PROTO_UNKNOWN = 0, /* 无法解析的行：静默丢弃 */
    PROTO_STATE,       /* STATE <flags> <x_um> <theta_urad> <vx_mm_s> <vth_mrad_s> */
    PROTO_MODE,        /* MODE <mode> <reason> */
    PROTO_OK,          /* OK <NAME> <value> */
    PROTO_ERR,         /* ERR <NAME> */
    PROTO_STEP_LOSS,   /* EVENT STEP_LOSS <event> <run> <device_us> */
} proto_msg_type_t;

typedef struct {
    proto_msg_type_t type;
    /* PROTO_STATE */
    unsigned valid_flags; /* bit 0 小车，bit 1 摆杆；无效路必须为零占位 */
    int64_t x_um;         /* µm   */
    int64_t theta_urad;   /* µrad */
    int64_t vx_mm_s;      /* mm/s */
    int64_t vth_mrad_s;   /* mrad/s */
    /* PROTO_MODE */
    fw_mode_t mode;
    fw_reason_t reason;
    /* PROTO_OK / PROTO_ERR */
    char param[16]; /* 参数名，NUL 结尾 */
    int64_t value;  /* PROTO_OK 的回显值 */
    int64_t event_id, run_id, device_us;
} proto_msg_t;

/* 解析一行（不含换行符）。返回消息类型；无法解析时返回 PROTO_UNKNOWN 且不写 msg。 */
proto_msg_type_t proto_parse_line(const char *line, proto_msg_t *msg);

/* 行缓冲累积器：喂入任意长度的原始字节，每凑成完整一行（'\n'）回调一次。
 * 超长行或嵌入NUL以 !OVERLONG 非法行回调，不接受截断前缀；可选 '\r' 剥离；
 * 不完整尾部保留等待下次拼接。回调返回值忽略。 */
typedef void (*proto_line_cb)(const char *line, size_t len, void *user);

typedef struct {
    char buf[160];
    size_t len;
    bool overflow;
} proto_rx_t;

void proto_rx_init(proto_rx_t *rx);
void proto_rx_feed(proto_rx_t *rx, const uint8_t *data, size_t n, proto_line_cb cb, void *user);

/* 命令构造（写入 buf，返回长度，含 '\n'；buf 至少 64 字节）。
 * 数值非法（NaN/超出 int 范围）时返回 0，调用方跳过发送。 */
size_t proto_format_vel(char *buf, double v_mm_s);
size_t proto_format_simple(char *buf, const char *cmd); /* RESET / QUERY / MARK LEFT / ENABLE */
size_t proto_format_config(char *buf, const char *name, long value);

/* 命令线路格式的只读校验/提取接口，可供记录分析器及协议消费者使用。
 * 输入是一个完整、NUL结尾、含LF的命令；拒绝额外行、非规范整数及未知命令。
 * 不发送命令，不改变设备；失败返回false且不改out。 */
typedef enum {
    PROTO_COMMAND_VELOCITY, PROTO_COMMAND_CONFIGURE, PROTO_COMMAND_QUERY,
    PROTO_COMMAND_CALIBRATION_QUERY, PROTO_COMMAND_CONFIGURATION_QUERY,
    PROTO_COMMAND_ENABLE, PROTO_COMMAND_DISABLE, PROTO_COMMAND_RESET,
    PROTO_COMMAND_MARK_LEFT, PROTO_COMMAND_MARK_RIGHT
} proto_command_kind_t;
typedef struct {
    proto_command_kind_t kind;
    char parameter[64]; /* 仅CONFIG；受整条命令的63字节上限约束 */
    long value;         /* VEL整数mm/s，或CONFIG整数值 */
} proto_command_t;
bool proto_parse_command(const char *line, proto_command_t *out);

/* 模式/原因枚举的名字（日志用） */
const char *proto_mode_name(fw_mode_t m);
const char *proto_reason_name(fw_reason_t r);

#endif /* EDGE_PROTO_H */
