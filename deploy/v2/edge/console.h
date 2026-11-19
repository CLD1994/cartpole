/* 交互控制台：解析用户命令并使用 cartpole 业务事务和循环。 */
#ifndef EDGE_CONSOLE_H
#define EDGE_CONSOLE_H
#include "cartpole.h"

typedef enum {
    CONSOLE_CMD_NONE = 0, /* 空行 */
    CONSOLE_CMD_QUIT,
    CONSOLE_CMD_HELP,
    CONSOLE_CMD_CALIB,
    CONSOLE_CMD_MARK_LEFT,
    CONSOLE_CMD_MARK_RIGHT,
    CONSOLE_CMD_ENABLE,
    CONSOLE_CMD_CENTER,
    CONSOLE_CMD_VEL,
    CONSOLE_CMD_STATE,
    CONSOLE_CMD_QUERY,
    CONSOLE_CMD_CONFIG,
    CONSOLE_CMD_RESET,
} console_cmd_t;

typedef struct {
    console_cmd_t cmd;
    double vel_mm_s; /* CONSOLE_CMD_VEL：目标速度 mm/s */
    char name[16];   /* CONSOLE_CMD_CONFIG：参数名（协议注册表词汇，大写） */
    long value;      /* CONSOLE_CMD_CONFIG：参数值 */
    int dump;        /* CONSOLE_CMD_CONFIG 无参数：CONFIG? 全量 dump */
    char err[80];    /* 解析失败原因（仅失败时有效） */
} console_req_t;

/* 解析一行命令（不含换行符，允许首尾与分隔空白）。
 * 成功返回 0；无法解析返回 -1 并填写 err。命令词大小写敏感，全小写。 */
int console_parse(const char *line, console_req_t *req);

/* 执行单条非保持命令。返回 0 成功，-1 失败（诊断已打印）。
 * 仅处理 CALIB/MARK_LEFT/MARK_RIGHT/ENABLE/CENTER/STATE/QUERY/CONFIG/RESET；
 * 其余命令返回 -1。 */
int console_exec(cartpole_t *c, const console_req_t *req);

/* 进入交互循环；会话须已打开并同步。失败返回非零，main统一确认停车。 */
int console_run(cartpole_t *c);

int console_transaction_result(const char *operation, cartpole_result_t result);
void console_print_state(cartpole_t *c);
int console_goto_center(cartpole_t *c, double timeout_s);

#endif /* EDGE_CONSOLE_H */
