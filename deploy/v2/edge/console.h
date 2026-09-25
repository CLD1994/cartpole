/* 交互控制台：无策略模型时的人工操作窗口（维护与板端联调）。
 *
 * console_parse 是纯字符串处理（主机可单测，console_test.c）；console_exec
 * 执行单条命令的非保持语义（calib/mark-left/mark-right/enable/center/state/
 * query/config/reset），一次性子命令与控制台共用；console_run 假设 rig 已
 * 打开并完成模式同步，进入行循环：stdin 命令、串口泵与 vel 目标保持合并
 * 在一个 select 循环里，退出（quit/Ctrl+D/信号）前发 VEL 0。
 */
#ifndef EDGE_CONSOLE_H
#define EDGE_CONSOLE_H

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
int console_exec(const console_req_t *req);

/* 进入交互循环（rig 已打开并同步模式）。始终返回 0。 */
int console_run(void);

#endif /* EDGE_CONSOLE_H */
