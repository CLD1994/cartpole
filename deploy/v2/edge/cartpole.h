/* 台位会话：串口 + 协议接收 + 遥测快照 + 模式跟踪 + 共用运动流程。
 *
 * run / console / 一次性命令三种运行形态共用同一个进程内会话：
 *   - 行处理统一在这里：STATE 更新快照，MODE 维护最后已知模式并打印转换，
 *     OK / ERR 打印回执并记录最近一条应答；
 *   - 停机请求（信号处理器调用 cartpole_request_stop）经这里传播，
 *     所有等待与保持循环统一可中断。
 */
#ifndef EDGE_CARTPOLE_H
#define EDGE_CARTPOLE_H

#include <stdint.h>

#include "config.h"
#include "proto.h"

void cartpole_request_stop(void); /* 信号处理器调用 */
int cartpole_stop_requested(void);

int cartpole_open(const char *port, int baud); /* 失败返回 -1 */
void cartpole_close(void);

/* 单次最多读取4096字节：0=已读空，1=预算耗尽，-1=I/O或协议错误。 */
int cartpole_pump(void);

/* 发送。失败返回 -1；VEL 数值非法（NaN/超界）同样返回 -1 且不发送。 */
int cartpole_send_vel(double v_mm_s);
/* 策略动作：deadline前仅尝试一次非阻塞写。0=完整写且返回时未过期；
 * 1=未写且丢弃；2=已完整写入但返回时已过期（须停止，不得宣称丢弃）；-1=错误。
 * 完整写入只证明内核接受，不证明设备接收时刻。 */
int cartpole_try_send_vel(double v_mm_s, double deadline_s);
int cartpole_send_simple(const char *cmd); /* RESET / QUERY / CONFIG? / MARK LEFT / ENABLE */
int cartpole_send_config(const char *name, long value);

int cartpole_sync_mode(double timeout_s);  /* 发QUERY等待边界后的新MODE；超时返回-1 */
int cartpole_wait_state(double timeout_s); /* 等调用后的新STATE；超时返回-1 */
int cartpole_enable(double timeout_s);
int cartpole_configure(const char *name, long value, double timeout_s);
int cartpole_refresh_metadata(double timeout_s);
/* 仅取最近refresh/configure回执缓存，timeout参数保留与查询调用统一。 */
int cartpole_get_config(const char *name, int64_t *value, double timeout_s);
/* 0=新确认DISABLED，1=新观察FAULT，-1=未确认；FAULT不是DISABLED。 */
int cartpole_stop_and_confirm(double timeout_s);
int cartpole_valid_flags(void);
uint64_t cartpole_state_sequence(void);
int cartpole_io_error(void);
/* 每条STATE/MODE/OK更新后同步调用；hook只观察状态，不发送命令或重入pump。 */
void cartpole_set_update_hook(void (*hook)(void *), void *context);
double cartpole_state_received_at_s(void); /* 原始CLOCK_MONOTONIC接收时刻；未收到为-1。 */
int cartpole_fault_seen(void);             /* 当前串口会话曾观察FAULT，恢复READY也不清除。 */
int cartpole_wait_mode(fw_mode_t target, double timeout_s); /* 只等待新的MODE，不发命令 */
int cartpole_goto_center(double timeout_s);                 /* 以 CFG_CENTER_VEL_MMS 移回行程中点 */

/* 等待新的 OK/ERR 应答：0 = OK，1 = ERR，-1 = 超时（应答本身由行处理打印）。 */
int cartpole_wait_reply(double timeout_s);

/* 快照访问（出参均可为 NULL）。mode 在模式未知时返回 FW_DISABLED。 */
fw_mode_t cartpole_mode(int *known, fw_reason_t *reason);
void cartpole_telemetry(int64_t *x_um, int64_t *theta_urad, int64_t *vx_mm_s, int64_t *vth_mrad_s);
double cartpole_state_age_s(void); /* 距最近 STATE 的秒数；从未收到返回 -1 */
double cartpole_now_s(void);       /* CLOCK_MONOTONIC 秒 */

void cartpole_print_state(void); /* 打印一行最新遥测快照（原始整数 + 可读换算） */

void cartpole_window_reset(void);  /* 清零窗口计数器 */
long cartpole_state_count(void);   /* 窗口内 STATE 行数 */
long cartpole_unknown_count(void); /* 窗口内无法解析行数 */

#endif /* EDGE_CARTPOLE_H */
