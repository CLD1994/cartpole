/* 台位会话：串口 + 协议接收 + 遥测快照 + 模式跟踪 + 共用运动流程。
 *
 * run / console / 一次性命令三种运行形态共用同一个进程内会话：
 *   - 行处理统一在这里：STATE 更新快照，MODE 维护最后已知模式并打印转换，
 *     OK / ERR 打印回执并记录最近一条应答；
 *   - 停机请求（信号处理器调用 rig_request_stop）经这里传播，
 *     所有等待与保持循环统一可中断。
 */
#ifndef EDGE_RIG_H
#define EDGE_RIG_H

#include <stdint.h>

#include "config.h"
#include "proto.h"

void rig_request_stop(void); /* 信号处理器调用 */
int rig_stop_requested(void);

int rig_open(const char *port, int baud); /* 失败返回 -1 */
void rig_close(void);

void rig_pump(void); /* 泵串口：非阻塞读 + 行分发 */

/* 发送。失败返回 -1；VEL 数值非法（NaN/超界）同样返回 -1 且不发送。 */
int rig_send_vel(double v_mm_s);
int rig_send_simple(const char *cmd); /* HOME / RESET / QUERY / CONFIG? */
int rig_send_config(const char *name, long value);

int rig_sync_mode(double timeout_s);    /* 发 QUERY 等模式已知；超时返回 -1 */
int rig_wait_state(double timeout_s);   /* 等第一条 STATE；超时返回 -1 */
int rig_push_params(void);              /* 推送 CFG_FW_PARAMS 表（间隔 10 ms） */
int rig_wait_ready(double timeout_s);   /* 发 HOME 并等 READY；FAULT/超时返回 -1 */
int rig_wait_mode(fw_mode_t target, double timeout_s); /* 只等待不发命令 */
int rig_goto_center(double timeout_s);  /* 以 CFG_CENTER_VEL_MMS 移回行程中点 */

/* 等待新的 OK/ERR 应答：0 = OK，1 = ERR，-1 = 超时（应答本身由行处理打印）。 */
int rig_wait_reply(double timeout_s);

/* 快照访问（出参均可为 NULL）。mode 在模式未知时返回 FW_DISABLED。 */
fw_mode_t rig_mode(int *known, fw_reason_t *reason);
void rig_telemetry(int64_t *x_um, int64_t *theta_urad, int64_t *vx_mm_s,
                   int64_t *vth_mrad_s);
double rig_state_age_s(void); /* 距最近 STATE 的秒数；从未收到返回 -1 */
double rig_now_s(void);       /* CLOCK_MONOTONIC 秒 */

void rig_print_state(void); /* 打印一行最新遥测快照（协议原单位） */

#endif /* EDGE_RIG_H */
