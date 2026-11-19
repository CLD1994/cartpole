#ifndef EDGE_SERIAL_CARTPOLE_H
#define EDGE_SERIAL_CARTPOLE_H
#include "cartpole_device.h"
/* 串口适配器拥有串口、组帧和固件回执状态；业务会话只借用它提供的设备能力。
 * 必须先销毁业务会话，再close；close不发送停车指令。每次open创建独立接收状态。
 */
typedef struct serial_cartpole serial_cartpole_t;
typedef struct {
    uint64_t received_measurements, unknown_received_lines;
} serial_cartpole_statistics_t;
serial_cartpole_t *serial_cartpole_open(const char *port, int baud, const cartpole_clock_t *clock);
/* CLI打开并绑定取消源后调用一次，只接收，启动最多等待25 ms。没有STATE时
 * 仍可在独立的事务时限内发送命令，已运行会话的清理也不重新同步。 */
void serial_cartpole_sync_startup(serial_cartpole_t *);
cartpole_device_t serial_cartpole_get_device(serial_cartpole_t *);
void serial_cartpole_close(serial_cartpole_t *);
/* 诊断统计是主机成功解析的测量及未知行累计数，不表示设备丢包数量。 */
serial_cartpole_statistics_t serial_cartpole_get_statistics(const serial_cartpole_t *);
#endif
