/* 端侧控制程序常量（新装置 v2）。
 *
 * 权威关系：固件参数（VEL_MAX/ACC_MAX 等）的会话权威在这里，启动时经 CONFIG 命令推送；
 * 协议行为契约见 deploy/v2/protocol.md，仿真契约见 deploy/v2/sim2real-spec.md。
 * 换台位只改本文件。
 */
#ifndef EDGE_CONFIG_H
#define EDGE_CONFIG_H

/* ---- 串口 ---- */
#define CFG_SERIAL_PORT     "/dev/ttyAMA1" /* CV610 上连 ESP32-S3 的 UART 设备节点，待硬件定 */
#define CFG_BAUD            460800

/* ---- 控制环 ---- */
#define CFG_CONTROL_HZ      200.0     /* 与训练 dt 一致（sim2real-spec） */
#define CFG_STATE_TIMEOUT_S 0.2       /* 超过此时长无 STATE 行：发 VEL 0 并等待恢复 */

/* ---- 动作映射（策略输出 [-1,1] -> 目标速度） ---- */
#define CFG_V_MAX_MPS       1.0       /* 训练随机化上界（0.8–1.0），部署取上界 */

/* ---- 安全 ---- */
#define CFG_TRACK_LIMIT_M   0.260     /* 端侧软限位（与固件 X_MARGIN_UM 窗口各自独立，再挡一层向外的速度） */
#define CFG_ENGAGE_ANGLE_RAD 0.524    /* 策略接管门限：|theta| 超过 30° 发 VEL 0（摆杆下垂/大倾角不驱动） */
#define CFG_CENTER_VEL_MMS  100       /* 标定 ENABLE 后把小车从端点移回行程中点的速度 */

/* ---- 会话启动时推送的固件参数（CONFIG 命令，见 protocol.md 注册表） ---- */
struct fw_param {
    const char *name;
    long value;
};

#define CFG_FW_PARAM_COUNT  4
extern const struct fw_param CFG_FW_PARAMS[CFG_FW_PARAM_COUNT];

#endif /* EDGE_CONFIG_H */
