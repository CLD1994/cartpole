/* NPU 推理库接口（float obs[4] -> float action）。
 *
 * 接口契约（端侧架构讨论确定）：量化封装在库内，控制程序只见到 float。
 * 真实现由另一项目的 SVP_NPU 封装库提供；交叉编译时链接它。
 */
#ifndef EDGE_POLICY_H
#define EDGE_POLICY_H

/* 加载 .om 模型。成功返回 0，失败返回非 0（程序启动即失败退出，不进控制环）。 */
int policy_init(const char *model_path);

/* 同步推理：obs = [cart_pos m, pole_angle rad, cart_vel m/s, pole_vel rad/s]，
 * 顺序与符号与训练一致（sim2real-spec）。返回动作（训练约定 [-1,1]，调用方仍钳位）。 */
float policy_forward(const float obs[4]);

void policy_deinit(void);

#endif /* EDGE_POLICY_H */
