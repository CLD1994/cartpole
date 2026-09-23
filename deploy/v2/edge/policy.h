/* NPU 策略推理封装（cartpole_npu 库，见 npu/）：float obs[4] -> float action。
 *
 * 量化封装在库内，控制程序只见到 float。库要求调用方在同一线程先行初始化
 * SVP ACL 运行时并持有其生命周期，本封装承担该职责（初始化与退出梯子同
 * npu/src/replay.c 参考实现一致）。单线程调用，进程生命周期只初始化一次。
 */
#ifndef EDGE_POLICY_H
#define EDGE_POLICY_H

/* 初始化 SVP ACL 运行时（svp_acl_init + svp_acl_rt_set_device(0)）并加载
 * .om 模型。成功返回 0；失败返回非 0 且内部已按契约清理，程序应启动即退出，
 * 不进控制环。 */
int policy_init(const char *model_path);

/* 同步推理：obs = [cart_pos m, pole_angle rad, cart_vel m/s, pole_vel rad/s]，
 * 顺序与符号与训练一致（sim2real-spec）。成功返回 0 且 *action 为未缩放的
 * 原始动作（训练约定 [-1,1]，调用方仍钳位）；失败返回非 0 且 *action 置 0。
 * 库契约：推理失败后句柄失效，只能经 policy_deinit 释放，不得继续推理。 */
int policy_forward(const float obs[4], float *action);

/* 释放模型与运行时（cartpole_npu_close -> svp_acl_rt_reset_device ->
 * svp_acl_finalize；close 未完成时不释放运行时，由进程退出兜底）。
 * 可重复调用；close 失败后再次调用会重试 close。 */
void policy_deinit(void);

#endif /* EDGE_POLICY_H */
