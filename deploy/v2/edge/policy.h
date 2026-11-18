#ifndef EDGE_POLICY_H
#define EDGE_POLICY_H
/* 显式选择后端，不自动切换精度或后端。 */
typedef enum policy_backend { POLICY_CPU=1, POLICY_NPU=2 } policy_backend;
int policy_init(policy_backend backend, const char *model_path);
/* 模型坐标 [x,theta,vx,omega]，SI 单位；输出未经缩放。失败将 action 置零，
 * 并禁止继续推理，直至释放。接口只供单线程调用。 */
int policy_forward(const float obs[4], float *action);
void policy_deinit(void);
#endif
