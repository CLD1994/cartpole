#ifndef EDGE_POLICY_NPU_H
#define EDGE_POLICY_NPU_H
int npu_policy_init(const char *path);
int npu_policy_forward(const float obs[4], float *action);
void npu_policy_deinit(void);
int npu_policy_active(void);
#endif
