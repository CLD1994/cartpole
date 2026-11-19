#ifndef EDGE_CPU_POLICY_H
#define EDGE_CPU_POLICY_H
int cpu_policy_init(const char *path);
int cpu_policy_forward(const float obs[4], float *action);
void cpu_policy_deinit(void);
#endif
