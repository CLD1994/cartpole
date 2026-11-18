#ifndef EDGE_PACKAGE_H
#define EDGE_PACKAGE_H
#include <stddef.h>
/* 模型包由 prepare_package.py 从正式 model.json 派生，不手工维护参数。 */
typedef struct edge_package {
    char model_cpu_path[4096];
    char model_npu_path[4096];
    char sha256_cpu[65];
    char sha256_npu[65];
    double control_hz;
    int action_scale_mm_s;
    int velocity_limit_mm_s;
    int acceleration_mm_s2;
    int angle_sign;
    double pause_deg;
    double resume_deg;
} edge_package;
/* 返回0成功，-1失败；失败时清空输出并在err中保留原因。 */
int edge_package_load(const char *manifest_path, edge_package *out, char *err, size_t err_size);
#endif
