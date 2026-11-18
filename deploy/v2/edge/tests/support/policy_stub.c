/* 仅测试程序链接。生产 CMake 目标没有此源文件。 */
#include "policy.h"
#include <math.h>
#include <stdio.h>
#ifndef EDGE_TEST_STUB
#error "policy_stub.c is test-only"
#endif
int policy_init(const char *path) {
    (void)path;
    fputs("[TEST ONLY] NPU stub: never deploy this binary\n", stderr);
    return 0;
}
int policy_forward(const float obs[4], float *action) {
    for (int i=0;i<4;i++) if (!isfinite(obs[i])) return -1;
    *action=0.25f;
    return 0;
}
void policy_deinit(void) {}
