/* 策略推理桩：未链接 NPU 库时（主机测试/联调早期）返回 0 动作并告警一次。
 * 交叉编译接真库时替换本文件或链接真实现。 */

#include "policy.h"

#include <stdio.h>

static int g_loaded = 0;

int policy_init(const char *model_path)
{
    /* TODO: 替换为 NPU 库真实现（SVP_NPU 封装，加载 .om） */
    fprintf(stderr, "[policy] STUB: model '%s' not loaded (NPU library not linked)\n", model_path);
    g_loaded = 1; /* 桩模式下放行，便于串口链路联调 */
    return 0;
}

float policy_forward(const float obs[4])
{
    (void)obs;
    if (!g_loaded) {
        return 0.0f;
    }
    return 0.0f; /* 零动作：小车保持静止，链路与安全逻辑仍可验证 */
}

void policy_deinit(void)
{
    g_loaded = 0;
}
