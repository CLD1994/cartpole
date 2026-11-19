/* ctypes 测试读取公共枚举，避免在 Python 中复制其数值。 */
#include "policy.h"
int test_policy_cpu_backend(void) { return POLICY_CPU; }
int test_policy_npu_backend(void) { return POLICY_NPU; }
