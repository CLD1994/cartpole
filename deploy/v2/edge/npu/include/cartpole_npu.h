#ifndef CARTPOLE_NPU_H
#define CARTPOLE_NPU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cartpole_npu cartpole_npu;

typedef enum {
    CARTPOLE_OK = 0,
    CARTPOLE_ARGUMENT,
    CARTPOLE_IO,
    CARTPOLE_MODEL,
    CARTPOLE_SDK,
    CARTPOLE_STATE,
    CARTPOLE_MEMORY
} cartpole_status;

typedef struct {
    cartpole_status code;
    int32_t sdk_code;
    const char *stage;
    int32_t cleanup_code;
    const char *cleanup_stage;
} cartpole_error;

/* 调用者在同一线程初始化 SVP ACL 并设置设备 0；所有调用顺序执行。
 * error 可为 NULL。库不 reset_device/finalize，也不拥有调用者的运行时。
 * *handle 必须为 NULL；失败清理若再次失败，保留仅可 close 的句柄。
 */
cartpole_status cartpole_npu_open(const char *model_path, cartpole_npu **handle, cartpole_error *error);

/* obs 顺序：小车位置、杆角度、小车速度、杆角速度，全部为有限 float。
 * 返回未经缩放、限幅的原始动作；失败时非空 action 被设为 NaN。
 * SDK 执行失败或非有限输出使句柄失效，只能 close。
 * infer 不分配内存、不读写文件；obs 和 action 不得重叠。
 */
cartpole_status cartpole_npu_infer(cartpole_npu *handle, const float obs[4], float *action, cartpole_error *error);

/* 成功置 *handle 为 NULL，NULL 句柄幂等。
 * 失败保留尚未释放的资源，可再次 close；不得提前 finalize 运行时。
 */
cartpole_status cartpole_npu_close(cartpole_npu **handle, cartpole_error *error);

#ifdef __cplusplus
}
#endif
#endif
