/* 策略推理封装：把 cartpole_npu（.om 模型执行）接入 policy.h 接口。
 *
 * 本文件同时承担库契约要求的 SVP ACL 运行时生命周期，顺序与错误处理
 * 同库参考程序 npu/src/replay.c 一致：
 *   svp_acl_init(NULL) -> svp_acl_rt_set_device(0) -> cartpole_npu_open
 *   -> cartpole_npu_infer* -> cartpole_npu_close
 *   -> svp_acl_rt_reset_device(0) -> svp_acl_finalize
 * close 未完成（句柄仍非 NULL）时不 reset/finalize，资源由进程退出回收。
 */

#include "policy.h"

#include "cartpole_npu.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#if defined(__has_include)
#if __has_include(<acl/svp_acl.h>)
#include <acl/svp_acl.h>
#define POLICY_HAVE_SVP_ACL 1
#endif
#endif
#ifndef POLICY_HAVE_SVP_ACL
/* 主机单元测试没有 SDK 头时的最小声明：svp_acl_error 为 int32 错误码
 * （0 成功），签名与 acl/svp_acl.h 一致；交叉编译经 -I 指向 SDK 后由
 * 真实头文件取代本分支。 */
typedef int32_t svp_acl_error;
svp_acl_error svp_acl_init(const char *config_path);
svp_acl_error svp_acl_rt_set_device(int32_t device_id);
svp_acl_error svp_acl_rt_reset_device(int32_t device_id);
svp_acl_error svp_acl_finalize(void);
#endif

static cartpole_npu *g_handle; /* 库句柄：open 置位，close 成功后置 NULL */
static int g_runtime_init;     /* svp_acl_init 已成功 */
static int g_device_set;       /* svp_acl_rt_set_device(0) 已成功 */
static int g_dead;             /* 推理失败标志：库契约规定此后句柄只能 close */

static const char *status_name(cartpole_status code)
{
    switch (code) {
    case CARTPOLE_OK: return "OK";
    case CARTPOLE_ARGUMENT: return "ARGUMENT";
    case CARTPOLE_IO: return "IO";
    case CARTPOLE_MODEL: return "MODEL";
    case CARTPOLE_SDK: return "SDK";
    case CARTPOLE_STATE: return "STATE";
    case CARTPOLE_MEMORY: return "MEMORY";
    default: return "?";
    }
}

static void report_error(const char *where, const cartpole_error *err)
{
    fprintf(stderr, "[policy] %s: %s stage=%s sdk=%" PRId32 "\n", where,
            status_name(err->code), err->stage ? err->stage : "unknown", err->sdk_code);
    if (err->cleanup_stage) {
        fprintf(stderr, "[policy] cleanup: %s stage=%s\n",
                status_name(err->cleanup_code), err->cleanup_stage);
    }
}

int policy_init(const char *model_path)
{
    cartpole_error err;
    svp_acl_error rc;

    if (g_handle != NULL || g_runtime_init) {
        return -1; /* 进程生命周期只初始化一次 */
    }
    rc = svp_acl_init(NULL);
    if (rc != 0) {
        fprintf(stderr, "[policy] svp_acl_init failed sdk=%" PRId32 "\n", rc);
        return -1; /* 运行时未建立，无需清理 */
    }
    g_runtime_init = 1;

    rc = svp_acl_rt_set_device(0);
    if (rc != 0) {
        fprintf(stderr, "[policy] svp_acl_rt_set_device failed sdk=%" PRId32 "\n", rc);
        policy_deinit();
        return -1;
    }
    g_device_set = 1;

    if (cartpole_npu_open(model_path, &g_handle, &err) != CARTPOLE_OK) {
        report_error("open", &err);
        policy_deinit();
        return -1;
    }
    return 0;
}

int policy_forward(const float obs[4], float *action)
{
    cartpole_error err;

    if (action == NULL) {
        return -1;
    }
    *action = 0.0f;
    if (g_handle == NULL || g_dead) {
        return -1; /* 未初始化，或推理已失败（句柄只能 close） */
    }
    if (cartpole_npu_infer(g_handle, obs, action, &err) != CARTPOLE_OK) {
        report_error("infer", &err);
        g_dead = 1;
        *action = 0.0f;
        return -1;
    }
    if (!isfinite(*action)) {
        /* 库契约：非有限输出同样使句柄失效 */
        fprintf(stderr, "[policy] infer returned non-finite action\n");
        g_dead = 1;
        *action = 0.0f;
        return -1;
    }
    return 0;
}

void policy_deinit(void)
{
    cartpole_error err;
    svp_acl_error rc;

    if (cartpole_npu_close(&g_handle, &err) != CARTPOLE_OK) {
        report_error("close", &err);
        /* 未释放的句柄仍依赖运行时：不 reset/finalize（同 replay.c）。
         * 资源由进程退出回收；再次调用本函数会重试 close。 */
    }
    if (g_handle != NULL) {
        return;
    }
    if (g_device_set) {
        rc = svp_acl_rt_reset_device(0);
        if (rc != 0) {
            fprintf(stderr, "[policy] svp_acl_rt_reset_device failed sdk=%" PRId32 "\n", rc);
            return; /* 设备未释放，不得 finalize */
        }
        g_device_set = 0;
    }
    if (g_runtime_init) {
        rc = svp_acl_finalize();
        if (rc != 0) {
            fprintf(stderr, "[policy] svp_acl_finalize failed sdk=%" PRId32 "\n", rc);
        }
        g_runtime_init = 0;
    }
    g_dead = 0;
}
