/* policy.c 的主机端单元测试：用链接级替身验证封装的状态机，不接触 NPU/SDK。
 *
 * 替身按真实接口约束实现：
 *  - cartpole_npu_* 严格按 npu/include/cartpole_npu.h 的注释语义（*handle 必须
 *    为 NULL、失败清理失败保留 close-only 句柄、infer 失败置 NaN、close 幂等）；
 *  - svp_acl_* 与 policy.c 的最小声明签名一致，记录调用顺序供断言。
 * 每条测试覆盖一个可证伪契约分支；每个测试结束时封装状态必须回到未初始化。
 */
#include "policy.h"

#include "cartpole_npu.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---- 事件记录 ---- */
enum {
    EV_ACL_INIT,
    EV_SET_DEVICE,
    EV_RESET_DEVICE,
    EV_ACL_FINALIZE,
    EV_OPEN,
    EV_INFER,
    EV_CLOSE,
};

static int g_events[64];
static int g_event_count;

static void ev(int e)
{
    if (g_event_count < (int)(sizeof(g_events) / sizeof(g_events[0]))) {
        g_events[g_event_count++] = e;
    }
}

static int count(int e)
{
    int n = 0;
    for (int i = 0; i < g_event_count; i++) {
        if (g_events[i] == e) {
            n++;
        }
    }
    return n;
}

static int first_index(int e)
{
    for (int i = 0; i < g_event_count; i++) {
        if (g_events[i] == e) {
            return i;
        }
    }
    return -1;
}

/* A 在 B 之前发生（两者都存在） */
static int before(int a, int b)
{
    int ia = first_index(a), ib = first_index(b);
    return ia >= 0 && ib >= 0 && ia < ib;
}

/* ---- svp_acl 运行时替身：签名与 policy.c 的最小声明一致 ---- */
static int32_t g_acl_init_ret, g_set_device_ret, g_reset_device_ret, g_finalize_ret;

int32_t svp_acl_init(const char *config_path)
{
    (void)config_path;
    ev(EV_ACL_INIT);
    return g_acl_init_ret;
}

int32_t svp_acl_rt_set_device(int32_t device_id)
{
    (void)device_id;
    ev(EV_SET_DEVICE);
    return g_set_device_ret;
}

int32_t svp_acl_rt_reset_device(int32_t device_id)
{
    (void)device_id;
    ev(EV_RESET_DEVICE);
    return g_reset_device_ret;
}

int32_t svp_acl_finalize(void)
{
    ev(EV_ACL_FINALIZE);
    return g_finalize_ret;
}

/* ---- cartpole_npu 替身：按头文件注释语义 ---- */
static char g_fake_storage[8];
static cartpole_npu *const g_fake = (cartpole_npu *)g_fake_storage;

static cartpole_status g_open_ret = CARTPOLE_OK;
static int g_open_leave_handle;   /* open 失败且内部清理失败：保留 close-only 句柄 */
static cartpole_status g_infer_ret = CARTPOLE_OK;
static int g_infer_inf_output;    /* 成功返回但动作为无穷大（库外部防御分支） */
static int g_close_fail_times;    /* 前 N 次 close 失败（资源保留，可重试） */
static int g_infer_calls;

static void fill_error(cartpole_error *err, cartpole_status code, const char *stage)
{
    if (err != NULL) {
        err->code = code;
        err->sdk_code = 7;
        err->stage = stage;
        err->cleanup_code = CARTPOLE_OK;
        err->cleanup_stage = NULL;
    }
}

cartpole_status cartpole_npu_open(const char *model_path, cartpole_npu **handle, cartpole_error *error)
{
    (void)model_path;
    ev(EV_OPEN);
    if (handle == NULL || *handle != NULL) {
        fill_error(error, CARTPOLE_ARGUMENT, "open_argument");
        return CARTPOLE_ARGUMENT; /* 契约：*handle 必须为 NULL */
    }
    if (g_open_ret != CARTPOLE_OK) {
        if (g_open_leave_handle) {
            *handle = g_fake; /* 内部清理失败的残留：仅可 close */
            fill_error(error, g_open_ret, "open_model");
            error->cleanup_code = CARTPOLE_STATE;
            error->cleanup_stage = "free_model";
        } else {
            fill_error(error, g_open_ret, "open_model");
        }
        return g_open_ret;
    }
    *handle = g_fake;
    return CARTPOLE_OK;
}

cartpole_status cartpole_npu_infer(cartpole_npu *handle, const float obs[4], float *action, cartpole_error *error)
{
    ev(EV_INFER);
    g_infer_calls++;
    if (handle != g_fake) {
        if (action != NULL) {
            *action = NAN;
        }
        fill_error(error, CARTPOLE_STATE, "session_failed");
        return CARTPOLE_STATE;
    }
    for (int i = 0; i < 4; i++) {
        if (!isfinite(obs[i])) { /* 契约：观测必须全部有限 */
            if (action != NULL) {
                *action = NAN;
            }
            fill_error(error, CARTPOLE_ARGUMENT, "observation_nonfinite");
            return CARTPOLE_ARGUMENT;
        }
    }
    if (g_infer_ret != CARTPOLE_OK) {
        if (action != NULL) {
            *action = NAN; /* 契约：失败时非空 action 置 NaN */
        }
        fill_error(error, g_infer_ret, "execute");
        return g_infer_ret;
    }
    if (action != NULL) {
        *action = g_infer_inf_output ? INFINITY : 0.1f * (obs[0] + obs[1] + obs[2] + obs[3]);
    }
    return CARTPOLE_OK;
}

cartpole_status cartpole_npu_close(cartpole_npu **handle, cartpole_error *error)
{
    ev(EV_CLOSE);
    if (handle == NULL || *handle == NULL) {
        return CARTPOLE_OK; /* 契约：NULL 句柄幂等 */
    }
    if (g_close_fail_times > 0) {
        g_close_fail_times--;
        fill_error(error, CARTPOLE_STATE, "free_buffer");
        return CARTPOLE_STATE; /* 资源保留，句柄仍非 NULL，可再次 close */
    }
    *handle = NULL;
    return CARTPOLE_OK;
}

static void reset_state(void)
{
    g_event_count = 0;
    memset(g_events, 0, sizeof(g_events));
    g_acl_init_ret = g_set_device_ret = g_reset_device_ret = g_finalize_ret = 0;
    g_open_ret = CARTPOLE_OK;
    g_open_leave_handle = 0;
    g_infer_ret = CARTPOLE_OK;
    g_infer_inf_output = 0;
    g_close_fail_times = 0;
    g_infer_calls = 0;
}

/* ---- 测试 ---- */

static void test_init_happy_path(void)
{
    reset_state();
    assert(policy_init("model.om") == 0);
    assert(before(EV_ACL_INIT, EV_SET_DEVICE) && before(EV_SET_DEVICE, EV_OPEN));
    assert(count(EV_OPEN) == 1 && count(EV_ACL_FINALIZE) == 0);

    float action = 1.0f;
    float obs[4] = {0.1f, 0.2f, -0.3f, -0.4f};
    assert(policy_forward(obs, &action) == 0);
    assert(action == 0.1f * (obs[0] + obs[1] + obs[2] + obs[3])); /* 原始动作直传，不缩放 */

    policy_deinit();
    assert(before(EV_CLOSE, EV_RESET_DEVICE) && before(EV_RESET_DEVICE, EV_ACL_FINALIZE));
    assert(count(EV_CLOSE) == 1 && count(EV_RESET_DEVICE) == 1 && count(EV_ACL_FINALIZE) == 1);

    policy_deinit(); /* 幂等：仅剩对 NULL 句柄的幂等 close，不再释放任何资源 */
    assert(count(EV_CLOSE) == 2); /* 幂等 close 仍被调用（契约允许） */
    assert(count(EV_RESET_DEVICE) == 1 && count(EV_ACL_FINALIZE) == 1);
    assert(count(EV_OPEN) == 1 && count(EV_INFER) == 1);
}

static void test_init_acl_init_fails(void)
{
    reset_state();
    g_acl_init_ret = 1;
    assert(policy_init("model.om") != 0);
    assert(count(EV_ACL_INIT) == 1 && count(EV_SET_DEVICE) == 0 && count(EV_OPEN) == 0);

    policy_deinit(); /* 运行时未建立：不得 finalize */
    assert(count(EV_ACL_FINALIZE) == 0 && count(EV_RESET_DEVICE) == 0);
}

static void test_init_set_device_fails(void)
{
    reset_state();
    g_set_device_ret = 2;
    assert(policy_init("model.om") != 0);
    assert(count(EV_OPEN) == 0 && count(EV_RESET_DEVICE) == 0);
    assert(count(EV_ACL_FINALIZE) == 1); /* init 已成功：须 finalize 释放 */
    policy_deinit();
    assert(count(EV_ACL_FINALIZE) == 1);
}

static void test_init_open_fails_clean(void)
{
    reset_state();
    g_open_ret = CARTPOLE_IO;
    assert(policy_init("model.om") != 0);
    /* 退出梯子仍完整：close（NULL 幂等）-> reset -> finalize */
    assert(count(EV_CLOSE) == 1 && count(EV_RESET_DEVICE) == 1 && count(EV_ACL_FINALIZE) == 1);
    policy_deinit();
    assert(count(EV_ACL_FINALIZE) == 1);
}

static void test_init_open_leaves_close_only_handle(void)
{
    reset_state();
    g_open_ret = CARTPOLE_MODEL;
    g_open_leave_handle = 1;
    assert(policy_init("model.om") != 0);
    /* 残留句柄由退出路径的 close 释放，随后正常释放运行时 */
    assert(count(EV_CLOSE) == 1 && count(EV_RESET_DEVICE) == 1 && count(EV_ACL_FINALIZE) == 1);
    policy_deinit();
    assert(count(EV_RESET_DEVICE) == 1);
}

static void test_close_failure_holds_runtime(void)
{
    reset_state();
    g_open_leave_handle = 1;
    g_open_ret = CARTPOLE_MODEL;
    g_close_fail_times = 1; /* 第一次 close 失败 */
    assert(policy_init("model.om") != 0);
    assert(count(EV_CLOSE) == 1 && count(EV_RESET_DEVICE) == 0 && count(EV_ACL_FINALIZE) == 0);

    policy_deinit(); /* 重试 close：成功后补全梯子 */
    assert(count(EV_CLOSE) == 2);
    assert(count(EV_RESET_DEVICE) == 1 && count(EV_ACL_FINALIZE) == 1);
}

static void test_forward_before_init(void)
{
    reset_state();
    float action = 1.0f;
    assert(policy_forward((const float[4]){0, 0, 0, 0}, &action) != 0);
    assert(action == 0.0f);
    assert(count(EV_INFER) == 0);
}

static void test_forward_infer_failure_invalidates_handle(void)
{
    reset_state();
    assert(policy_init("model.om") == 0);
    float action = 1.0f;
    g_infer_ret = CARTPOLE_SDK;
    assert(policy_forward((const float[4]){0.5f, 0.1f, 0, 0}, &action) != 0);
    assert(action == 0.0f);

    g_infer_ret = CARTPOLE_OK; /* 即使库恢复，封装也不得再用失效句柄 */
    assert(policy_forward((const float[4]){0.5f, 0.1f, 0, 0}, &action) != 0);
    assert(g_infer_calls == 1); /* 第二次未触达库 */

    policy_deinit(); /* 失效句柄仍可正常走 close 释放 */
    assert(count(EV_CLOSE) == 1 && count(EV_ACL_FINALIZE) == 1);
}

static void test_forward_rejects_nonfinite_observation(void)
{
    reset_state();
    assert(policy_init("model.om") == 0);
    float action = 1.0f;
    const float obs[4] = {0.0f, NAN, 0.0f, 0.0f};
    assert(policy_forward(obs, &action) != 0);
    assert(action == 0.0f && g_infer_calls == 1);
    policy_deinit();
}

static void test_forward_rejects_nonfinite_action(void)
{
    reset_state();
    assert(policy_init("model.om") == 0);
    g_infer_inf_output = 1;
    float action = 1.0f;
    assert(policy_forward((const float[4]){1, 1, 1, 1}, &action) != 0);
    assert(action == 0.0f);
    g_infer_inf_output = 0;
    /* 输出非有限同样使句柄失效：不得再次推理 */
    assert(policy_forward((const float[4]){1, 1, 1, 1}, &action) != 0);
    assert(g_infer_calls == 1);
    policy_deinit();
}

static void test_double_init_rejected(void)
{
    reset_state();
    assert(policy_init("model.om") == 0);
    assert(policy_init("model.om") != 0);
    assert(count(EV_OPEN) == 1); /* 未重复 open，也未破坏现有会话 */
    policy_deinit();
    assert(count(EV_CLOSE) == 1);
}

int main(void)
{
    test_init_happy_path();
    test_init_acl_init_fails();
    test_init_set_device_fails();
    test_init_open_fails_clean();
    test_init_open_leaves_close_only_handle();
    test_close_failure_holds_runtime();
    test_forward_before_init();
    test_forward_infer_failure_invalidates_handle();
    test_forward_rejects_nonfinite_observation();
    test_forward_rejects_nonfinite_action();
    test_double_init_rejected();
    printf("policy_test: all passed\n");
    return 0;
}
