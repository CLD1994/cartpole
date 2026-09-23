# cartpole_npu 推理库（外部依赖产物）

来自 CV610 SVP_NPU 示例项目整理出的 `.om` 模型执行封装，本目录收编其产物：

- `include/cartpole_npu.h` —— 唯一公共头，接口契约以其注释为准。
- `lib/libcartpole_npu.a` —— ARM 32-bit（musl gcc 10.3）静态库，只能被 CV610
  交叉工具链链接。量化与张量布局已封装在库内（期望 FP32[1,4] 输入、FP32[1,1]
  输出，库自行校验模型布局与数值有限性）。
- `src/replay.c` —— 库作者的参考回放程序（观测 CSV -> 动作 CSV），完整演示
  调用顺序与错误处理，不参与本目录构建。

## 调用方义务（已由上级目录的 policy.c 实现）

1. 进程启动，同一线程依次调用 `svp_acl_init(NULL)`、`svp_acl_rt_set_device(0)`
   并检查返回码；库只接受 DEVICE 运行模式。句柄预置 NULL 后调用
   `cartpole_npu_open` 加载模型。
2. 推理：同一线程调用 `cartpole_npu_infer`，仅当返回 `CARTPOLE_OK` 才使用动作
   （未缩放的原始值，缩放与执行器协议由调用方负责）。观测顺序为小车位置、
   杆角度、小车速度、杆角速度，全部有限。推理失败或输出非有限值后句柄失效，
   只能 `cartpole_npu_close`，不得继续推理。
3. 退出：`cartpole_npu_close`；成功（句柄变 NULL）后 `svp_acl_rt_reset_device(0)`
   -> `svp_acl_finalize()`。close 未完成时不 reset/finalize，资源由进程退出回收。

## 链接

`svp_acl_init` / `svp_acl_rt_set_device` 等运行时符号不在本库内，由 CV610 SDK
的 `svp_acl` 库提供（头文件为 `<acl/svp_acl.h>`，错误码类型 `svp_acl_error`，
0 表示成功）。构建时用 `-DSVP_ACL_ROOT` 指向该安装根目录，见上级
`CMakeLists.txt`。`policy.c` 在拿不到 SDK 头（主机单元测试）时按同一签名使用
最小声明。
