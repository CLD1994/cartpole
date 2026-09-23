# 端侧控制程序（新装置 v2）。
#
# 交叉编译：用 CV610 SDK 工具链（另一个项目的推理示例已建立该工具链），
#   cmake -B build -DCMAKE_C_COMPILER=<sdk-toolchain-gcc> -DCMAKE_FIND_ROOT_PATH=...
#   cmake --build build
# 产物为单二进制，与 .om 模型一起 scp 上板。
#
# 真实策略：把 policy.c 换成 NPU 封装库实现（policy.h 接口不变），并在
# target_link_libraries 挂上该库。

cmake_minimum_required(VERSION 3.13)
project(cartpole_edge C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(cartpole_edge
    main.c
    uart.c
    proto.c
    policy.c
)

# TODO: 链接 NPU 推理库（SVP_NPU 封装）时取消注释并替换路径：
# target_sources(cartpole_edge PRIVATE policy_npu.c)
# target_link_libraries(cartpole_edge PRIVATE /path/to/libnpu_policy.a)
