# 端侧控制程序（新装置 v2）。
#
# 交叉编译：用 CV610 SDK 交叉工具链（arm 32-bit musl，与 npu/lib 中推理库的
# 构建工具链一致），并指向 SDK 侧 SVP ACL 安装根目录（含 include/acl/svp_acl.h
# 与 libsvp_acl）：
#   cmake -B build \
#     -DCMAKE_C_COMPILER=<sdk-cross-gcc> \
#     -DSVP_ACL_ROOT=<sdk 中 svp_acl 的安装根（含 include/ 与 lib/）>
#   cmake --build build
# 产物为单二进制，与 .om 模型一起 scp 上板。

cmake_minimum_required(VERSION 3.13)
project(cartpole_edge C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# SVP ACL 运行时（CV610 SDK 提供）：svp_acl_init / svp_acl_rt_set_device 等
# 运行时入口不在推理库内，libcartpole_npu.a 的全部未解析符号都来自它。
set(SVP_ACL_ROOT "" CACHE PATH "SDK 中 svp_acl 安装根目录（含 include/ 与 lib/）")
if(NOT SVP_ACL_ROOT)
    message(FATAL_ERROR "需要 -DSVP_ACL_ROOT 指向 CV610 SDK 的 svp_acl 目录（含 include/ 与 lib/）")
endif()

add_executable(cartpole_edge
    main.c
    uart.c
    proto.c
    policy.c
)

target_include_directories(cartpole_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/npu/include
    ${SVP_ACL_ROOT}/include
)
target_link_directories(cartpole_edge PRIVATE ${SVP_ACL_ROOT}/lib)
target_link_libraries(cartpole_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/npu/lib/libcartpole_npu.a
    svp_acl
)
