# 端侧控制程序（新装置 v2）。
#
# 日常构建入口是同目录的 build.sh（在 WSL Ubuntu 内执行）：
#   bash deploy/v2/edge/build.sh        # 依赖默认位于 WSL 内部 ~/cv610
# 依赖导入与环境修复（一次性）见同目录 setup-wsl.sh：把 CV610 交叉编译交接包
# 复制进 WSL ext4（NTFS 会破坏符号链接与大小写敏感文件名，不能作工具链的家）
# 并按交接清单恢复完整性。本文件由 build.sh 以如下变量驱动，也可手动调用：
#   CROSS_COMPILE（env）            arm-v01c02-linux-musleabi- 前缀（含路径）
#   -DSVP_ACL_ROOT                  SDK 根（含 include/acl 与 lib/）
#   -DSVP_ACL_LIBRARY_DIR           musl ARM SDK 库目录
#   -DSVP_ACL_EXTRA_LIBRARIES       附加链接库（见 npu/README.md，缺则未定义符号）

cmake_minimum_required(VERSION 3.18)
project(cartpole_edge C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_SKIP_RPATH ON) # 板上库经 LD_LIBRARY_PATH/默认路径解析，产物不带主机路径

# SVP ACL 运行时（CV610 SDK）：svp_acl_init / svp_acl_rt_set_device 等运行时
# 入口不在推理库内，libcartpole_npu.a 的全部未解析符号都来自它。
# libsvp_acl.so 自身只登记 libc.so，附加库必须显式链接，否则出现未定义符号。
set(SVP_ACL_ROOT "" CACHE PATH "SVP ACL 目录，包含 include/acl")
set(SVP_ACL_LIBRARY_DIR "" CACHE PATH "与目标 ABI 配套的 SVP ACL 库目录")
set(SVP_ACL_EXTRA_LIBRARIES "" CACHE STRING "目标 SDK 要求的附加链接库")
if(NOT EXISTS "${SVP_ACL_ROOT}/include/acl/svp_acl_mdl.h")
    message(FATAL_ERROR "请设置 SVP_ACL_ROOT，使用配套的真实 SDK 头文件")
endif()
if(NOT IS_DIRECTORY "${SVP_ACL_LIBRARY_DIR}")
    message(FATAL_ERROR "请设置 SVP_ACL_LIBRARY_DIR，使用与目标 ABI 配套的 SDK 库")
endif()
find_library(SVP_ACL_LIBRARY NAMES svp_acl PATHS "${SVP_ACL_LIBRARY_DIR}" NO_DEFAULT_PATH REQUIRED)

add_executable(cartpole_edge
    main.c
    uart.c
    proto.c
    rig.c
    console.c
    policy.c
)

target_include_directories(cartpole_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/npu/include
    ${SVP_ACL_ROOT}/include
)
target_link_libraries(cartpole_edge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/npu/lib/libcartpole_npu.a
    ${SVP_ACL_LIBRARY}
    m
    ${SVP_ACL_EXTRA_LIBRARIES}
)
