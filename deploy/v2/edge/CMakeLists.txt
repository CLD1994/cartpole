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
# SDK 可提供交叉链接用 stub；板端使用匹配运行库。附加库随 SDK 版本配置。
option(EDGE_ENABLE_NPU "Build SVP ACL backend (requires target SDK)" OFF)
set(SVP_ACL_ROOT "" CACHE PATH "SVP ACL 目录，包含 include/acl")
set(SVP_ACL_LIBRARY_DIR "" CACHE PATH "与目标 ABI 配套的 SVP ACL 库目录")
set(SVP_ACL_EXTRA_LIBRARIES "" CACHE STRING "目标 SDK 要求的附加链接库")
if(EDGE_ENABLE_NPU)
if(NOT EXISTS "${SVP_ACL_ROOT}/include/acl/svp_acl_mdl.h")
    message(FATAL_ERROR "请设置 SVP_ACL_ROOT，使用配套的真实 SDK 头文件")
endif()
if(NOT IS_DIRECTORY "${SVP_ACL_LIBRARY_DIR}")
    message(FATAL_ERROR "请设置 SVP_ACL_LIBRARY_DIR，使用与目标 ABI 配套的 SDK 库")
endif()
find_library(SVP_ACL_LIBRARY NAMES svp_acl PATHS "${SVP_ACL_LIBRARY_DIR}" NO_DEFAULT_PATH REQUIRED)
endif()

add_executable(cartpole_edge
    main.c
    uart.c
    proto.c
    cartpole.c
    cartpole_format.c
    cartpole_runtime.c
    cartpole_schedule.c
    serial_cartpole.c
    cartpole_controller.c
    cartpole_motion.c
    console.c
    policy.c
    cpu_policy.c
    control.c
    package.c
    third_party/yyjson/yyjson.c
)

target_include_directories(cartpole_edge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/npu/include)
target_compile_definitions(cartpole_edge PRIVATE YYJSON_DISABLE_WRITER=1 YYJSON_DISABLE_UTILS=1 YYJSON_DISABLE_FILE=1 YYJSON_DISABLE_INCR_READER=1 YYJSON_READER_DEPTH_LIMIT=16)
target_link_libraries(cartpole_edge PRIVATE m)
target_compile_options(cartpole_edge PRIVATE -ffp-contract=off)
if(EDGE_ENABLE_NPU)
    target_sources(cartpole_edge PRIVATE policy_npu.c)
    target_compile_definitions(cartpole_edge PRIVATE EDGE_ENABLE_NPU=1)
    target_include_directories(cartpole_edge PRIVATE ${SVP_ACL_ROOT}/include)
    target_link_directories(cartpole_edge PRIVATE "${SVP_ACL_LIBRARY_DIR}")
    target_link_libraries(cartpole_edge PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/npu/lib/libcartpole_npu.a
        ${SVP_ACL_LIBRARY} ${SVP_ACL_EXTRA_LIBRARIES})
else()
    target_compile_definitions(cartpole_edge PRIVATE EDGE_ENABLE_NPU=0)
endif()
