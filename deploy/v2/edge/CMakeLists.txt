# 端侧控制程序（新装置 v2）。
#
# 交叉编译需要两类外部输入（来自 CV610 侧项目的交叉编译交接包）：
#   1. 交叉工具链：arm-v01c02-linux-musleabi-gcc（GCC 10.3 musl 1.2.3，
#      Cortex-A7 softfp），交接包含完整工具链与目标 sysroot；
#   2. SVP ACL SDK：include/acl/svp_acl.h 头文件与 musl ARM 动态库
#      （libsvp_acl.so 及其必须显式带入的附加库）。
# Windows 主机经 WSL Ubuntu 构建（cmake >= 3.18、make；无需主机 GCC）：
#   CROSS_COMPILE=/path/to/toolchain/arm-v01c02-linux-musleabi-gcc/bin/arm-v01c02-linux-musleabi-
#   SVP_ACL_ROOT=/path/to/sdk
#   export CROSS_COMPILE SVP_ACL_ROOT
#   cmake -S deploy/v2/edge -B deploy/v2/edge/build-arm -G 'Unix Makefiles' \
#     -DCMAKE_TOOLCHAIN_FILE=$PWD/deploy/v2/edge/cmake/arm-linux.cmake \
#     -DSVP_ACL_ROOT=$SVP_ACL_ROOT \
#     -DSVP_ACL_LIBRARY_DIR=$SVP_ACL_ROOT/lib \
#     -DSVP_ACL_EXTRA_LIBRARIES='ss_mpi_sysmem;protobuf-c;securec;ot_osal;ot_irq;ot_sys;ot_base' \
#     -DCMAKE_BUILD_TYPE=Release
#   cmake --build deploy/v2/edge/build-arm --parallel 2
# 产物为单二进制，与 .om 模型一起 scp 上板。

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
