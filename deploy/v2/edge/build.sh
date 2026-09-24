#!/usr/bin/env bash
# 端侧控制程序（cartpole_edge）的交叉编译入口，在 WSL（Ubuntu）内执行。
# 依赖（交叉工具链 + SVP ACL SDK）默认位于 WSL 内部 ~/cv610，
# 由 setup-wsl.sh 一次性导入；也可用 CROSS_ROOT 指向其他交接包位置。
# 用法：
#   bash deploy/v2/edge/build.sh
#   CROSS_ROOT=/path/to/bundle bash deploy/v2/edge/build.sh
#   BUILD_DIR=... CMAKE_BUILD_TYPE=... 可覆盖（默认 build-arm / Release）。
# 产物：ELF32 ARM soft-float musl 单二进制，与 policy.om 一起 scp 上板；
# 板端动态库解析沿用已验证的 SDK 启动环境。
set -euo pipefail

CROSS_ROOT="${CROSS_ROOT:-$HOME/cv610}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build-arm}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
PREFIX="$CROSS_ROOT/toolchain/arm-v01c02-linux-musleabi-gcc/bin/arm-v01c02-linux-musleabi-"

[ -x "${PREFIX}gcc" ] || {
    echo "错误：交叉编译器不存在：${PREFIX}gcc" >&2
    echo "先运行 setup-wsl.sh 导入依赖，或用 CROSS_ROOT 指定交接包位置" >&2
    exit 1
}
[ -f "$CROSS_ROOT/sdk/include/acl/svp_acl_mdl.h" ] || {
    echo "错误：$CROSS_ROOT/sdk 下没有 SVP ACL 头文件" >&2
    exit 1
}

export CROSS_COMPILE="$PREFIX"

# 每次全新构建：程序只有 4 个翻译单元，全量重编代价可忽略，
# 换来零陈旧缓存问题（依赖位置、工具链文件变更后必然正确）。
rm -rf "$BUILD_DIR"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -G 'Unix Makefiles' \
    -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/cmake/arm-linux.cmake" \
    -DSVP_ACL_ROOT="$CROSS_ROOT/sdk" \
    -DSVP_ACL_LIBRARY_DIR="$CROSS_ROOT/sdk/lib" \
    -DSVP_ACL_EXTRA_LIBRARIES='ss_mpi_sysmem;protobuf-c;securec;ot_osal;ot_irq;ot_sys;ot_base' \
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

BIN="$BUILD_DIR/cartpole_edge"
RE="${PREFIX}readelf"
echo "---- 产物自检：$BIN"
fail() { echo "自检失败：$1" >&2; exit 1; }
"$RE" -h "$BIN" | grep -q 'Class:.*ELF32' || fail '非 ELF32'
"$RE" -h "$BIN" | grep -q 'Machine:.*ARM' || fail '非 ARM'
"$RE" -h "$BIN" | grep -q 'soft-float ABI' || fail '非 soft-float ABI'
"$RE" -l "$BIN" | grep -q '/lib/ld-musl-arm.so.1' || fail '非 musl 解释器'
if "$RE" -d "$BIN" | grep -qE 'RPATH|RUNPATH'; then
    fail '产物携带 RPATH/RUNPATH'
fi
for lib in svp_acl ss_mpi_sysmem protobuf-c securec ot_osal ot_irq ot_sys ot_base; do
    "$RE" -d "$BIN" | grep -q "\[lib${lib}\.so" || fail "缺少 NEEDED lib${lib}.so"
done
echo "自检通过：ELF32 ARM soft-float musl，8 个 SDK 动态依赖齐全，无 RPATH"
echo "产物：$BIN（$(du -h "$BIN" | cut -f1)）"
