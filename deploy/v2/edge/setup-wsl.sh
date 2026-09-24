#!/usr/bin/env bash
# 把 CV610 交叉编译交接包导入 WSL 内部文件系统，并修复 Windows 拷贝造成的两类损耗。
# 在 WSL（Ubuntu）内执行，一次性操作。用法：
#   bash deploy/v2/edge/setup-wsl.sh <交接包路径> [目标目录]
#     交接包路径：包含 toolchain/ sdk/ SHA256SUMS SYMLINKS.tsv 的目录；
#                 在 Windows 侧时写 /mnt/c/... 形式。
#     目标目录：默认 ~/cv610，之后 build.sh 默认从这里取依赖。
#
# Windows 拷贝造成的损耗与本脚本的修复（可重复执行）：
#   1. 符号链接被实体化为目标文件的完整副本——按 SYMLINKS.tsv 重建相对链接；
#   2. 仅大小写不同的内核 netfilter 头成对文件（xt_CONNMARK.h 与
#      xt_connmark.h 等，共 8 对）在 NTFS 大小写不敏感下只剩一个名——从
#      kernel.org 的 linux v5.10 UAPI 取回缺失或校验不符的一方，仅当 sha256
#      与交接清单一致时落盘；这些头与本项目构建无关，恢复失败不阻塞。
set -euo pipefail

SRC="${1:?用法: $0 <交接包路径> [目标目录]}"
DST="${2:-$HOME/cv610}"
TOOLCHAIN_DIR="toolchain/arm-v01c02-linux-musleabi-gcc"

[ -f "$SRC/SHA256SUMS" ] || { echo "错误：$SRC 下没有 SHA256SUMS，不是交接包目录" >&2; exit 1; }
if [ -e "$DST" ]; then
    [ -f "$DST/SHA256SUMS" ] || { echo "错误：$DST 已存在且不是交接包目录" >&2; exit 1; }
    echo "$DST 已导入，仅执行修复步骤"
else
    echo "复制交接包 -> $DST（约 247 MiB）"
    mkdir -p "$DST"
    cp -a "$SRC"/. "$DST"/
fi
cd "$DST"

# 1) 重建符号链接
if [ -f SYMLINKS.tsv ]; then
    restored=0
    while IFS=$'\t' read -r path target; do
        path="${path%$'\r'}"
        target="${target%$'\r'}"
        case "$path" in
            ''|path) continue ;;
        esac
        if [ ! -L "$path" ]; then
            rm -f "$path"
            ln -s "$target" "$path"
            restored=$((restored + 1))
        fi
    done < SYMLINKS.tsv
    echo "符号链接重建：$restored 个"
else
    echo "告警：无 SYMLINKS.tsv，跳过符号链接重建" >&2
fi

# 2) 恢复大小写受损的内核 UAPI 头。
# NTFS 大小写不敏感把成对文件（xt_CONNMARK.h 与 xt_connmark.h 等）合并或
# 丢失其中一个名；对两种拼写都做"缺失或校验不符则取回"，取回内容必须与
# 交接清单的 sha256 一致才落盘。
KERNEL_BASE="https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/plain/include/uapi"
for h in \
    linux/netfilter/xt_CONNMARK.h \
    linux/netfilter/xt_connmark.h \
    linux/netfilter/xt_DSCP.h \
    linux/netfilter/xt_dscp.h \
    linux/netfilter/xt_MARK.h \
    linux/netfilter/xt_mark.h \
    linux/netfilter/xt_RATEEST.h \
    linux/netfilter/xt_rateest.h \
    linux/netfilter/xt_TCPMSS.h \
    linux/netfilter/xt_tcpmss.h \
    linux/netfilter_ipv4/ipt_ECN.h \
    linux/netfilter_ipv4/ipt_ecn.h \
    linux/netfilter_ipv4/ipt_TTL.h \
    linux/netfilter_ipv4/ipt_ttl.h \
    linux/netfilter_ipv6/ip6t_HL.h \
    linux/netfilter_ipv6/ip6t_hl.h
do
    dest="$TOOLCHAIN_DIR/target/usr/include/$h"
    want="$(awk -v f="$dest" '$2 == f {print $1}' SHA256SUMS)"
    [ -n "$want" ] || { echo "跳过 $h：清单无记录"; continue; }
    if [ -f "$dest" ] && echo "$want  $dest" | sha256sum -c --status; then
        continue # 本来就完好
    fi
    tmp="$(mktemp)"
    fetched=0
    for attempt in 1 2; do
        if curl -fsSL --max-time 30 "$KERNEL_BASE/$h" -o "$tmp" \
            && echo "$want  $tmp" | sha256sum -c --status; then
            fetched=1
            break
        fi
        sleep 2 # kernel.org 偶发连接失败，稍候重试一次
    done
    if [ "$fetched" = 1 ]; then
        mv "$tmp" "$dest"
        echo "已恢复 $h（sha256 与交接清单一致）"
    else
        rm -f "$tmp"
        echo "告警：$h 未能恢复（下载失败或校验不符），保留现状" >&2
    fi
done

# 3) 完整性校验（本目录为根）
if sha256sum --quiet -c SHA256SUMS; then
    echo "完整性校验通过：全部文件与交接清单一致"
else
    echo "注意：仍有校验失败项（见上）；若超出 8 个 netfilter 头的范围请人工排查" >&2
fi
echo "完成。日常构建：bash <仓库>/deploy/v2/edge/build.sh"
