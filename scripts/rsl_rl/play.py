# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""使用 Isaac Lab 3.0 的公开入口回放本项目任务。"""

from __future__ import annotations

import sys
from collections.abc import Callable


def main(argv: list[str] | None = None, *, run_cli: Callable[[list[str]], int] | None = None) -> int:
    """保留本项目命令行，回放生命周期交由 Isaac Lab 管理。

    ``run_cli`` 是可注入的公开命令分发接口；默认先初始化 Warp 并注册任务。
    """
    if run_cli is None:
        import warp as wp

        # 必须先于任务导入：Warp 在创建 kernel 模块时读取这个设置。
        wp.config.enable_backward = False
        import cartpole.tasks  # noqa: F401
        from isaaclab_rl.entrypoints import run_play_cli

        run_cli = run_play_cli
    args = list(sys.argv[1:] if argv is None else argv)
    return run_cli(["--rl_library", "rsl_rl", *args])


if __name__ == "__main__":
    raise SystemExit(main())
