# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""使用 Isaac Lab 3.0 的公开入口训练本项目任务。"""

from __future__ import annotations

import sys
from collections.abc import Callable


def main(argv: list[str] | None = None, *, run_cli: Callable[[list[str]], int] | None = None) -> int:
    """保留本项目命令行，训练生命周期交由 Isaac Lab 管理。

    ``run_cli`` 是可注入的公开命令分发接口；默认先初始化 Warp 并注册任务。
    """
    if run_cli is None:
        import warp as wp

        # 必须先于任务导入：Warp 在创建 kernel 模块时读取这个设置。
        wp.config.enable_backward = False
        import cartpole.tasks  # noqa: F401
        from isaaclab_rl.entrypoints import run_train_cli

        run_cli = run_train_cli
    args = list(sys.argv[1:] if argv is None else argv)
    # 项目任务使用公开 Gym entry_point 子类记录完整回合；不替换上游 runner/logger。
    task = next((a.split("=", 1)[1] for a in args if a.startswith("--task=")), "")
    if "--task" in args:
        index = args.index("--task")
        if index + 1 < len(args):
            task = args[index + 1]
    if task.startswith("Hexchip-Cartpole") and not any(
        a.startswith("env.training_episode_log_window_s=") for a in args
    ):
        args.append("env.training_episode_log_window_s=1.0")
    return run_cli(["--rl_library", "rsl_rl", *args])


if __name__ == "__main__":
    raise SystemExit(main())
