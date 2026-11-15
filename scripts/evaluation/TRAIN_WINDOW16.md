# 从 model_1050 续训速度窗口适配模型

目标：保留模型结构、奖励、动作和物理参数，只把两路速度观测改为最近 16 ms 的位置端点差除以时间。不是把速度整体向后延迟 16 ms。理想窗口在低频约有 8 ms 群延迟，真实固件的量化、采样抖动和通信延迟仍未复现。

专用任务 `Hexchip-Cartpole-Real-V2-Window16-v0` 在训练、play 和评估中均自带窗口；原任务不变。每控制步保留一次位置，2 ms 步长共 9 点。回合重置清空该槽位历史，前 16 ms 使用瞬时速度。原有高斯噪声在估速之后添加。奖励和评估指标仍使用真实物理状态，不使用窗口速度替代。

## 续训

在现有 Isaac Lab 开发环境、cartpole 目录运行。创建独立工作树，不切换正在运行诊断的工作树；PYTHONPATH 只在此 PowerShell 会话指向新工作树源码，不改共享虚拟环境的 editable 安装。保持原有环境数量；下面沿用 512。输出到新目录，不覆盖原模型。`--checkpoint` 会载入 actor、critic 和默认优化器状态；`--max_iterations 1000` 是额外执行 1000 次学习迭代，不是把总轮次设为 1000。学习率与其他 PPO 配置沿用当前配置/恢复状态，不同时调参。

```powershell
git fetch origin
git worktree add ..\cartpole-window16 origin/feat/changqing-train-velocity-window
Set-Location ..\cartpole-window16
$env:PYTHONPATH = "$($PWD.Path)\source\cartpole;$env:PYTHONPATH"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
$checkpoint = 'C:\Users\cldus\workspace\cartpole\logs\rsl_rl\cartpole_real_v2\2026-11-15_07-34-57_resume_model500\model_1050.pt'
& $python -c "import importlib.util; print(importlib.util.find_spec('cartpole').origin)"
& $python .\scripts\rsl_rl\train.py --task Hexchip-Cartpole-Real-V2-Window16-v0 --checkpoint $checkpoint --num_envs 512 --max_iterations 1000 --run_name resume1050_window16 --viz none --device cuda:0
```

续训不等于一定优于原模型，也不证明实机振荡来自速度窗口。保留原模型，训练结束后用相同任务评估候选，不直接上真机。

## 评估

将 `$candidate` 指向本次续训的候选 `.pt`。Window16 任务本身已带窗口，不要再传 `--velocity-observation-window-ms 16`，避免叠加。

```powershell
& $python .\scripts\evaluation\evaluate_policy.py --task Hexchip-Cartpole-Real-V2-Window16-v0 --checkpoint $candidate --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --output-dir .\logs\evaluation-window16
```

此实现已做语法与静态接口检查，尚未在本工作环境运行 Isaac Lab。
