# 16 ms 窗口、无参考速度一阶滤波的续训对照

本任务 `Hexchip-Cartpole-Real-V2-Window16-Tau0-v0` 继承 Window16 任务，唯一变化是 `tau=0`。配置初值和每回合随机化均固定为零。加速度限制、指令延迟、速度范围、限力伺服、奖励和 PPO 参数均保持。速度参考每物理步向目标最多改变 `acc_max*dt`，达到目标后保持。它近似梯形限加速度，不是固件逐 STEP 波形与电机机械响应的完整复现。

以下命令在**新 PowerShell** 中从原 cartpole 仓库开始，不切换或修改正在训练的工作树。与 Window16 分支一样，从原 model_1050 恢复，而不是等待 Window16 训练完成。保留默认优化器状态，额外训练 1000 轮，512 环境；输出在新工作树的 logs 内。

```powershell
Set-Location C:\Users\cldus\workspace\cartpole
git fetch origin
git worktree add ..\cartpole-window16-tau0 origin/feat/changqing-train-window16-tau0
Set-Location ..\cartpole-window16-tau0
$env:PYTHONPATH = "$($PWD.Path)\source\cartpole;$env:PYTHONPATH"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
$checkpoint = 'C:\Users\cldus\workspace\cartpole\logs\rsl_rl\cartpole_real_v2\2026-11-15_07-34-57_resume_model500\model_1050.pt'
& $python -c "import importlib.util; print(importlib.util.find_spec('cartpole').origin)"
& $python .\scripts\rsl_rl\train.py --task Hexchip-Cartpole-Real-V2-Window16-Tau0-v0 --checkpoint $checkpoint --num_envs 512 --max_iterations 1000 --run_name resume1050_window16_tau0 --viz none --device cuda:0
```

打印的源码路径应位于 `cartpole-window16-tau0\source\cartpole`。

训练、play 和评估通过同一任务配置选择动力学。评估用本任务，不再额外叠加速度窗口：

```powershell
& $python .\scripts\evaluation\evaluate_policy.py --task Hexchip-Cartpole-Real-V2-Window16-Tau0-v0 --checkpoint $candidate --num_envs 32 --seeds 101 202 --viz none --device cuda:0 --output-dir .\logs\evaluation-window16-tau0
```

两路训练并行会竞争 GPU，墙钟耗时不用于比较学习效果；训练后还需在相同条件交叉评估，不能仅比较两种动力学各自的训练成功率。尚未在本工作环境实际运行 Isaac Lab。
