# Window16 low-LR continuation (original tau)

Only change the PPO learning-rate treatment versus Window16: after checkpoint
load (including Adam moments), set actual learning rate to 0.0003; clamp adaptive
LR to 0.001 immediately before every optimizer minibatch step. Original adaptive
KL rule/lower bound remain unchanged. No tau, rewards, observation or dynamics changes.

Use an independent worktree/session; do not alter a currently running checkout.

```powershell
Set-Location C:\Users\cldus\workspace\cartpole
git fetch origin
git worktree add ..\cartpole-window16-low-lr origin/feat/changqing-window16-low-lr
Set-Location ..\cartpole-window16-low-lr
$env:PYTHONPATH = "$($PWD.Path)\source\cartpole;$env:PYTHONPATH"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
$checkpoint = 'C:\Users\cldus\workspace\cartpole\logs\rsl_rl\cartpole_real_v2\2026-11-15_07-34-57_resume_model500\model_1050.pt'
& $python -c "import importlib.util; print(importlib.util.find_spec('cartpole').origin)"
& $python .\scripts\rsl_rl\train.py --task Hexchip-Cartpole-Real-V2-Window16-v0 --checkpoint $checkpoint --num_envs 512 --max_iterations 1000 --run_name resume1050_window16_low_lr --viz none --device cuda:0 --resume-learning-rate 0.0003 --adaptive-lr-max 0.001 2>&1 | Tee-Object -FilePath .\train-window16-low-lr.log
"Exit code: $LASTEXITCODE"
```

Confirm printed module path is inside this worktree and `LEARNING_RATE_GUARD`
contains `effective_initial_lr: 0.0003`, `optimizer_lrs: [0.0003]`,
`adaptive_lr_max: 0.001`. The training output directory also contains
`learning-rate-guard.json`; the base dumped agent configuration may still say
0.001 because it predates this deliberate post-load override. Future resumes
must repeat the two flags to reinstall the cap (the Python hook is not serialized).
1000 means 1000 additional iterations, starting from checkpoint iteration 1050.
Evaluation/play still use the same Window16 task; there is no LR option at inference.
No Isaac Lab training was run in the delivery environment.
