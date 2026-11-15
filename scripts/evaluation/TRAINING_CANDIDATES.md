# Training-metric candidate checkpoints

Opt-in `--training-candidates`, for NEW training or explicit checkpoint resume.
It cannot attach to an already running Python process. This branch includes the
original V2, Window16, Window16-Tau0 tasks and optional low-LR resume guard.

Every iteration after logging, consider the last 100 rollout summaries. Require
at least 256 completed non-partial episodes. Rank by total successes / total
completed (not average of percentages), then Mean episode length, then Mean reward;
keep the earlier checkpoint on an exact tie. Keep the top 3 and replace only
generated candidate files. Ordinary periodic checkpoints are never removed.
Default candidate interval is 1, so no eligible iteration is skipped. Candidates
can be adjacent and windows overlap; they are not independent trials.

Success-rate scope: complete episodes ending in the last 100 rollouts, excluding
initial randomized partial episodes. Length/reward scope: official logger's last
100 ended episodes (possibly initial partial episodes), exactly the console values.
These are different windows, explicitly recorded; secondary metrics are tie-breaks
only. Episodes themselves can span several changing policies. Values do NOT measure
one frozen checkpoint. Saves occur AFTER the triggering PPO update (the training
rollout preceded that update), include actor/critic/Adam state, and record this
relationship in infos and metadata. This is NOT a claim of best deployment policy.
Final candidates need common fixed-policy evaluation.

Outputs in the new run's `training-candidates/`:
- `candidates.json`: ranked top 3, counts, metric scopes, window iteration range,
  checkpoint paths, and best_checkpoint (null before sufficient evidence).
- `candidate-history.jsonl`: save/eviction history.
- `candidate_<iteration>.pt`: resumable full checkpoints.
On normal end: `TRAINING_METRIC_BEST <path>`. Interrupted runs may retain status
running; this does not invalidate earlier candidate files. Standard output and
training-checkpoints.jsonl also record candidate saves. Extra disk I/O occurs
when rankings improve; do not use this run for throughput benchmarking.

```powershell
Set-Location C:\Users\cldus\workspace\cartpole
git fetch origin
git worktree add ..\cartpole-training-candidates origin/feat/changqing-training-candidates
Set-Location ..\cartpole-training-candidates
$env:PYTHONPATH = "$($PWD.Path)\source\cartpole;$env:PYTHONPATH"
$python = 'C:\Users\cldus\IsaacLab\.venv\Scripts\python.exe'
& $python -c "import importlib.util; print(importlib.util.find_spec('cartpole').origin)"
# Example NEW run, NOT a command to restart currently running experiments:
& $python .\scripts\rsl_rl\train.py --task Hexchip-Cartpole-Real-V2-Window16-Tau0-v0 --num_envs 512 --max_iterations 2000 --run_name scratch_window16_tau0_candidates --viz none --device cuda:0 --training-candidates 2>&1 | Tee-Object .\train-candidates.log
```

To resume instead, add `--checkpoint 'ABSOLUTE_CHECKPOINT_PATH'`; max_iterations
then means additional iterations. To replicate the low-LR Window16 experiment,
choose `Hexchip-Cartpole-Real-V2-Window16-v0`, supply the original model1050 path,
and add `--resume-learning-rate 0.0003 --adaptive-lr-max 0.001`.
Do not choose tau0 when replicating the original-tau Window16 group.
Candidate history starts fresh on each new run; it does not import prior windows.
Settings: `--candidate-window 100 --candidate-min-episodes 256 --candidate-keep 3`.
Single-process only. No Isaac training was run in the delivery environment.
