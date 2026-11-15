"""Optional resume LR override and per-optimizer-step adaptive ceiling."""
import json
import math
from pathlib import Path


def install_lr_guard(runner, *, initial_lr, max_lr, log_dir):
    if not (math.isfinite(initial_lr) and math.isfinite(max_lr) and 0 < initial_lr <= max_lr):
        raise ValueError('Require finite 0 < resume_learning_rate <= adaptive_lr_max')
    alg = runner.alg
    optimizer = alg.optimizer
    if alg.schedule != 'adaptive':
        raise ValueError('LR guard experiment requires the existing adaptive schedule')
    restored = [float(group['lr']) for group in optimizer.param_groups]
    alg.learning_rate = initial_lr
    for group in optimizer.param_groups:
        group['lr'] = initial_lr

    def before_step(opt, args, kwargs):
        # PPO may raise LR inside EVERY minibatch, not only once per iteration.
        # Clamp immediately before the actual Adam step, and feed capped value
        # back to PPO so its next KL adjustment starts from the applied rate.
        lr = float(alg.learning_rate)
        if not math.isfinite(lr) or lr <= 0:
            raise RuntimeError(f'Invalid adaptive learning rate: {lr}')
        alg.learning_rate = min(lr, max_lr)
        for group in opt.param_groups:
            group['lr'] = alg.learning_rate

    # The original optimizer and its restored moments/step counters are retained.
    runner._cartpole_lr_guard_handle = optimizer.register_step_pre_hook(before_step)
    report = dict(restored_optimizer_lrs=restored, effective_initial_lr=alg.learning_rate,
                  optimizer_lrs=[float(g['lr']) for g in optimizer.param_groups],
                  adaptive_lr_max=max_lr, enforcement='optimizer step pre-hook; every minibatch',
                  optimizer_state_entries=len(optimizer.state))
    Path(log_dir, 'learning-rate-guard.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('LEARNING_RATE_GUARD ' + json.dumps(report), flush=True)
