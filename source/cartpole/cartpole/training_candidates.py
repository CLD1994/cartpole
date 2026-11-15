"""Training-metric checkpoint shortlist, NOT fixed-policy evaluation."""
from collections import deque
import json
import math
import statistics
from fractions import Fraction
from pathlib import Path
from types import MethodType


def install_training_candidates(runner, *, window=100, min_episodes=256, interval=1, keep=3):
    if min(window, min_episodes, interval, keep) <= 0:
        raise ValueError('candidate settings must be positive integers')
    if runner.is_distributed:
        raise ValueError('training candidate selection currently requires single-process training')
    logger = runner.logger
    if not hasattr(logger, '_last_row'):
        raise TypeError('training candidates require the complete-episode logger')
    history = deque(maxlen=window)
    top = []
    last_saved = None
    original_log = logger.log
    original_stop = logger.stop_logging_writer
    directory = Path(logger.log_dir) / 'training-candidates'
    scope = ('Rolling training episodes may span multiple policy updates; observations precede '
             'the triggering update, saved policy/optimizer are AFTER that update. '
             'Ranking is a training-metric shortlist, not fixed-policy evaluation or deployment acceptance.')

    def write_manifest(status):
        directory.mkdir(parents=True, exist_ok=True)
        payload = dict(schema_version=1, status=status, scope=scope,
                       window_rollouts=window, minimum_completed_episodes=min_episodes,
                       minimum_save_interval=interval, keep=keep,
                       ranking='successes/completed, Mean episode length, Mean reward descending; earlier iteration on exact tie',
                       best_checkpoint=top[0]['checkpoint'] if top else None,
                       candidates=top)
        tmp = directory / 'candidates.json.tmp'
        tmp.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
        tmp.replace(directory / 'candidates.json')

    def rank(row):
        return (Fraction(row['successes'], row['completed']),
                row['mean_episode_length'] if row['mean_episode_length'] is not None else -math.inf,
                row['mean_reward'] if row['mean_reward'] is not None else -math.inf,
                -row['saved_after_update_iteration'])

    def log(self, *args, **kwargs):
        nonlocal last_saved
        original_log(*args, **kwargs)
        if self.writer is None:
            return
        row = dict(self._last_row)
        history.append(row)
        if len(history) < window:
            return
        it = row['iteration']
        completed = sum(r['completed'] for r in history)
        if completed < min_episodes or (last_saved is not None and it - last_saved < interval):
            return
        successes = sum(r['successes'] for r in history)
        path = directory / f'candidate_{it}.pt'
        candidate = dict(checkpoint=str(path.resolve()), saved_after_update_iteration=it,
                         rollout_start_iteration=history[0]['iteration'], rollout_end_iteration=it,
                         completed=completed, successes=successes, failures=completed-successes,
                         success_rate=successes/completed,
                         excluded_partial_episodes=sum(r['excluded_partial_episodes'] for r in history),
                         mean_episode_length=statistics.mean(self.lenbuffer) if self.lenbuffer else None,
                         mean_reward=statistics.mean(self.rewbuffer) if self.rewbuffer else None,
                         console_metric_samples=len(self.lenbuffer),
                         console_metric_scope='Official rolling last 100 ended episodes, may include initial partial episodes; NOT the 100-rollout success window',
                         optimizer_state_included=True, scope=scope)
        for key in ('mean_episode_length', 'mean_reward'):
            if candidate[key] is not None and not math.isfinite(candidate[key]):
                candidate[key] = None
        if len(top) == keep and rank(candidate) <= rank(top[-1]):
            return
        directory.mkdir(parents=True, exist_ok=True)
        # save() includes Adam and invokes existing checkpoint metadata logging.
        runner.save(str(path), infos={'training_candidate': candidate})
        last_saved = it
        top.append(candidate)
        top.sort(key=rank, reverse=True)
        evicted = top.pop() if len(top) > keep else None
        write_manifest('running')
        # Only remove generated shortlist files; ordinary periodic checkpoints stay intact.
        if evicted is not None:
            Path(evicted['checkpoint']).unlink()
        with (directory / 'candidate-history.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(dict(saved=candidate, evicted=evicted), allow_nan=False)+'\n')
        print(f"TRAINING_CANDIDATE iteration={it} success={successes}/{completed} "
              f"rate={successes/completed:.4f} path={path}", flush=True)

    def stop(self):
        write_manifest('complete')
        print('TRAINING_METRIC_BEST ' + (top[0]['checkpoint'] if top else 'NONE (insufficient eligible windows)'), flush=True)
        original_stop()

    logger.log = MethodType(log, logger)
    logger.stop_logging_writer = MethodType(stop, logger)
    write_manifest('running')
    print(f'TRAINING_CANDIDATES window={window} min_episodes={min_episodes} interval={interval} keep={keep}', flush=True)
