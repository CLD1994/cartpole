"""可注入时钟的回放计时；不依赖仿真器。"""
import math
import time


def measure_playback(step, *, dt, warmup=500, steps=5000, real_time=False,
                     clock=time.perf_counter, sleep=time.sleep, synchronize=lambda: None):
    """预热后计时。异常直接传播，不输出不完整的成功结果。"""
    if not math.isfinite(dt) or dt <= 0 or warmup < 0 or steps <= 0:
        raise ValueError('dt 和 steps 必须为正，warmup 不得为负')
    for _ in range(warmup):
        step()
    synchronize()
    start = clock()
    for _ in range(steps):
        before = clock()
        step()
        remaining = dt - (clock() - before)
        if real_time and remaining > 0:
            sleep(remaining)
    synchronize()
    wall = clock() - start
    if wall <= 0:
        raise ValueError('计时器未产生正时长')
    return dict(steps=steps, warmup_steps=warmup, step_dt_s=dt,
                sim_s=steps * dt, wall_s=wall, rtf=steps * dt / wall,
                mode='real-time' if real_time else 'unpaced')
