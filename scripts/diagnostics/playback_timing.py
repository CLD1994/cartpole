"""可注入时钟的回放计时；不依赖仿真器。"""
import math
import time


def measure_playback(step, *, dt, warmup=500, steps=5000, real_time=False,
                     clock=time.perf_counter, sleep=time.sleep, synchronize=lambda: None,
                     on_measurement_start=lambda: None):
    """预热后计时。异常直接传播，不输出不完整的成功结果。"""
    if not math.isfinite(dt) or dt <= 0 or warmup < 0 or steps <= 0:
        raise ValueError('dt 和 steps 必须为正，warmup 不得为负')
    for _ in range(warmup):
        step()
    synchronize()
    on_measurement_start()
    start = clock()
    for _ in range(steps):
        before = clock()
        step()
        remaining = dt - (clock() - before)
        if real_time and remaining > 0:
            sleep(remaining)
    loop_end = clock()
    synchronize()
    end = clock()
    wall = end - start
    if wall <= 0:
        raise ValueError('计时器未产生正时长')
    return dict(steps=steps, warmup_steps=warmup, step_dt_s=dt,
                sim_s=steps * dt, wall_s=wall, rtf=steps * dt / wall,
                step_loop_wall_s=loop_end - start, completion_sync_s=end - loop_end,
                mode='real-time' if real_time else 'unpaced')


class HostPhaseTimer:
    """主机函数调用耗时，不同步GPU，不把数据解释成GPU执行时间。"""
    def __init__(self, *, clock=time.perf_counter):
        self.clock = clock
        self.samples = {}

    def call(self, phase, function, *args):
        start = self.clock()
        result = function(*args)
        elapsed = self.clock() - start
        self.samples.setdefault(phase, []).append(elapsed)
        return result

    def clear(self):
        self.samples.clear()

    def summary(self):
        result = {}
        for phase, samples in self.samples.items():
            ordered = sorted(samples)
            count = len(ordered)
            result[phase] = dict(calls=count, total_s=sum(ordered),
                mean_ms=1000 * sum(ordered) / count,
                p50_ms=1000 * ordered[math.ceil(count * .50) - 1],
                p95_ms=1000 * ordered[math.ceil(count * .95) - 1],
                max_ms=1000 * ordered[-1])
        return result
