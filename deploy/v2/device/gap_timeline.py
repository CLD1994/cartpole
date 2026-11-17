"""Bounded opt-in host timeline; no device timing or scheduling attribution."""
from collections import deque
import time

class GapTimeline:
    def __init__(self, enabled=False, capacity=4096, wall=time.perf_counter_ns, cpu=time.thread_time_ns):
        self.enabled = enabled
        self.wall, self.cpu = wall, cpu
        self.events = deque(maxlen=capacity)
        self.total = 0
        self.previous = None
        self.frozen = False

    def mark(self, stage, cycle=0, requested_sleep_s=None):
        if not self.enabled or self.frozen:
            return
        wall, cpu = self.wall(), self.cpu()
        previous = self.previous
        self.events.append((stage, cycle, wall, cpu,
                            None if previous is None else wall-previous[0],
                            None if previous is None else cpu-previous[1], requested_sleep_s))
        self.previous = wall, cpu
        self.total += 1

    def freeze(self):
        self.frozen = True

    def snapshot(self):
        if not self.enabled:
            return None
        fields = ('stage','cycle','wall_ns','thread_cpu_ns','wall_delta_ns','thread_cpu_delta_ns','requested_sleep_s')
        return {'scope':'bounded host stage boundaries before stopping; includes instrumentation overhead; wall minus thread CPU cannot distinguish OS preemption, blocking, or other threads; not device timing',
                'capacity':self.events.maxlen,'total_marks':self.total,
                'events':[dict(zip(fields,row)) for row in self.events]}
