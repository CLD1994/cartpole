class WriteBudget:
    """经验写入预留，不是调度保证；原始写后截止检查仍必须执行。"""
    reserve_s = .0005

    def cutoff(self, deadline):
        return deadline - self.reserve_s

    def insufficient(self, now, deadline):
        return now >= self.cutoff(deadline)
