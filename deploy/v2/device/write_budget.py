class WriteBudget:
    """Empirical reserve, not a scheduling guarantee; check active gap after write.

    A late observation is reported and recovered, never silently retimestamped.
    """
    reserve_s = .001

    def cutoff(self, deadline):
        return deadline - self.reserve_s

    def insufficient(self, now, deadline):
        return now >= self.cutoff(deadline)
