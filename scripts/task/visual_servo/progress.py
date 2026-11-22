"""Measured convergence toward a fixed visual reference pose."""
from collections import deque
import math


class FrozenProgress:
    def __init__(self, config):
        self.window = config['frozen_progress_window_s']
        self.minimum = (config['stationary_position_m'], math.radians(config['stationary_angle_deg']))
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, stamp, errors, tolerances, *, limited, position_first=False):
        # Error inside either arrival band cannot keep an otherwise stalled
        # target alive by oscillating within that band's measurement noise.
        residuals = tuple(max(0., error-tol) for error, tol in zip(errors, tolerances))
        if not limited or not any(residuals):
            self.reset()
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, residuals))
        while len(self.samples) > 1 and self.samples[1][0] <= stamp-self.window:
            self.samples.popleft()
        then, previous = self.samples[0]
        if stamp-then < self.window:
            return False
        gains = tuple(before-after for before, after in zip(previous, residuals))
        # A visible approach can advance the TCP while temporarily yielding
        # orientation to preserve the feasible joint branch.
        if position_first and gains[0] >= self.minimum[0]:
            return False
        # Compare the net reduction in units of each component's noise band.
        progress = sum(gain/threshold for gain, threshold in zip(gains, self.minimum))
        return progress < 1.
