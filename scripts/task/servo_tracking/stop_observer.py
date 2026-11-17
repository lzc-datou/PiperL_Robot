"""Fresh-source, full-window measured-joint stop confirmation."""
from collections import deque
import numpy as np


class StopObserver:
    def __init__(self, window=.30, joint_span=.003, max_samples=512):
        self.window=float(window); self.joint_span=float(joint_span)
        if self.window <= 0 or self.joint_span <= 0:
            raise ValueError('stop observer bounds must be positive')
        self.samples=deque(maxlen=max_samples)
        self.last_source_stamp=0.

    def reset(self):
        self.samples.clear(); self.last_source_stamp=0.

    def observe(self, source_stamp, received_monotonic, joints):
        q=np.asarray(joints,dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all(): return False
        if not np.isfinite(source_stamp) or source_stamp <= self.last_source_stamp: return False
        if not np.isfinite(received_monotonic): return False
        self.samples.append((float(source_stamp),float(received_monotonic),q.copy()))
        self.last_source_stamp=float(source_stamp)
        return True

    def confirmed(self, now, feedback_timeout):
        while self.samples and now-self.samples[0][1] > self.window+feedback_timeout:
            self.samples.popleft()
        if len(self.samples)<2 or now-self.samples[-1][1] > feedback_timeout:
            return False
        span=self.samples[-1][1]-self.samples[0][1]
        if span+1e-9 < self.window: return False
        receive_stamps=np.asarray([item[1] for item in self.samples])
        if len(receive_stamps)>1 and np.max(np.diff(receive_stamps))>feedback_timeout:
            return False
        positions=np.stack([q for _,_,q in self.samples])
        return bool(np.max(np.ptp(positions,axis=0)) <= self.joint_span)
