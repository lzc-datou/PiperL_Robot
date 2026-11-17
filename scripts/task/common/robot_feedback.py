"""Fresh real feedback and position stability; no velocity filtering or control."""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from .schema import JOINTS

@dataclass(frozen=True)
class Sample:
    q: tuple
    stamp: float
    received: float


class FeedbackUnavailableError(RuntimeError):
    """Distinguish a temporary missing/stale sample from malformed feedback."""
    def __init__(self, details):
        self.details = details
        self.summary = '反馈时钟异常' if details['reason']=='future_timestamp' else '反馈中断 · 检查连接'
        labels={'missing':'尚未收到真实关节反馈','receive_stale':'真实关节反馈停止更新',
                'source_stale':'真实关节反馈源时间戳过期','future_timestamp':'反馈时间戳超前于本机时钟'}
        ages='' if details['source_age_seconds'] is None else (
            f'；接收间隔 {details["receive_age_seconds"]:.3f}s，源数据年龄 {details["source_age_seconds"]:.3f}s')
        super().__init__(labels[details['reason']]+ages+f'，新鲜度上限 {details["timeout_seconds"]:.3f}s。'
                         '请检查官方驱动、CAN 连接和时钟；恢复连续反馈后重新规划。')

class Feedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.sample = None
        self.error = None
        self.diagnostics = {}

    def joint_callback(self, msg):
        try:
            if len(msg.name) != len(set(msg.name)) or len(msg.position) != len(msg.name):
                raise ValueError('Invalid joint feedback names/positions')
            q = tuple(float(msg.position[msg.name.index(name)]) for name in JOINTS)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            if not all(math.isfinite(v) for v in q) or not math.isfinite(stamp) or stamp <= 0:
                raise ValueError('Nonfinite joint feedback or invalid timestamp')
            with self.lock:
                # Re-publishing an old sample must never refresh its reception age.
                if self.sample is None or stamp > self.sample.stamp:
                    self.sample = Sample(q, stamp, time.monotonic())
                elif stamp < self.sample.stamp:
                    raise ValueError('Feedback timestamp moved backwards')
                self.error = None
        except (ValueError, IndexError, TypeError) as exc:
            with self.lock:
                self.error = str(exc)

    def diagnostic_callback(self, kind, msg):
        with self.lock:
            self.diagnostics[kind] = (time.monotonic(), msg)

    def get(self, timeout, *, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        with self.lock:
            sample, error = self.sample, self.error
        if error:
            raise RuntimeError(error)
        receive_age=None if sample is None else now-sample.received
        source_age=None if sample is None else wall-sample.stamp
        reason=('missing' if sample is None else 'future_timestamp' if source_age < -0.1 else
                'receive_stale' if receive_age > timeout else 'source_stale' if source_age > timeout else None)
        if reason:
            raise FeedbackUnavailableError(dict(kind='feedback_unavailable',reason=reason,
                receive_age_seconds=receive_age,source_age_seconds=source_age,timeout_seconds=timeout,
                last_stamp=None if sample is None else sample.stamp,
                last_joints_rad=None if sample is None else list(sample.q)))
        return sample

    def health(self, timeout, diagnostic_timeout=None):
        self.get(timeout)
        # Official diagnostics are 2 Hz; their cadence differs from joint feedback.
        diagnostic_timeout=timeout if diagnostic_timeout is None else diagnostic_timeout
        with self.lock:
            data = dict(self.diagnostics)
        for kind in ('motor', 'arm'):
            if kind not in data:
                raise RuntimeError(f'{kind} diagnostics missing/stale: no message received')
            received_age=time.monotonic()-data[kind][0]
            if received_age > diagnostic_timeout:
                raise RuntimeError(f'{kind} diagnostics missing/stale: receive age={received_age:.3f}s, limit={diagnostic_timeout:.3f}s')
            msg = data[kind][1]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            source_age=time.time()-stamp
            if stamp<=0 or not -0.1 <= source_age <= diagnostic_timeout:
                raise RuntimeError(f'{kind} diagnostic source timestamp stale: source age={source_age:.3f}s, limit={diagnostic_timeout:.3f}s')
            statuses = {status.name: status for status in msg.status}
            required = JOINTS if kind == 'motor' else ['arm']
            for name in required:
                status = statuses.get(name)
                if status is None:
                    raise RuntimeError(f'{kind}/{name}: status missing; received {sorted(statuses)}')
                # ROS byte fields/constants are bytes on Jazzy, not integer zero.
                if status.level != status.OK:
                    levels={status.WARN:'WARN',status.ERROR:'ERROR',status.STALE:'STALE'}
                    level=levels.get(status.level,repr(status.level))
                    raise RuntimeError(f'{kind}/{name}: {level}: {status.message}')
                if kind == 'motor' and {v.key: v.value for v in status.values}.get('enabled') != 'true':
                    raise RuntimeError(f'{name} is disabled; use official enable service manually')

class StableWindow:
    def __init__(self, seconds, max_range):
        self.seconds, self.max_range = seconds, max_range
        self.samples = deque()
        self.last_stamp = None

    def update(self, sample, now, target=None, tolerance=None):
        if target is not None and max(abs(a-b) for a,b in zip(sample.q, target)) > tolerance:
            self.samples.clear()
        if sample.stamp == self.last_stamp:
            return False
        self.last_stamp = sample.stamp
        if target is not None and max(abs(a-b) for a,b in zip(sample.q, target)) > tolerance:
            return False
        self.samples.append((now, sample.q))
        while len(self.samples) > 2 and self.samples[1][0] <= now-self.seconds:
            self.samples.popleft()
        if len(self.samples) < 3 or now-self.samples[0][0] < self.seconds:
            return False
        return all(max(row[1][i] for row in self.samples)-min(row[1][i] for row in self.samples)
                   <= self.max_range for i in range(6))
