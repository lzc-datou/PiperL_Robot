"""Match a camera exposure to a bracket of measured TCP feedback."""
from collections import deque
import math

import numpy as np

from scripts.tools.handeye_calibration.quality import interpolate_transform
from .geometry import CameraTarget, chessboard_target, matrix


def position_feedforward(estimate, error, band):
    # Far-goal pursuit already uses the available acceleration. Fade out the
    # extra speed there so a sudden reversal has less momentum to shed.
    return estimate*max(0., 1.-np.linalg.norm(error)/band)


class CenterVelocity:
    """Estimate base-frame board translation from consecutive exposures."""
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.stamp, self.center = 0., None
        self.velocity = np.zeros(3)

    def update(self, stamp, center):
        dt = stamp-self.stamp
        if self.center is not None and dt <= 0.:
            return np.zeros(3)
        maximum = self.config['feedforward_max_mps']
        if self.center is None or dt > self.config['max_age_s']:
            self.velocity[:] = 0.
        else:
            raw_velocity = (center-self.center)/dt
            speed = np.linalg.norm(raw_velocity)
            if speed > maximum*2.5 or speed < self.config['feedforward_deadband_mps']:
                self.velocity[:] = 0.
            else:
                if raw_velocity @ self.velocity < 0.:
                    self.velocity[:] = 0.
                weight = -math.expm1(-dt/self.config['feedforward_filter_s'])
                self.velocity += weight*(raw_velocity*min(1., maximum/max(speed, 1e-12))-self.velocity)
        self.stamp, self.center = stamp, center.copy()
        # The norm threshold avoids commanding motion from fixed-board PnP noise.
        if np.linalg.norm(self.velocity) < self.config['feedforward_deadband_mps']:
            return np.zeros(3)
        return self.velocity*self.config['feedforward_gain']


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_pose(pose):
    p, q = pose.position, pose.orientation
    return matrix([p.x, p.y, p.z, q.x, q.y, q.z, q.w])


class Observations:
    def __init__(self, frame, cfg):
        self.frame, self.cfg = frame, cfg
        self.feedback = deque(maxlen=256)
        self.last_input = 0.
        self.last_delayed = None
        self.pending = None

    def receive(self, msg, now, received):
        cg = read_pose(msg.pose)
        if cg[2, 3] <= 0:
            raise ValueError('棋盘位姿深度无效')
        self.receive_target(msg.header.frame_id, stamp_seconds(msg), chessboard_target(cg, self.cfg), now, received)

    def receive_target(self, frame, stamp, target: CameraTarget, now, received):
        if frame != self.frame:
            raise ValueError(f'观测坐标系不匹配：收到 {frame}，期望 {self.frame}')
        age_ms = (now-stamp)*1000
        if not 0 < stamp <= now:
            raise ValueError(f'观测曝光时间无效，图像年龄 {age_ms:.0f} ms')
        if stamp <= self.last_input:
            raise ValueError('重复或乱序图像')
        self.last_input = stamp
        max_age = self.cfg['max_age_s']
        if now-stamp > max_age:
            self.last_delayed = (stamp, received)
            raise ValueError(f'观测已过期 {age_ms:.0f} ms，允许 {max_age*1000:.0f} ms')
        self.pending = (stamp, received, target)

    def delayed_stream_alive(self, after, now, received_now):
        if self.last_delayed is None:
            return False
        stamp, received = self.last_delayed
        limit = self.cfg['tracking_loss_timeout_s']
        return (stamp > after and 0 <= now-stamp <= limit and
                0 <= received_now-received <= limit)

    def match(self, now, monotonic):
        if self.pending is None:
            return None
        stamp, received, target = self.pending
        if not (0 <= now-stamp <= self.cfg['max_age_s'] and
                0 <= monotonic-received <= self.cfg['max_age_s']):
            self.pending = None
            raise ValueError('观测已过期')
        if not self.feedback or self.feedback[-1][0] < stamp:
            return None
        self.pending = None
        # Only the two adjacent samples need SO(3) validation and interpolation.
        history = self.feedback
        i = len(history)-1
        while i > 0 and history[i-1][0] >= stamp:
            i -= 1
        bracket = [history[i]] if i == 0 else [history[i-1], history[i]]
        be, info = interpolate_transform(bracket, stamp, self.cfg['max_feedback_gap_s'])
        return stamp, received, be, target, info['bracket_gap_s']
