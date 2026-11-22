"""Project the observed board through a short candidate eye-in-hand motion."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.tools.handeye_calibration.chessboard import camera_intrinsics
from .geometry import matrix


class BoardView:
    def __init__(self, info, end_camera, config):
        self.k, self.d = camera_intrinsics(info)
        self.image_limit = np.array([info.width-1., info.height-1.])
        self.ec = end_camera
        self.horizon = config['view_prediction_s']
        self.safe_margin = config['view_margin_px']

    def project(self, corners, camera):
        optical = (corners-camera[:3, 3]) @ camera[:3, :3]
        if np.any(optical[:, 2] <= 1e-5):
            return None, None
        pixels, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3), self.k, self.d)
        pixels = pixels.reshape(-1, 2)
        if not np.isfinite(pixels).all():
            return None, None
        return pixels, float(np.mean(optical[:, 2]))

    def margin(self, pixels):
        return -np.inf if pixels is None else float(np.min(np.minimum(pixels, self.image_limit-pixels)))

    def predicted_margin(self, corners, tcp, twist):
        margin = np.inf
        for step in (.5*self.horizon, self.horizon):
            predicted = tcp.copy()
            predicted[:3, 3] += twist[:3]*step
            predicted[:3, :3] = Rotation.from_rotvec(twist[3:]*step).as_matrix() @ tcp[:3, :3]
            pixels, _ = self.project(corners, predicted @ self.ec)
            margin = min(margin, self.margin(pixels))
        return margin

    def shape(self, corners, actual, twist, speed_limit):
        tcp = matrix(actual)
        camera = tcp @ self.ec
        pixels, depth = self.project(corners, camera)
        margin = self.margin(pixels)
        if self.predicted_margin(corners, tcp, twist) >= self.safe_margin:
            return twist, margin, False
        if not np.any(twist) or pixels is None:
            return np.zeros(6), margin, bool(np.any(twist))

        candidate = twist.copy()
        for factor in (.75, .5, .25):
            candidate[3:] = twist[3:]*factor
            if self.predicted_margin(corners, tcp, candidate) >= self.safe_margin:
                return candidate, margin, True

        # Shift the camera toward the board within its image plane. Restrict
        # this motion to a fraction of the operator's current speed cap.
        offset = (pixels.min(axis=0)+pixels.max(axis=0)-self.image_limit)*.5
        shift = camera[:3, :2] @ (offset*depth/np.diag(self.k)[:2])/self.horizon
        shift *= min(1., .4*speed_limit/max(np.linalg.norm(shift), 1e-12))
        recenter = np.r_[shift, np.zeros(3)]
        candidate[3:] = 0.
        candidates = (candidate, recenter)
        best = max(candidates, key=lambda v: self.predicted_margin(corners, tcp, v))
        floor = min(margin, self.safe_margin)
        if self.predicted_margin(corners, tcp, best) > floor:
            return best, margin, True
        return np.zeros(6), margin, True

    def joint_scale(self, corners, actual, realized):
        """Bound the actual Jacobian twist after the joint solver trades axes."""
        tcp = matrix(actual)
        pixels, _ = self.project(corners, tcp @ self.ec)
        if pixels is None:
            return 0.
        floor = min(self.margin(pixels), self.safe_margin)
        for factor in (1., .5, .25):
            if self.predicted_margin(corners, tcp, realized*factor) >= floor:
                return factor
        return 0.
