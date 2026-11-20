"""Eye-in-hand target geometry. Distances in metres, rotations in base_link."""
from dataclasses import dataclass, replace
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from scripts.tools.handeye_calibration.quality import checked_transform

DEFAULT = Path(__file__).with_name('config') / 'config.yaml'
POSE_TOPIC = '/handeye/chessboard/pose'
SIM_TOPIC = '/piper_servo/sim/chessboard/pose'
SIM_FRAME = 'sim_camera_color_optical_frame'


def matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:]) - 1) > .01:
        raise ValueError('位姿或四元数无效')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(p[3:]).as_matrix()
    transform[:3, 3] = p[:3]
    return transform


def pose7(transform):
    return np.r_[transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_quat()]


def settings(path=DEFAULT):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    cfg.setdefault('tracking_loss_timeout_s', 1.5)  # Existing task configurations.
    cfg.setdefault('brake_window_s', min(.10, cfg['tracking_loss_timeout_s']-cfg['max_age_s']))
    for key, value in dict(default_speed_scale=.5, stationary_position_m=.001, stationary_angle_deg=.35,
                           center_filter_s=.05, linear_acceleration_mps2=.20, angular_acceleration_deg_s2=40.).items():
        cfg.setdefault(key, value)
    for key in ('distance_m', 'normal_tolerance_deg', 'linear_speed_mps', 'angular_speed_deg_s',
                'max_age_s', 'max_feedback_gap_s', 'correction_target_deg',
                'stationary_position_m', 'stationary_angle_deg', 'center_filter_s',
                'linear_acceleration_mps2', 'angular_acceleration_deg_s2'):
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError(f'{key} 必须是正有限数')
    if not 0 < cfg['brake_window_s'] <= min(.15, cfg['tracking_loss_timeout_s']-cfg['max_age_s']):
        raise ValueError('短时迟帧减速窗口应在失联超时以内且最多 0.15 秒')
    if not cfg['correction_target_deg'] < cfg['normal_tolerance_deg'] < 90:
        raise ValueError('纠偏目标角应小于容差且容差小于 90°')
    if not .1 <= cfg['default_speed_scale'] <= 1.:
        raise ValueError('默认速度比例须在面板范围 10% 至 100% 内')
    if not (cfg['max_feedback_gap_s'] <= cfg['max_age_s'] <= 1):
        raise ValueError('反馈间隔与观测时效配置无效')
    if not (np.isfinite(cfg['tracking_loss_timeout_s']) and
            cfg['tracking_loss_timeout_s'] > cfg['max_age_s']):
        raise ValueError('跟踪失联超时应为大于单帧运动有效期的有限数')
    if type(cfg['retreat_frames']) is not int or cfg['retreat_frames'] < 1:
        raise ValueError('退回确认帧数必须为正整数')
    limits = np.array([cfg['workspace_min_m'], cfg['workspace_max_m']], dtype=float)
    if limits.shape != (2, 3) or not np.isfinite(limits).all() or np.any(limits[0] >= limits[1]):
        raise ValueError('工作空间上下界无效')
    for key in ('board_config', 'servo_config'):
        cfg[key] = str((path.parent / cfg[key]).resolve())
    board = yaml.safe_load(Path(cfg['board_config']).read_text())
    from scripts.tools.handeye_calibration.chessboard import board_settings
    if board['target'] != 'chessboard':
        raise ValueError('检测器须配置为棋盘格')
    pattern, size = board_settings(board['chessboard'])
    cfg.update(pattern=pattern, size=size, frame=board['camera_name']+'_color_optical_frame',
               board=board['chessboard'])
    return cfg


def calibration(cfg, directory, sim=False):
    if sim:
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('y', 18, degrees=True).as_matrix()
        t[:3, 3] = [-.06, -.02, .035]
        return t, SIM_FRAME, {'source': 'sim fixture', 'matrix': t.tolist()}
    if not directory:
        raise ValueError('请选择当前 TCP 与相机支架对应的 --handeye-dir')
    path = Path(directory).resolve()
    metadata = json.loads((path / 'session.json').read_bytes())
    expected = dict(mode='eye_in_hand', end_frame='tcp_link', base_frame='base_link',
                    camera_frame=cfg['frame'], transform='tcp_link_T_'+cfg['frame'])
    if any(metadata.get(k) != v for k, v in expected.items()):
        raise ValueError('手眼标定模式、坐标系或变换方向不匹配')
    raw = (path / 'T_end_camera.json').read_bytes()
    return checked_transform(json.loads(raw)), cfg['frame'], dict(
        directory=str(path), sha256=hashlib.sha256(raw).hexdigest(), frame=cfg['frame'])


@dataclass(frozen=True)
class CameraTarget:
    """Optical-frame centre in metres and optional desired TCP +Z direction."""
    center: np.ndarray
    direction: np.ndarray | None = None
    two_sided: bool = False

    def __post_init__(self):
        center = np.array(self.center, dtype=float, copy=True)
        if center.shape != (3,) or not np.isfinite(center).all() or center[2] <= 0:
            raise ValueError('目标中心或深度无效')
        object.__setattr__(self, 'center', center)
        if self.direction is not None:
            direction = np.asarray(self.direction, dtype=float)
            length = np.linalg.norm(direction)
            if direction.shape != (3,) or not np.isfinite(length) or length < 1e-8:
                raise ValueError('目标方向无效')
            object.__setattr__(self, 'direction', direction / length)


def chessboard_target(cg, cfg):
    nx, ny = cfg['pattern']
    offset = np.array([nx-1, ny-1, 0.]) * (cfg['size']/2)
    return CameraTarget(cg[:3, 3] + cg[:3, :3] @ offset, cg[:3, 2], two_sided=True)


@dataclass
class Goal:
    pose: np.ndarray
    center: np.ndarray
    camera_center: np.ndarray
    normal: np.ndarray | None
    alpha: float | None
    correcting: bool
    switched: bool
    viewing_cos: float | None


def smooth_center(raw, previous_center, dt, tau):
    # Filter only measured translation in base_link; keep orientation, standoff
    # and exposure-specific camera measurements consistent with this frame.
    center = raw.center + np.exp(-dt/tau)*(previous_center-raw.center)
    pose = raw.pose.copy()
    pose[:3] += center-raw.center
    return replace(raw, center=center, pose=pose)


def stabilize_goal(raw, previous, cfg):
    """Coalesce noise against the last accepted base-frame pose, not the last raw sample."""
    if previous is None or raw.correcting != previous.correcting or raw.switched:
        return raw
    if (raw.normal is None) != (previous.normal is None):
        return raw
    distance = cfg['stationary_position_m']
    angle = np.deg2rad(cfg['stationary_angle_deg'])
    if (np.linalg.norm(raw.center-previous.center) > distance or
            np.linalg.norm(raw.pose[:3]-previous.pose[:3]) > distance or
            abs(raw.pose[3:] @ previous.pose[3:]) < np.cos(angle/2) or
            (raw.normal is not None and raw.normal @ previous.normal < np.cos(angle))):
        return raw
    # Preserve this exposure's measured tilt, depth and branch decision.
    return replace(raw, center=previous.center, normal=previous.normal, pose=previous.pose)


class TargetGeometry:
    def __init__(self, cfg, end_camera):
        self.cfg = cfg
        self.ec = checked_transform(end_camera)
        self.reset()

    def reset(self):
        self.sign = self.held_rotation = self.correcting = None

    def measure(self, be, target: CameraTarget):
        bc = be @ self.ec
        center_c = target.center
        center = bc[:3, 3] + bc[:3, :3] @ center_c
        normal = angle = viewing_cos = sign = None
        if target.direction is not None:
            viewing_cos = float(np.dot(target.direction, center_c) / np.linalg.norm(center_c))
            sign = 1
            if target.two_sided:
                sign = self.sign if self.sign is not None else (1 if viewing_cos >= 0 else -1)
                viewing_cos *= sign
                if viewing_cos < .1:
                    raise ValueError('板面侧别不明或检测法线翻转')
            normal = sign * (bc[:3, :3] @ target.direction)
            axis = np.cross(be[:3, 2], normal)
            angle = float(np.rad2deg(np.arctan2(np.linalg.norm(axis), np.dot(be[:3, 2], normal))))
        correcting = angle is not None and angle > self.cfg['normal_tolerance_deg'] + 1e-10
        switched = self.correcting is not None and self.correcting != correcting
        if correcting:
            if np.linalg.norm(axis) < 1e-8:
                raise ValueError(f'工具轴与法线夹角 {angle:.2f}°，纠偏方向不确定')
            delta = np.deg2rad(angle-self.cfg['correction_target_deg'])
            rotation = Rotation.from_rotvec(axis * delta / np.linalg.norm(axis)).as_matrix() @ be[:3, :3]
            self.held_rotation = None
        else:
            if self.held_rotation is None:
                self.held_rotation = be[:3, :3].copy()
            rotation = self.held_rotation
        self.sign, self.correcting = sign, correcting
        pose = np.r_[center - rotation[:, 2] * self.cfg['distance_m'], Rotation.from_matrix(rotation).as_quat()]
        return Goal(pose, center, center_c, normal, angle, correcting, switched, viewing_cos)


def workspace_reason(position, cfg, radius):
    for name, p, low, high in zip('XYZ', position, cfg['workspace_min_m'], cfg['workspace_max_m']):
        if p < low or p > high:
            return f'工作空间 {name}{"下" if p < low else "上"}限'
    if np.linalg.norm(position) > radius:
        return '超出机械臂几何可达半径'
    return ''


def joint_boundary(q, lower, upper, margin, velocity=None, horizon=0.):
    """Report every limiting face. At a soft edge a first step must point inward."""
    blocked = []
    for i, (position, low, high) in enumerate(zip(q, lower+margin, upper-margin)):
        speed = velocity[i] if velocity is not None else 0.
        if velocity is not None and speed == 0.:
            continue
        if position + min(0., speed)*horizon <= low and speed <= 0:
            blocked.append(f'J{i+1} 下限')
        if position + max(0., speed)*horizon >= high and speed >= 0:
            blocked.append(f'J{i+1} 上限')
    return '、'.join(blocked)


class WristReach:
    """Triangle-inequality bounds from shoulder pivot to the last wrist pivot."""
    def __init__(self, model):
        first = next(i for i, (_, _, index) in enumerate(model.chain) if index is not None)
        last = max(i for i, (_, _, index) in enumerate(model.chain) if index is not None)
        base_shoulder = np.eye(4)
        for t, _, _ in model.chain[:first+1]:
            base_shoulder = base_shoulder @ t
        lengths = [np.linalg.norm(t[:3, 3]) for t, _, _ in model.chain[first+1:last]]
        self.shoulder = base_shoulder[:3, 3]
        flange_tcp = np.eye(4)
        for t, _, _ in model.chain[last+1:]:
            flange_tcp = flange_tcp @ t
        self.tcp_flange = np.linalg.inv(flange_tcp)
        origin, axis, _ = model.chain[last]
        offset = origin[:3, :3].T @ origin[:3, 3]
        self.axial_offset = axis * np.dot(axis, offset)
        # The axial component is invariant under the last joint's rotation.
        # Allow the full unknown perpendicular offset in both bounds.
        slack = np.linalg.norm(offset-self.axial_offset) + 1e-6
        self.outer = sum(lengths) + slack
        self.inner = max(0., 2*max(lengths)-sum(lengths)-slack)

    def reason(self, pose):
        flange = matrix(pose) @ self.tcp_flange
        wrist = flange[:3, 3] - flange[:3, :3] @ self.axial_offset
        length = np.linalg.norm(wrist-self.shoulder)
        if length > self.outer:
            return '腕心超出几何可达半径，请向肩部移回'
        if length < self.inner:
            return '腕心小于几何可达半径，请向肩部外移'
        return ''
