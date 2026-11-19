"""Publish a fixed chessboard's pose in the D405 color optical frame."""
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image


def board_settings(settings):
    settings = settings or {}
    if not isinstance(settings, dict):
        raise SystemExit('chessboard 必须包含 columns、rows、square_size_mm。')
    pattern = (settings.get('columns'), settings.get('rows'))
    size = settings.get('square_size_mm')
    if (any(type(n) is not int or n < 3 for n in pattern)
            or type(size) not in (int, float) or not math.isfinite(size) or size <= 0):
        raise SystemExit('棋盘格内角点行列数须为不小于 3 的整数，每格边长须为正数（毫米）。')
    return pattern, size / 1000


def object_grid(pattern, size):
    columns, rows = pattern
    points = np.zeros((columns * rows, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2) * size
    return points


def ordered_corners(gray, pattern):
    found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    columns, rows = pattern
    if (columns + rows) % 2 == 0:
        # A plain board of this parity cannot distinguish the two opposite origins.
        # The preview shows which origin the detector used so the operator can check.
        return corners
    grid = corners.reshape(rows, columns, 2)
    centers = (grid[:-1, :-1] + grid[1:, :-1] + grid[:-1, 1:] + grid[1:, 1:]) / 4
    x, y = np.rint(centers).astype(int).transpose(2, 0, 1)
    if np.any(x < 0) or np.any(x >= gray.shape[1]) or np.any(y < 0) or np.any(y >= gray.shape[0]):
        return None
    # For mixed parity (including 9x12), reversal swaps the two colors.
    # Pick the origin whose first interior cell is black.
    parity = np.indices(x.shape).sum(axis=0) % 2 == 0
    values = gray[y, x]
    contrast = float(np.median(values[~parity]) - np.median(values[parity]))
    if abs(contrast) < 20:
        return None
    return corners if contrast > 0 else corners[::-1].copy()


def solve_pose(points, corners, intrinsics, distortion):
    success, rvec, tvec = cv2.solvePnP(points, corners, intrinsics, distortion)
    if not success or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    if np.any((points @ rotation.T + tvec.ravel())[:, 2] <= 0):
        return None
    projected, _ = cv2.projectPoints(points, rvec, tvec, intrinsics, distortion)
    rms = float(np.sqrt(np.mean(np.sum((corners - projected) ** 2, axis=2))))
    if not math.isfinite(rms) or rms > 1.0:
        return None
    return rotation, tvec.ravel(), rms


class ChessboardDetector(Node):
    def __init__(self, color_topic, frame, pose_topic, preview_topic, pattern, size):
        super().__init__('handeye_chessboard')
        self.frame, self.pattern = frame, pattern
        self.points = object_grid(pattern, size)
        self.bridge = CvBridge()
        self.poses = self.create_publisher(PoseStamped, pose_topic, 1)
        self.preview = self.create_publisher(Image, preview_topic, 1)
        self.image = Subscriber(self, Image, f'{color_topic}/image_raw', qos_profile=qos_profile_sensor_data)
        self.info = Subscriber(self, CameraInfo, f'{color_topic}/camera_info', qos_profile=qos_profile_sensor_data)
        self.sync = TimeSynchronizer([self.image, self.info], 5)
        self.sync.registerCallback(self.detect)
        self.get_logger().info(f'等待同时间戳的 D405 彩色图和内参，内角点 {pattern}，每格 {size * 1000:g} mm。')
        if sum(pattern) % 2 == 0:
            self.get_logger().warning('棋盘转 180° 后图案相同；采样前须在预览中核对原点始终对应同一实物角点。')

    def detect(self, image, info):
        if (image.header.frame_id != self.frame or info.header.frame_id != self.frame
                or (image.width, image.height) != (info.width, info.height)):
            self.get_logger().warning('彩色图与 CameraInfo 的 frame 或尺寸不匹配。', throttle_duration_sec=5.0)
            return
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        supported = {'plumb_bob': 5, 'rational_polynomial': 8}
        if (supported.get(info.distortion_model) != d.size or not np.isfinite(d).all()
                or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0):
            self.get_logger().warning('D405 的 CameraInfo 内参无效或畸变模型不受支持。', throttle_duration_sec=5.0)
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            corners = ordered_corners(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), self.pattern)
            result = None if corners is None else solve_pose(self.points, corners, k, d)
            if result is not None:
                rotation, translation, rms = result
                pose = PoseStamped(header=image.header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, translation)
                q = Rotation.from_matrix(rotation).as_quat()
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
                self.poses.publish(pose)
            if self.preview.get_subscription_count():
                overlay = bgr.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(overlay, self.pattern, corners, True)
                label = 'No complete chessboard' if corners is None else 'Pose rejected'
                if result is not None:
                    origin = tuple(np.rint(corners[0, 0]).astype(int))
                    cv2.circle(overlay, origin, 9, (255, 255, 0), 2)
                    cv2.putText(overlay, '0', origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    label = f'POSE OK / RMS {rms:.2f} px'
                cv2.putText(overlay, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
                preview = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                preview.header = image.header
                self.preview.publish(preview)
        except (cv2.error, CvBridgeError) as error:
            self.get_logger().warning(f'棋盘格图像处理失败: {error}', throttle_duration_sec=5.0)


def run(*args):
    rclpy.init(args=[])
    node = ChessboardDetector(*args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
