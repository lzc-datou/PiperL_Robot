"""Read camera images, camera TF and official TCP feedback from ROS 2."""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import time

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from scripts.tools.handeye_calibration.feedback import POSE_TYPE
from scripts.tools.handeye_calibration.quality import interpolate_transform

from .geometry import (CORNERS, METHODS, Estimate, Frame, depth_estimate, depth_grid_rays,
                       marker_corners, pnp_estimate, transform_points, unit_rays)


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass
class Intrinsics:
    frame: str
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    distortion: str

    @classmethod
    def read(cls, info):
        k = np.asarray(info.k, dtype=float).reshape(3, 3)
        d = np.asarray(info.d, dtype=float)
        supported = {'plumb_bob': (0, 5), 'rational_polynomial': (8,), '': (0,)}
        if (d.size not in supported.get(info.distortion_model, ()) or not np.isfinite(d).all()
                or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                or min(info.width, info.height) <= 0 or not info.header.frame_id):
            raise ValueError('CameraInfo 内参、光学系或畸变模型无效')
        return cls(info.header.frame_id, info.width, info.height, k, d, info.distortion_model)

    def matches(self, image):
        return (self.frame, self.width, self.height) == (image.header.frame_id, image.width, image.height)

    def describe(self):
        return dict(frame_id=self.frame, width=self.width, height=self.height,
                    k=self.k.tolist(), d=self.d.tolist(), distortion_model=self.distortion)


def image_array(image, dtype, channels=1):
    dtype = np.dtype(dtype).newbyteorder('>' if image.is_bigendian else '<')
    pixel_bytes = dtype.itemsize * channels
    shape = (image.height, image.width) if channels == 1 else (image.height, image.width, channels)
    strides = (image.step, pixel_bytes) if channels == 1 else (image.step, pixel_bytes, dtype.itemsize)
    if image.step < image.width * pixel_bytes or len(image.data) < image.height * image.step:
        raise ValueError('ROS 图像缓冲区尺寸无效')
    return np.ndarray(shape, dtype=dtype, buffer=bytes(image.data), strides=strides)


def grayscale(image):
    if image.encoding == 'mono8':
        return image_array(image, 'u1')
    conversion = {'rgb8': cv2.COLOR_RGB2GRAY, 'bgr8': cv2.COLOR_BGR2GRAY}.get(image.encoding)
    if conversion is None:
        raise ValueError(f'不支持的彩色编码 {image.encoding}')
    return cv2.cvtColor(image_array(image, 'u1', 3), conversion)


def transform_matrix(position, quaternion):
    xyzw = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    xyz = np.array([position.x, position.y, position.z])
    if not np.isfinite([*xyz, *xyzw]).all() or abs(np.linalg.norm(xyzw) - 1) > 0.01:
        raise ValueError('机器人位姿或相机 TF 无效')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(xyzw).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


class Observer(Node):
    def __init__(self, config, calibration, detector, report):
        super().__init__('d405_depth_accuracy')
        self.cfg, self.calibration, self.detector, self.report = config, calibration, detector, report
        self.models = {}
        self.stream_metadata = {}
        self.depth_rays = None
        self.color_from_depth = None
        self.pending = deque()
        self.depth_frames = deque(maxlen=30)
        self.recent_rgb = deque(maxlen=60)
        self.poses = {}
        self.used_depth = deque(maxlen=60)
        self.publisher = None
        self.pose_status = 'pose_source_missing'
        self.marker_points = marker_corners(config['marker_size_mm'] / 1000)
        self.rng = np.random.default_rng(405)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pose_topic = config['robot_pose_topic']
        prefix = config['camera_namespace']
        self.create_subscription(Image, prefix + '/color/image_raw', self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, prefix + '/depth/image_rect_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, prefix + '/color/camera_info',
                                 lambda msg: self.on_info('color', msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, prefix + '/depth/camera_info',
                                 lambda msg: self.on_info('depth', msg), qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, qos_profile_sensor_data)
        self.create_timer(0.25, self.check_pose_source)

    def on_info(self, stream, info):
        try:
            model = Intrinsics.read(info)
        except ValueError:
            self.models[stream] = None
            return
        metadata = model.describe()
        if stream not in self.stream_metadata:
            self.stream_metadata[stream] = metadata
            self.report.set_camera_stream(stream, metadata)
        self.models[stream] = model if self.stream_metadata[stream] == metadata else None

    def on_color(self, image):
        key = stamp_ns(image)
        if key > 0 and key in self.recent_rgb:
            return
        self.recent_rgb.append(key)
        self.pending.append((image, time.monotonic()))

    def on_depth(self, image):
        key = stamp_ns(image)
        if key > 0:
            self.depth_frames.append((key, image))

    def check_pose_source(self):
        endpoints = self.get_publishers_info_by_topic(self.pose_topic)
        if len(endpoints) == 1 and endpoints[0].topic_type == POSE_TYPE:
            publisher = bytes(endpoints[0].endpoint_gid)
            self.pose_status = 'pose_missing'
        else:
            publisher = None
            self.pose_status = 'pose_source_conflict' if endpoints else 'pose_source_missing'
        if publisher != self.publisher:
            self.poses.clear()
            self.publisher = publisher

    def on_pose(self, message):
        if self.publisher is None or message.header.frame_id not in ('', self.calibration['base_frame']):
            return
        key = stamp_ns(message)
        if key <= 0:
            return
        try:
            self.poses[key] = transform_matrix(message.pose.position, message.pose.orientation)
        except ValueError:
            self.poses.pop(key, None)
            return
        while len(self.poses) > 240:
            self.poses.pop(min(self.poses))

    def pose_at(self, stamp):
        keys = sorted(self.poses)
        index = bisect_left(keys, stamp)
        if index < len(keys) and keys[index] == stamp:
            return self.poses[stamp], 0.0
        if index == 0 or index == len(keys):
            return None, None
        before, after = keys[index - 1], keys[index]
        gap_ms = (after - before) / 1e6
        if gap_ms > self.cfg['max_pose_gap_ms']:
            return None, gap_ms
        transform, _ = interpolate_transform([(0.0, self.poses[before]),
                                               ((after - before) / 1e9, self.poses[after])],
                                              (stamp - before) / 1e9, self.cfg['max_pose_gap_ms'] / 1000)
        return transform, gap_ms

    def next_frame(self):
        if not self.pending:
            return None
        image, received = self.pending[0]
        stamp = stamp_ns(image)
        tolerance = self.cfg['max_rgb_depth_skew_ms'] * 1e6
        depth_item = min((item for item in self.depth_frames if item[0] not in self.used_depth),
                         key=lambda item: abs(item[0] - stamp), default=None)
        if depth_item is not None and abs(depth_item[0] - stamp) > tolerance:
            depth_item = None
        base_from_feedback, gap = self.pose_at(stamp)
        have_pair = depth_item is not None and any(t >= stamp for t, _ in self.depth_frames)
        inputs_ready = have_pair and base_from_feedback is not None and all(self.models.get(n) for n in ('color', 'depth'))
        if not inputs_ready and (time.monotonic() - received) * 1000 < self.cfg['input_wait_ms']:
            return None
        self.pending.popleft()
        if depth_item is not None:
            self.used_depth.append(depth_item[0])
        frame = self.measure(image, depth_item[1] if depth_item else None, base_from_feedback, gap)
        return frame, received

    def measure(self, rgb, depth, base_from_feedback, gap):
        frame = Frame(stamp_ns(rgb), stamp_ns(depth) if depth is not None else None, {}, pose_gap_ms=gap)
        color = self.models.get('color')
        status = 'ok'
        if frame.rgb_stamp_ns <= 0:
            status = 'rgb_stamp_invalid'
        elif color is None or not color.matches(rgb):
            status = 'color_info'
        elif color.frame != self.calibration['camera_frame']:
            status = 'color_frame_mismatch'
        else:
            try:
                gray = grayscale(rgb)
                frame.pixels, status = self.detector.corners(gray)
            except (ValueError, cv2.error):
                status = 'rgb_invalid'
        if status != 'ok':
            frame.estimates = {method: Estimate(status) for method in METHODS}
            return frame
        self.save_annotation(gray, frame.pixels)
        try:
            pnp = pnp_estimate(cv2, frame.pixels, color, self.marker_points, self.cfg['pnp_max_reprojection_px'])
        except cv2.error:
            pnp = Estimate('pnp_no_solution')
        depth_result = self.measure_depth(depth, color, frame.pixels)
        frame.estimates = dict(depth=depth_result, pnp=pnp)
        base_from_camera = (base_from_feedback @ self.calibration['end_T_camera']
                            if base_from_feedback is not None else None)
        for estimate in frame.estimates.values():
            if estimate.points_m is None:
                continue
            if base_from_camera is None:
                estimate.status = 'pose_gap' if gap is not None else self.pose_status
                estimate.points_m = None
                continue
            estimate.points_m = transform_points(base_from_camera, estimate.points_m)
        return frame

    def measure_depth(self, depth, color, pixels):
        if depth is None:
            return Estimate('depth_missing')
        model = self.models.get('depth')
        if model is None or not model.matches(depth):
            return Estimate('depth_info')
        settings = self.cfg['depth']
        encodings = {'16UC1': ('u2', 0.001), '32FC1': ('f4', 1.0)}
        if depth.encoding not in encodings:
            return Estimate('depth_encoding')
        dtype, standard_scale = encodings[depth.encoding]
        scale = settings['scale_m_per_unit'] if settings['scale_m_per_unit'] is not None else standard_scale
        if self.color_from_depth is None:
            color_from_depth = np.eye(4)
            if model.frame != color.frame:
                try:
                    stamped = self.tf.lookup_transform(color.frame, model.frame, Time.from_msg(depth.header.stamp))
                    transform = stamped.transform
                    color_from_depth = transform_matrix(transform.translation, transform.rotation)
                except (TransformException, ValueError):
                    return Estimate('depth_color_tf')
            self.color_from_depth = color_from_depth
            self.report.metadata['depth_input'] = dict(encoding=depth.encoding, scale_m_per_unit=scale,
                                                      color_T_depth=color_from_depth.tolist())
            self.report.write_metadata()
        elif self.report.metadata['depth_input']['encoding'] != depth.encoding:
            return Estimate('depth_encoding')
        try:
            stride = settings['pixel_stride']
            if self.depth_rays is None:
                self.depth_rays = depth_grid_rays(cv2, model, stride)
            depth_m = image_array(depth, dtype)[::stride, ::stride].ravel() * scale
            rays = unit_rays(cv2, pixels, color)
            return depth_estimate(cv2, rays, depth_m, self.depth_rays, self.color_from_depth, settings, self.rng)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            return Estimate('depth_invalid')

    def save_annotation(self, gray, pixels):
        if self.report.annotation_saved:
            return
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for index, (corner, pixel) in enumerate(zip(CORNERS, pixels)):
            u, v = map(int, np.rint(pixel))
            cv2.circle(image, (u, v), 4, (0, 225, 0), 2)
            cv2.putText(image, f'{index} {corner}', (max(0, u - 12), max(14, v - 7)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 1, cv2.LINE_AA)
        ok, png = cv2.imencode('.png', image)
        if not ok:
            raise OSError('无法编码角点编号图')
        (self.report.directory / 'corners.png').write_bytes(png.tobytes())
        self.report.annotation_saved = True
