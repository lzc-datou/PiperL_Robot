"""Publish a fixed chessboard's pose in the D405 color optical frame."""
from collections import deque
import json
import math
from threading import Event, Lock, Thread
import time

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from message_filters import SimpleFilter, Subscriber, TimeSynchronizer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


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


def ordered_corners(gray, pattern, *, fast=False):
    if fast:
        flags = cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if found:
            cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 20, .01))
    else:
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


def visible_grid(gray, corners, pattern):
    """Every inner corner must still separate the four alternating board cells."""
    cols, rows = pattern
    grid = corners.reshape(rows, cols, 2)
    # Local row/column directions account for perspective and board rotation.
    dy, dx = np.gradient(grid, axis=(0, 1))
    # The outer squares may extend beyond the frame even when all inner
    # corners are visible. Keep all four probes on the image in that case.
    margin = np.minimum(grid, np.array([gray.shape[1]-1, gray.shape[0]-1])-grid)
    fraction = np.minimum(.28, np.min(margin / np.maximum(abs(dx)+abs(dy), .01), axis=2))
    if not np.isfinite(fraction).all() or np.any(fraction < .10):
        return False
    samples = np.stack([grid + fraction[..., None]*(sx*dx + sy*dy)
                        for sx, sy in ((1, 1), (-1, -1), (-1, 1), (1, -1))])
    values = cv2.remap(gray, samples.reshape(-1, cols, 2).astype(np.float32), None, cv2.INTER_LINEAR)
    a, b, c, d = values.reshape(4, rows, cols).astype(np.int16)
    parity = np.indices((rows, cols)).sum(axis=0) % 2 == 0
    # Mixed-parity grids have a canonical dark first interior square. With
    # indistinguishable ends either consistent checker polarity is valid.
    if (cols + rows) % 2 == 0 and np.median((c+d-a-b)[parity]) < 0:
        parity = ~parity
    contrast = np.where(parity, np.minimum(c, d) - np.maximum(a, b),
                        np.minimum(a, b) - np.maximum(c, d))
    return bool(np.all(contrast >= 12))


class BoardSearch:
    """Track measured corners between exposures; reacquire the whole grid on a miss."""
    def __init__(self, tracking=False):
        self.tracking = tracking
        self.previous_gray = self.previous_corners = self.previous_stamp = None
        self.calibration_roi = None
        self.misses = 0
        self.method = ''
        self.track_ms = self.search_ms = self.pose_ms = 0.

    def track(self, gray, pattern):
        params = dict(winSize=(15, 15), maxLevel=2,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, .03))
        corners, status, _ = cv2.calcOpticalFlowPyrLK(
            self.previous_gray, gray, self.previous_corners, None, **params)
        if (corners is None or not status.all() or not np.isfinite(corners).all()
                or np.any(corners < 0) or np.any(corners >= gray.shape[::-1])):
            return None
        back, status, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous_gray, corners, None, **params)
        if (back is None or not status.all() or not np.isfinite(back).all() or
                np.max(np.linalg.norm(back-self.previous_corners, axis=2)) > 1.):
            return None
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .01))
        return corners if visible_grid(gray, corners, pattern) else None

    def locate_calibration(self, gray, pattern, points, k, d):
        # Each sample is independently localized at native resolution. A nearby
        # previous detection only narrows the first SB search, never the PnP input.
        h, w = gray.shape
        full = (0, 0, w, h)
        roi, self.calibration_roi = self.calibration_roi, None
        for left, top, right, bottom in ([roi, full] if roi and roi != full else [full]):
            corners = ordered_corners(gray[top:bottom, left:right], pattern)
            if corners is None:
                continue
            corners += np.array([left, top], dtype=np.float32)
            pose = solve_pose(points, corners, k, d)
            if pose is not None:
                x, y, width, height = cv2.boundingRect(corners)
                pad = math.ceil(max(width, height)*.25)+8
                self.calibration_roi = (max(0, x-pad), max(0, y-pad),
                                        min(w, x+width+pad), min(h, y+height+pad))
                return corners, pose
        return None, None

    def locate(self, gray, pattern, points, k, d, stamp_ns=None):
        if not self.tracking:
            return self.locate_calibration(gray, pattern, points, k, d)
        self.track_ms = self.search_ms = self.pose_ms = 0.
        corners = result = None
        fresh_pair = (stamp_ns is None or self.previous_stamp is None or
                      0 < stamp_ns-self.previous_stamp <= 150_000_000)
        if (self.previous_gray is not None and self.previous_gray.shape == gray.shape
                and len(self.previous_corners) == len(points) and fresh_pair):
            self.method = 'flow'
            start = time.perf_counter()
            corners = self.track(gray, pattern)
            self.track_ms = (time.perf_counter()-start)*1000.
            if corners is not None:
                start = time.perf_counter()
                result = solve_pose(points, corners, k, d)
                self.pose_ms = (time.perf_counter()-start)*1000.
        if result is None:
            start = time.perf_counter()
            # One whole-grid search per exposure. On a miss use native pixels
            # on the next frame for distant/small boards.
            reduced = min(gray.shape) >= 320 and self.misses % 2 == 0
            scan = gray
            if reduced:
                scan_size = (round(gray.shape[1]*.75), round(gray.shape[0]*.75))
                scan = cv2.resize(gray, scan_size, interpolation=cv2.INTER_AREA)
            self.method = 'sb75' if reduced else 'sb100'
            # SB expects a light outer margin. Only the search image is padded;
            # every measured corner and its four quadrants must pass on real pixels.
            border = 12
            padded = cv2.copyMakeBorder(scan, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
            corners = ordered_corners(padded, pattern)
            if corners is not None:
                corners -= border
                xy = corners.reshape(-1, 2)
                if np.any(xy < 0) or np.any(xy >= scan.shape[::-1]):
                    corners = None
            if corners is not None and reduced:
                ratio = np.array([gray.shape[1]/scan.shape[1], gray.shape[0]/scan.shape[0]], np.float32)
                corners = (corners + .5)*ratio - .5
                cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .01))
            if corners is not None and not visible_grid(gray, corners, pattern):
                corners = None
            self.search_ms = (time.perf_counter()-start)*1000.
            if corners is not None:
                start = time.perf_counter()
                result = solve_pose(points, corners, k, d)
                self.pose_ms += (time.perf_counter()-start)*1000.
        if result is None:
            self.previous_gray = self.previous_corners = self.previous_stamp = None
            self.misses += 1
        else:
            self.previous_gray, self.previous_corners, self.previous_stamp = gray, corners, stamp_ns
            self.misses = 0
        return corners, result


class CaptureClock:
    """Estimate exposure time from matched device and host-kernel frame metadata."""
    def __init__(self):
        self.offsets = deque(maxlen=30)
        self.previous_hw = self.previous_kernel = self.last_stamp = 0

    def stamp(self, data, now_ns):
        if not isinstance(data, dict):
            raise ValueError('相机元数据格式无效')
        if 'hw_timestamp' in data:
            hw = int(data['hw_timestamp']) * 1000  # device readout, microseconds
        elif data.get('clock_domain') == 'hardware_clock':
            hw = round(float(data['frame_timestamp'])*1_000_000)  # SDK header in ms
        else:
            raise ValueError('缺少设备读出时刻')
        backend = int(data['backend_timestamp'])
        # SDK releases expose host epoch in either milliseconds or microseconds.
        kernel = backend * (1000 if backend >= 10**15 else 10**6)
        if hw <= 0 or not 0 <= now_ns-kernel <= 2_000_000_000:
            raise ValueError('内核帧时间无效或已过期')
        if 'sensor_timestamp' in data:
            readout_lag = hw - int(data['sensor_timestamp'])*1000
        else:
            readout_lag = int(data['actual_exposure'])*500  # half the exposure in ns
        if not 0 <= readout_lag <= 100_000_000:
            raise ValueError('曝光与读出时差无效')
        if hw <= self.previous_hw or kernel <= self.previous_kernel or kernel-self.previous_kernel > 1_000_000_000:
            self.offsets.clear()
        self.previous_hw, self.previous_kernel = hw, kernel
        self.offsets.append(kernel-hw)
        if len(self.offsets) < 4:
            raise ValueError('正在同步设备采集时间与主机内核时钟')
        # Use the earliest observed transport offset. A late DDS callback or
        # a single slow USB delivery must not advance this frame's exposure.
        stamp = hw + min(self.offsets) - readout_lag
        if stamp <= self.last_stamp or stamp > now_ns:
            raise ValueError('校正后的曝光时间倒退或超前')
        self.last_stamp = stamp
        return stamp


class ChessboardDetector(Node):
    def __init__(self, color_topic, frame, pose_topic, preview_topic, pattern, size, tracking=False, camera_metadata=False):
        super().__init__('handeye_chessboard')
        self.frame, self.pattern = frame, pattern
        self.points = object_grid(pattern, size)
        self.search = BoardSearch(tracking=tracking)
        self.bridge = CvBridge()
        self.poses = self.create_publisher(PoseStamped, pose_topic, 1)
        self.preview = self.create_publisher(Image, preview_topic, 1)
        self.misses = self.create_publisher(Header, pose_topic + "/not_visible", 1)
        if tracking and camera_metadata:
            # Start compatible with an externally launched camera. A reliable
            # reader can repair missing fragments of the large RGB message when
            # the D405 offers reliable delivery. Keep the DDS history short.
            self.raw_topic = f'{color_topic}/image_raw'
            self.image = SimpleFilter()
            self.image_reader = self.image_reliability = None
            self.match_image_publisher()
            self.create_timer(.25, self.match_image_publisher)
        else:
            self.image = Subscriber(self, Image, f'{color_topic}/image_raw', qos_profile=qos_profile_sensor_data)
        self.info = Subscriber(self, CameraInfo, f'{color_topic}/camera_info', qos_profile=qos_profile_sensor_data)
        streams = [self.image, self.info]
        if camera_metadata:
            from realsense2_camera_msgs.msg import Metadata
            self.capture_clock = CaptureClock()
            self.metadata = Subscriber(self, Metadata, f'{color_topic}/metadata', qos_profile=qos_profile_sensor_data)
            streams.append(self.metadata)
            self.last_metadata = time.monotonic()
            self.create_timer(2., self.check_metadata)
        self.sync = TimeSynchronizer(streams, 5)
        # Synchronization and DDS intake stay responsive while OpenCV and the
        # optional preview run. Only the latest complete exposure is pending.
        self.pair_lock = Lock()
        self.latest_pair = self.latest_preview = None
        self.overwritten = 0
        self.timing_since = time.monotonic()
        self.timing_samples = []
        self.convert_ms = self.publish_ms = 0.
        self.workers = []
        if tracking:
            self.closing = Event()
            self.wake_detection = self.start_worker(self.detect_latest)
            wake_preview = self.start_worker(self.preview_latest)
            self.create_timer(.1, wake_preview)
        else:
            ready = self.create_guard_condition(self.detect_latest, callback_group=MutuallyExclusiveCallbackGroup())
            self.wake_detection = ready.trigger
        self.sync.registerCallback(self.queue_capture if camera_metadata else self.queue_pair)
        self.get_logger().info(f'等待同时间戳的 D405 彩色图和内参，内角点 {pattern}，每格 {size * 1000:g} mm。')
        if sum(pattern) % 2 == 0:
            self.get_logger().warning('棋盘转 180° 后图案相同；采样前须在预览中核对原点始终对应同一实物角点。')

    def match_image_publisher(self):
        publishers = self.get_publishers_info_by_topic(self.raw_topic)
        reliable = ReliabilityPolicy.RELIABLE
        policy = (reliable if publishers and all(p.qos_profile.reliability == reliable for p in publishers)
                  else ReliabilityPolicy.BEST_EFFORT)
        if policy == self.image_reliability:
            return
        previous = self.image_reader
        self.image_reader = self.create_subscription(
            Image, self.raw_topic, self.image.signalMessage, QoSProfile(depth=2, reliability=policy))
        self.image_reliability = policy
        if previous is not None:
            self.destroy_subscription(previous)
        self.get_logger().info(f'D405 彩色图订阅 {policy.name}，接收队列 2 帧')

    def start_worker(self, callback):
        ready = Event()

        def work():
            while True:
                ready.wait()
                ready.clear()
                if self.closing.is_set():
                    return
                try:
                    callback()
                except Exception as exc:
                    if self.context.ok():
                        self.get_logger().error(f'{callback.__name__} 失败：{exc!r}')
                        self.context.try_shutdown()
                    return

        thread = Thread(target=work, name=callback.__name__, daemon=True)
        self.workers.append((ready, thread))
        thread.start()
        return ready.set

    def destroy_node(self):
        if self.workers:
            self.closing.set()
            for ready, _ in self.workers:
                ready.set()
            for _, thread in self.workers:
                thread.join()
        return super().destroy_node()

    def check_metadata(self):
        if time.monotonic()-self.last_metadata > 2.:
            self.get_logger().warning('等待同帧彩色图、内参和 D405 metadata，请检查相机话题', throttle_duration_sec=3.)

    def queue_capture(self, image, info, metadata):
        self.last_metadata = time.monotonic()
        if metadata.header.frame_id != image.header.frame_id:
            return
        try:
            now = self.get_clock().now().nanoseconds
            ns = self.capture_clock.stamp(json.loads(metadata.json_data), now)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.get_logger().warning(f'棋盘曝光时间不可用：{exc}', throttle_duration_sec=1.)
            return
        raw = image.header.stamp.sec*1_000_000_000 + image.header.stamp.nanosec
        self.get_logger().info(f'D405 采集时钟：原图年龄 {(now-raw)/1e6:.1f} ms，'
                               f'曝光估计年龄 {(now-ns)/1e6:.1f} ms', throttle_duration_sec=5.)
        self.queue_pair(image, info, ns)

    def queue_pair(self, image, info, capture_ns=None):
        stamp = image.header.stamp
        stamp_ns = stamp.sec*1_000_000_000 + stamp.nanosec if capture_ns is None else capture_ns
        with self.pair_lock:
            if self.latest_pair is not None:
                if stamp_ns <= self.latest_pair[0]:
                    return
                self.overwritten += 1
            self.latest_pair = stamp_ns, image, info, time.monotonic(), capture_ns
        self.wake_detection()

    def detect_latest(self):
        with self.pair_lock:
            pair, self.latest_pair = self.latest_pair, None
        if pair is not None:
            stamp, image, info, queued, capture_ns = pair
            start = time.monotonic()
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            success = self.detect(image, info) if capture_ns is None else self.detect(image, info, capture_ns)
            elapsed = time.monotonic()-start
            if self.search.tracking:
                # Preview runs in its own callback and is not part of pose latency.
                total_ms = (age+elapsed)*1000.
                self.timing_samples.append((bool(success), self.search.method == 'flow', elapsed*1000.,
                                            self.pose_age_ms if success else np.nan))
                if total_ms > 150:
                    wait = start-queued
                    self.get_logger().warning(
                        f'棋盘帧延迟：曝光至配对 {(age-wait)*1000:.0f} ms，等待检测 {wait*1000:.0f} ms，'
                        f'灰度 {self.convert_ms:.1f} ms，光流及验证 {self.search.track_ms:.1f} ms，'
                        f'整板搜索 {self.search.search_ms:.1f} ms，PnP {self.search.pose_ms:.1f} ms，'
                        f'位姿发布 {self.publish_ms:.1f} ms，路径 {self.search.method}，成功 {bool(success)}',
                        throttle_duration_sec=1.)
                if start-self.timing_since >= 5.:
                    data = np.asarray(self.timing_samples)
                    valid = data[:, 0].astype(bool)
                    age_p95 = f'{np.percentile(data[valid, 3], 95):.0f}' if valid.any() else '—'
                    with self.pair_lock:
                        overwritten, self.overwritten = self.overwritten, 0
                    self.get_logger().info(
                        f'棋盘跟踪 {start-self.timing_since:.1f} s：处理 {len(data)} 帧，位姿 {sum(valid)}，'
                        f'光流通过 {int(data[:, 1].sum())}，被更新配对替换 {overwritten}，'
                        f'处理 P95 {np.percentile(data[:, 2], 95):.1f} ms，发布时曝光年龄 P95 {age_p95} ms')
                    self.timing_since = start
                    self.timing_samples.clear()

    def detect(self, image, info, capture_ns=None):
        self.convert_ms = self.publish_ms = 0.
        self.search.method = 'invalid'
        self.search.track_ms = self.search.search_ms = self.search.pose_ms = 0.
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
        header = image.header
        if capture_ns is not None:
            header = type(header)(frame_id=header.frame_id, stamp=Time(
                sec=capture_ns//1_000_000_000, nanosec=capture_ns%1_000_000_000))
        try:
            start = time.perf_counter()
            gray = self.bridge.imgmsg_to_cv2(image, 'mono8')
            stamp_ns = header.stamp.sec*10**9 + header.stamp.nanosec
            self.convert_ms = (time.perf_counter()-start)*1000.
            corners, result = self.search.locate(gray, self.pattern, self.points, k, d, stamp_ns)
            if result is not None:
                start = time.perf_counter()
                rotation, translation, _ = result
                pose = PoseStamped(header=header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, translation)
                q = Rotation.from_matrix(rotation).as_quat()
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, q)
                self.poses.publish(pose)
                self.publish_ms = (time.perf_counter()-start)*1000.
                self.pose_age_ms = (self.get_clock().now().nanoseconds-stamp_ns)/1e6
            elif corners is None:
                # A processed image without a grid distinguishes occlusion from lost camera input.
                self.misses.publish(header)
            if self.preview.get_subscription_count():
                preview = image, header, corners, result
                if self.search.tracking:
                    with self.pair_lock:
                        self.latest_preview = preview
                else:
                    self.publish_preview(*preview)
            return result is not None
        except (cv2.error, CvBridgeError) as error:
            self.get_logger().warning(f'棋盘格图像处理失败：{error}', throttle_duration_sec=5.0)

    def preview_latest(self):
        with self.pair_lock:
            preview, self.latest_preview = self.latest_preview, None
        if preview is not None and self.preview.get_subscription_count():
            start = time.perf_counter()
            self.publish_preview(*preview)
            elapsed = (time.perf_counter()-start)*1000.
            if elapsed > 25:
                self.get_logger().warning(f'棋盘预览绘制与发布 {elapsed:.1f} ms', throttle_duration_sec=1.)

    def publish_preview(self, image, header, corners, result):
        try:
            overlay = self.bridge.imgmsg_to_cv2(image, 'bgr8').copy()
            if corners is not None:
                cv2.drawChessboardCorners(overlay, self.pattern, corners, True)
            label = 'No complete chessboard' if corners is None else 'Pose rejected'
            if result is not None:
                origin = tuple(np.rint(corners[0, 0]).astype(int))
                cv2.circle(overlay, origin, 9, (255, 255, 0), 2)
                cv2.putText(overlay, '0', origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                stamp = header.stamp
                age_ms = (self.get_clock().now().nanoseconds - (stamp.sec*10**9 + stamp.nanosec))/10**6
                label = f'POSE OK / RMS {result[2]:.2f} px / exposure age {age_ms:.0f} ms'
            cv2.putText(overlay, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            msg.header = header
            self.preview.publish(msg)
        except (cv2.error, CvBridgeError) as error:
            self.get_logger().warning(f'棋盘格预览失败：{error}', throttle_duration_sec=5.)


def spin_detector(node):
    cv2.setNumThreads(min(4, max(1, cv2.getNumThreads())))
    # For tracking the ROS executor only pairs DDS input and wakes workers.
    # Neither OpenCV nor a blocked image subscriber occupies its receive thread.
    executor = (SingleThreadedExecutor(context=node.context) if node.search.tracking else
                MultiThreadedExecutor(num_threads=2, context=node.context))
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def run(*args):
    rclpy.init(args=[])
    node = ChessboardDetector(*args)
    try:
        spin_detector(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
