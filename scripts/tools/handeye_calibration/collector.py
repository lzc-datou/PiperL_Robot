"""Collect stationary, timestamp-matched observations for AgileX's hand-eye solver."""
import json
import os
import time
import traceback
import uuid
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from handeye_calibration_ros.agx_arm_calibration import AgxArmCalibrationNode, PoseMartix

from quality import check_observability, interpolate_transform, stationary_window
from feedback import resolve_robot_pose_topic


DEFAULT_CAPTURE = {
    'timeout_s': 10.0,
    'duration_s': 1.0,
    'max_translation_m': 0.001,
    'max_rotation_deg': 0.2,
    'max_robot_gap_s': 0.05,
    'max_static_gap_s': 0.1,
    'max_age_s': 0.5,
    'min_span_deg': 15.0,
    'min_axis_ratio': 0.1,
}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def pose_matrix(msg):
    p, q = msg.pose.position, msg.pose.orientation
    values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 0.01:
        raise ValueError('位姿含非有限值或无效四元数。')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def valid_pair(robot, marker, now, camera_frame):
    """Legacy input check; actual capture additionally requires images and stationarity."""
    try:
        for msg in (robot, marker):
            pose_matrix(msg)
            if not 0 <= now - stamp(msg) <= 0.5:
                return False
    except ValueError:
        return False
    return abs(stamp(robot) - stamp(marker)) <= 0.10 and marker.header.frame_id == camera_frame


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(temporary, path)


def header_record(msg):
    return {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec,
            'frame_id': msg.header.frame_id}


class CheckedCalibration(AgxArmCalibrationNode):
    def __init__(self, output, camera_frame, capture_config=None, camera_topic=None):
        super().__init__()
        self.output = output
        self.camera_frame = camera_frame
        supplied = dict(capture_config or {})
        self.base_frame = supplied.pop('base_frame', 'base_link')
        self.capture = {**DEFAULT_CAPTURE, **supplied}
        unknown = set(self.capture) - set(DEFAULT_CAPTURE)
        if unknown:
            raise ValueError(f'未知采样参数：{sorted(unknown)}')
        for key, value in self.capture.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'采样参数 {key} 必须是正的有限数值。')
        if self.capture['timeout_s'] <= self.capture['duration_s']:
            raise ValueError('timeout_s 必须大于 duration_s。')
        self.camera_topic = camera_topic or '/handeye/handeye_d405/color'
        self.bridge = CvBridge()
        self.accepted_samples = []
        self.used_frame_stamps = set()
        self._pending_sample = None
        self.robots, self.markers, self.images, self.infos = (OrderedDict() for _ in range(4))
        self.robot_stamp_revisions = OrderedDict()

    @staticmethod
    def _cache(cache, msg, limit):
        key = stamp_ns(msg)
        if key in cache:
            if isinstance(msg, PoseStamped) and (
                    msg.header.frame_id != cache[key].header.frame_id or
                    not np.allclose(pose_matrix(msg), pose_matrix(cache[key]), atol=1e-9, rtol=0)):
                raise ValueError('同一源时间戳收到不同位姿，请检查反馈或检测器。')
            return
        cache[key] = msg
        while len(cache) > limit:
            cache.popitem(last=False)

    def _cache_robot(self, msg, limit=4096):
        """Keep one observation per SDK stamp while recording bounded CAN-fragment revisions."""
        key, transform = stamp_ns(msg), pose_matrix(msg)
        if key in self.robots:
            revision = self.robot_stamp_revisions[key]
            if msg.header.frame_id != revision['header']['frame_id']:
                raise ValueError('同一机器人源时间戳的 frame 发生变化。')
            first = np.asarray(revision['first_base_T_end'])
            translation = float(np.linalg.norm(transform[:3, 3] - first[:3, 3]))
            angle = float(np.degrees(Rotation.from_matrix(first[:3, :3].T @ transform[:3, :3]).magnitude()))
            if translation > self.capture['max_translation_m'] or angle > self.capture['max_rotation_deg']:
                raise ValueError('同一机器人源时间戳的变化超过停稳阈值，拒绝本次采样。')
            if not np.allclose(transform, revision['last_base_T_end'], atol=1e-12, rtol=0):
                revision['revision_count'] += 1
                revision['translation_from_first_max_mm'] = max(revision['translation_from_first_max_mm'], translation * 1000)
                revision['rotation_from_first_max_deg'] = max(revision['rotation_from_first_max_deg'], angle)
                revision['last_base_T_end'] = transform.tolist()
            self.robots[key] = msg
            return
        self.robots[key] = msg
        self.robot_stamp_revisions[key] = {
            'header': header_record(msg), 'first_base_T_end': transform.tolist(),
            'last_base_T_end': transform.tolist(), 'revision_count': 0,
            'translation_from_first_max_mm': 0.0, 'rotation_from_first_max_deg': 0.0,
        }
        while len(self.robots) > limit:
            discarded, _ = self.robots.popitem(last=False)
            self.robot_stamp_revisions.pop(discarded)

    def _sample_candidate(self, marker, requested_at, now):
        """Require one observed stationary window ending at the image exposure time."""
        settings = self.capture
        key, exposure = stamp_ns(marker), stamp(marker)
        if key in self.used_frame_stamps:
            raise ValueError('此曝光帧已经采过。')
        if exposure < requested_at + settings['duration_s']:
            raise ValueError('正在等待本次 Enter 之后的完整静止窗口。')
        if not 0 <= now - exposure <= settings['max_age_s']:
            raise ValueError('检测位姿已过期或时间戳来自未来。')
        if marker.header.frame_id != self.camera_frame:
            raise ValueError('检测位姿的相机坐标系不匹配。')
        camera_T_marker = pose_matrix(marker)
        if key not in self.images or key not in self.infos:
            raise ValueError('等待与检测位姿同一曝光时间的原图和 CameraInfo。')
        image, info = self.images[key], self.infos[key]
        if image.header.frame_id != self.camera_frame or info.header.frame_id != self.camera_frame:
            raise ValueError('原图或 CameraInfo 的相机坐标系不匹配。')
        if image.width != info.width or image.height != info.height:
            raise ValueError('原图与 CameraInfo 的尺寸不一致。')
        intrinsics = np.asarray(info.k, dtype=float).reshape(3, 3)
        if not np.isfinite(intrinsics).all() or not np.isfinite(info.d).all() or min(intrinsics[0, 0], intrinsics[1, 1]) <= 0:
            raise ValueError('CameraInfo 内参无效。')
        history, source_headers = [], {}
        for robot in self.robots.values():
            frame = robot.header.frame_id
            if frame and frame != self.base_frame:
                raise ValueError(f'机器人反馈 frame 为 {frame}，预期 {self.base_frame}。')
            if not 0 <= now - stamp(robot) <= settings['duration_s'] + settings['max_age_s'] + settings['max_static_gap_s']:
                continue
            history.append((stamp(robot), pose_matrix(robot)))
            source_headers[stamp(robot)] = header_record(robot)
        history.sort(key=lambda item: item[0])
        stationary = stationary_window(
            history, exposure, duration_s=settings['duration_s'],
            max_translation_m=settings['max_translation_m'], max_rotation_deg=settings['max_rotation_deg'],
            max_gap_s=settings['max_static_gap_s'])
        base_T_end, interpolation = interpolate_transform(history, exposure, max_gap_s=settings['max_robot_gap_s'])
        bracket_times = {interpolation['before_stamp_s'], interpolation['after_stamp_s']}
        bracket = [{'header': source_headers[timestamp], 'base_T_end': transform.tolist()}
                   for timestamp, transform in history if timestamp in bracket_times]
        history_stamps = {header['sec'] * 1_000_000_000 + header['nanosec'] for header in source_headers.values()}
        revisions = [dict(revision) for key, revision in self.robot_stamp_revisions.items()
                     if key in history_stamps and revision['revision_count']]
        sample_id = uuid.uuid4().hex
        image_path = f'images/{sample_id}.png'
        (self.output / 'images').mkdir(exist_ok=True)
        frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        if not cv2.imwrite(str(self.output / image_path), frame):
            raise OSError('原始图像保存失败。')
        robot = PoseStamped()
        robot.header.stamp = marker.header.stamp
        robot.header.frame_id = self.base_frame
        robot.pose.position.x, robot.pose.position.y, robot.pose.position.z = map(float, base_T_end[:3, 3])
        quaternion = Rotation.from_matrix(base_T_end[:3, :3]).as_quat()
        robot.pose.orientation.x, robot.pose.orientation.y, robot.pose.orientation.z, robot.pose.orientation.w = map(float, quaternion)
        self._pending_sample = {
            'id': sample_id, 'base_T_end': base_T_end.tolist(), 'camera_T_marker': camera_T_marker.tolist(),
            'source': {'robot_pose_topic': self.agx_arm_topic,
                       'marker': header_record(marker), 'image': header_record(image),
                       'camera_info': header_record(info), 'robot_headers': list(source_headers.values()),
                       'robot_bracket': bracket,
                       'robot_history': [{'header': source_headers[timestamp], 'base_T_end': transform.tolist()}
                                         for timestamp, transform in history],
                       'robot_stamp_revisions': revisions,
                       'robot_stamp_revision_note': 'Repeated SDK source stamps count once; bounded CAN-fragment revisions use the last received pose. This does not establish hardware synchronization.',
                       'robot_empty_frame_assumed_as': self.base_frame if any(not item['frame_id'] for item in source_headers.values()) else None},
            'interpolation': interpolation, 'stationarity': stationary,
            'exposure_age_s': now - exposure, 'image_path': image_path,
            'camera_info': {'width': info.width, 'height': info.height, 'distortion_model': info.distortion_model,
                            'k': list(info.k), 'd': list(info.d), 'r': list(info.r), 'p': list(info.p)},
        }
        return marker, robot

    def subscribe_message(self):
        self.robots.clear()
        self.robot_stamp_revisions.clear()
        self.markers.clear()
        self.images.clear()
        self.infos.clear()
        self._pending_sample = None
        subscriptions = []
        requested_at = self.get_clock().now().nanoseconds * 1e-9
        deadline = time.monotonic() + self.capture['timeout_s']
        reason = '未收到相机、检测位姿或真实机器人反馈。'
        try:
            streams = [(PoseStamped, self.agx_arm_topic, self.robots, 4096),
                       (PoseStamped, self.marker_topic, self.markers, 64),
                       (Image, self.camera_topic + '/image_raw', self.images, 64),
                       (CameraInfo, self.camera_topic + '/camera_info', self.infos, 64)]
            for message_type, topic, cache, limit in streams:
                callback = (lambda msg: self._cache_robot(msg)) if cache is self.robots else (
                    lambda msg, cache=cache, limit=limit: self._cache(cache, msg, limit))
                subscriptions.append(self.create_subscription(
                    message_type, topic, callback, qos_profile_sensor_data))
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                now = self.get_clock().now().nanoseconds * 1e-9
                # Check the newest complete exposure. Repeating a full history
                # check for every cached frame delays feedback while the arm moves.
                candidates = [marker for marker in reversed(self.markers.values())
                              if stamp_ns(marker) in self.images and stamp_ns(marker) in self.infos]
                if not candidates and self.markers:
                    reason = '等待与检测位姿同一曝光时间的原图和 CameraInfo。'
                for marker in candidates[:1]:
                    try:
                        return self._sample_candidate(marker, requested_at, now)
                    except ValueError as error:
                        reason = str(error)
            diagnostics = self._input_diagnostics(now=self.get_clock().now().nanoseconds * 1e-9)
            atomic_json(self.output / 'input_diagnostics.json', diagnostics)
            counts = ', '.join(f'{name}={value["unique_messages"]}' for name, value in diagnostics['streams'].items())
            if not self.robots:
                reason = f'机器人话题 {self.agx_arm_topic} 未收到消息，请检查控制会话的话题与 ROS 域'
            elif not any(0 <= diagnostics['now_s'] - stamp(message) <=
                         self.capture['duration_s'] + self.capture['max_age_s'] + self.capture['max_static_gap_s']
                         for message in self.robots.values()):
                age = diagnostics['streams']['robot']['latest_age_s']
                reason = f'机器人反馈已到达，但源时间戳未通过新鲜度检查，最新一帧年龄 {age:.3f} 秒'
            raise RuntimeError(f'{self.capture["timeout_s"]:g} 秒内未获得合格样本：{reason}；已收 {counts}。')
        except Exception:
            (self.output / 'sampling_error.txt').write_text(traceback.format_exc())
            raise
        finally:
            for subscription in subscriptions:
                self.destroy_subscription(subscription)

    def _input_diagnostics(self, now):
        streams = {}
        for name, topic, cache in (
                ('robot', self.agx_arm_topic, self.robots), ('marker', self.marker_topic, self.markers),
                ('image', self.camera_topic + '/image_raw', self.images),
                ('camera_info', self.camera_topic + '/camera_info', self.infos)):
            latest = max(cache.values(), key=stamp_ns) if cache else None
            streams[name] = {'topic': topic, 'unique_messages': len(cache),
                             'latest_header': header_record(latest) if latest is not None else None,
                             'latest_age_s': now - stamp(latest) if latest is not None else None}
        return {'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'), 'now_s': now, 'streams': streams}

    def _write_samples(self):
        accepted = getattr(self, 'accepted_samples', [])
        samples = []
        for index, (robot, marker) in enumerate(zip(self.agx_arm_poses, self.marker_poses)):
            metadata = dict(accepted[index]) if index < len(accepted) else {}
            samples.append({**metadata, 'base_T_end': matrix(robot).tolist(), 'camera_T_marker': matrix(marker).tolist()})
        atomic_json(self.output / 'samples.json', samples)
        return samples

    def get_poses(self):
        while rclpy.ok():
            count = len(self.agx_arm_poses)
            print(f'\n已保存 {count} 组；至少 {self.min_num} 组才能求解。', flush=True)
            command = input('请输入(Enter-静止采样, d-删除上一组, q-计算并退出, c-仅保存退出): ').strip()
            if command == '':
                try:
                    marker, robot = self.subscribe_message()
                except (RuntimeError, ValueError) as error:
                    print(f'本次未采入：{error}；原有样本保留，请调整后再按 Enter。', flush=True)
                    continue
                self.agx_arm_poses.append(PoseMartix(robot.pose, True, self.mode))
                self.marker_poses.append(PoseMartix(marker.pose, False, self.mode))
                self.accepted_samples.append(self._pending_sample)
                self.used_frame_stamps.add(stamp_ns(marker))
                self._write_samples()
                print(f'已采入第 {len(self.agx_arm_poses)} 组，原图和时间配对信息已保存。', flush=True)
            elif command == 'd':
                if count:
                    self.agx_arm_poses.pop()
                    self.marker_poses.pop()
                    self.accepted_samples.pop()
                    self._write_samples()
                    print('已删除上一组；原图保留用于复查。', flush=True)
                else:
                    print('当前没有样本。', flush=True)
            elif command == 'c':
                self._write_samples()
                raise SystemExit(0)
            elif command == 'q':
                if count < self.min_num:
                    print(f'样本不足，还需要 {self.min_num - count} 组。', flush=True)
                    continue
                try:
                    self._observability()
                except ValueError as error:
                    print(f'不能求解：{error}；请补充绕不同轴旋转的姿态。', flush=True)
                    continue
                return
            else:
                print('无效输入。', flush=True)
        raise SystemExit(0)

    def _observability(self):
        settings = getattr(self, 'capture', DEFAULT_CAPTURE)
        return check_observability([matrix(pose) for pose in self.agx_arm_poses],
                                   min_span_deg=settings['min_span_deg'],
                                   min_axis_ratio=settings['min_axis_ratio'])

    def process_handeye(self):
        samples = self._write_samples()
        observability = self._observability()
        result = super().process_handeye()
        transform = matrix(result)
        if not np.isfinite(transform).all():
            raise RuntimeError('标定结果不是有限数值，请重新采集包含多轴旋转的姿态。')
        boards = np.array([matrix(a) @ transform @ matrix(b)
                           for a, b in zip(self.agx_arm_poses, self.marker_poses)])
        spread_mm = np.linalg.norm(boards[:, :3, 3] - np.mean(boards[:, :3, 3], axis=0), axis=1) * 1000
        rotations = Rotation.from_matrix(boards[:, :3, :3])
        spread_deg = np.degrees((rotations.mean().inv() * rotations).magnitude())
        report = {'samples': len(samples), 'translation_spread_max_mm': float(max(spread_mm)),
                  'translation_spread_rms_mm': float(np.sqrt(np.mean(spread_mm ** 2))),
                  'rotation_spread_max_deg': float(max(spread_deg)), 'observability': observability,
                  'note': 'Training-sample consistency only; not independent accuracy validation.'}
        atomic_json(self.output / 'T_end_camera.json', transform.tolist())
        atomic_json(self.output / 'consistency.json', report)
        print('标记在基坐标系中的一致性（仅采样内残差）:', report, flush=True)
        return result


def matrix(pose):
    value = np.eye(4)
    value[:3, :3] = pose.rotation_matrix
    value[:3, 3] = np.asarray(pose.translation_matrix).reshape(3)
    return value


def run(cfg, output, camera_frame):
    args = ['--ros-args', '-p', 'mode:=eye_in_hand', '-p', f'min_num:={cfg["min_samples"]}',
            '-p', f'agx_arm_topic:={cfg["robot_pose_topic"]}', '-p', f'marker_topic:={cfg["marker_pose_topic"]}',
            '-p', f'result_save_path:={output}']
    rclpy.init(args=args)
    node = None
    try:
        capture = {**cfg.get('capture', {}), 'base_frame': cfg['base_frame']}
        topic = f'/{cfg["camera_namespace"]}/{cfg["camera_name"]}/color'
        node = CheckedCalibration(output, camera_frame, capture_config=capture, camera_topic=topic)
        node.agx_arm_topic = resolve_robot_pose_topic(node, cfg['robot_pose_topic'])
        session_path = output / 'session.json'
        if session_path.exists():
            session = json.loads(session_path.read_text())
            session['robot_pose_topic_requested'] = cfg['robot_pose_topic']
            session['robot_pose_topic'] = node.agx_arm_topic
            atomic_json(session_path, session)
        node.run()
    except SystemExit as error:
        if error.code not in (0, None):
            raise
        print('已保存当前样本，未重新求解。', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('已取消等待或采样，已接受的样本会保留。', flush=True)
    finally:
        if node is not None:
            node._write_samples()
            node.destroy_node()
        rclpy.try_shutdown()
