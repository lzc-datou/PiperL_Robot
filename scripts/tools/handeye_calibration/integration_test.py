"""Exercise synthetic ROS images through the real detector and sample persistence.

Run this script explicitly in the hand-eye dependency environment. It creates an
isolated ROS domain and unique topics; it never opens a camera, CAN interface or
robot driver. All generated files are removed with its temporary directory.
The ordinary offline-test command intentionally does not run this ROS test.
"""
import copy
import json
import os
from pathlib import Path
import signal
import tempfile
import threading
import time
import uuid
from unittest.mock import patch

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from chessboard import ChessboardDetector, object_grid
from collector import CheckedCalibration, stamp_ns
from feedback import resolve_robot_pose_topic


def synthetic_board():
    pattern, square_size = (8, 11), 0.015
    intrinsics = np.array([[620., 0., 320.], [0., 620., 240.], [0., 0., 1.]])
    distortion = np.zeros(5)
    rotation = Rotation.from_euler('xyz', [0.2, -0.15, 0.1])
    center = object_grid(pattern, square_size).mean(axis=0)
    translation = np.array([0., 0., 0.43]) - rotation.as_matrix() @ center
    image = np.full((480, 640, 3), 255, np.uint8)
    for y in range(-1, pattern[1]):
        for x in range(-1, pattern[0]):
            if (x + y) % 2:
                continue
            cell = np.array([[x, y, 0], [x + 1, y, 0],
                             [x + 1, y + 1, 0], [x, y + 1, 0]], float) * square_size
            pixels, _ = cv2.projectPoints(cell, rotation.as_rotvec(), translation, intrinsics, distortion)
            cv2.fillConvexPoly(image, np.rint(pixels.reshape(4, 2)).astype(np.int32), (0, 0, 0))
    expected = np.eye(4)
    expected[:3, :3], expected[:3, 3] = rotation.as_matrix(), translation
    return image, intrinsics, distortion, expected


class SyntheticCameraAndRobot(Node):
    def __init__(self, namespace, camera_frame, base_frame, robot_topic):
        super().__init__('handeye_synthetic_fixture_' + namespace.rsplit('/', 1)[-1])
        self.camera_frame, self.base_frame = camera_frame, base_frame
        self.image_array, self.intrinsics, self.distortion, self.expected_marker = synthetic_board()
        self.expected_robot = np.eye(4)
        self.expected_robot[:3, :3] = Rotation.from_euler('xyz', [0.15, -0.20, 0.25]).as_matrix()
        self.expected_robot[:3, 3] = [0.35, 0.10, 0.20]
        self.image_template = CvBridge().cv2_to_imgmsg(self.image_array, encoding='bgr8')
        self.published_image_stamps, self.published_robot_stamps = set(), set()
        self.image_publisher = self.create_publisher(Image, namespace + '/color/image_raw', qos_profile_sensor_data)
        self.info_publisher = self.create_publisher(CameraInfo, namespace + '/color/camera_info', qos_profile_sensor_data)
        self.robot_topic = robot_topic
        self.robot_publisher = self.robot_timer = self.robot_enable_timer = None
        self.robot_publisher_created_s = None
        self.camera_timer = self.create_timer(0.10, self.publish_camera)

    def start_robot_feedback_after(self, delay_s):
        """Keep the graph empty initially, then create a namespaced TCP publisher."""
        def enable_robot_feedback():
            self.robot_enable_timer.cancel()
            self.robot_publisher = self.create_publisher(PoseStamped, self.robot_topic, qos_profile_sensor_data)
            self.robot_publisher_created_s = time.monotonic()
            # Match the official driver's default feedback publication rate.
            self.robot_timer = self.create_timer(0.005, self.publish_robot)
        self.robot_enable_timer = self.create_timer(delay_s, enable_robot_feedback)

    def publish_robot(self):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, self.expected_robot[:3, 3])
        quaternion = Rotation.from_matrix(self.expected_robot[:3, :3]).as_quat()
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = map(float, quaternion)
        self.published_robot_stamps.add(stamp_ns(message))
        self.robot_publisher.publish(message)

    def publish_camera(self):
        image = copy.deepcopy(self.image_template)
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.camera_frame
        info = CameraInfo()
        info.header = copy.deepcopy(image.header)
        info.width, info.height = image.width, image.height
        info.distortion_model = 'plumb_bob'
        info.k = self.intrinsics.ravel().tolist()
        info.d = self.distortion.tolist()
        info.r = np.eye(3).ravel().tolist()
        info.p = np.column_stack([self.intrinsics, np.zeros(3)]).ravel().tolist()
        self.published_image_stamps.add(stamp_ns(image))
        self.image_publisher.publish(image)
        self.info_publisher.publish(info)


def verify_sample(output, fixture, camera_frame, base_frame):
    samples = json.loads((output / 'samples.json').read_text())
    assert len(samples) == 1, 'The interactive capture path must persist exactly one accepted sample.'
    sample = samples[0]
    np.testing.assert_allclose(sample['base_T_end'], fixture.expected_robot, atol=1e-8)
    actual = np.asarray(sample['camera_T_marker'])
    assert np.linalg.norm(actual[:3, 3] - fixture.expected_marker[:3, 3]) < 0.002
    angle = Rotation.from_matrix(actual[:3, :3].T @ fixture.expected_marker[:3, :3]).magnitude()
    assert angle < np.deg2rad(1.0)
    source = sample['source']
    assert source['robot_pose_topic'] == '/piper_servo/feedback/tcp_pose'
    assert source['marker'] == source['image'] == source['camera_info']
    assert source['image']['frame_id'] == camera_frame
    exposure_ns = source['image']['sec'] * 1_000_000_000 + source['image']['nanosec']
    assert exposure_ns in fixture.published_image_stamps
    assert source['robot_bracket'], 'The source robot interpolation bracket must be persisted.'
    for item in source['robot_bracket']:
        header = item['header']
        assert header['frame_id'] == base_frame
        assert header['sec'] * 1_000_000_000 + header['nanosec'] in fixture.published_robot_stamps
    info = sample['camera_info']
    assert (info['width'], info['height'], info['distortion_model']) == (640, 480, 'plumb_bob')
    np.testing.assert_allclose(info['k'], fixture.intrinsics.ravel())
    np.testing.assert_allclose(info['d'], fixture.distortion)
    saved_image = cv2.imread(str(output / sample['image_path']))
    assert saved_image is not None
    np.testing.assert_array_equal(saved_image, fixture.image_array)
    assert sample['stationarity']['duration_s'] == 1.0
    assert len(source['robot_history']) > 50


def run():
    token = uuid.uuid4().hex[:12]
    namespace = '/handeye_synthetic_test_' + token
    camera_frame, base_frame = token + '_color_optical_frame', token + '_base'
    # Keep the project/hardware domain (43) outside the synthetic test's range.
    domain_id = 60 + int(token[:4], 16) % 36
    with tempfile.TemporaryDirectory(prefix='handeye-ros-integration-') as temporary:
        output = Path(temporary)
        previous_log_dir = os.environ.get('ROS_LOG_DIR')
        os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
        nodes, executor, worker = [], None, None
        worker_errors = []
        previous_alarm_handler = signal.getsignal(signal.SIGALRM)
        def timeout(signum, frame):
            raise TimeoutError('Synthetic hand-eye integration exceeded its 45-second test limit.')
        signal.signal(signal.SIGALRM, timeout)
        signal.alarm(45)
        try:
            rclpy.init(args=['--ros-args', '-p', 'mode:=eye_in_hand', '-p', 'min_num:=20',
                              '-p', 'agx_arm_topic:=auto',
                              '-p', f'marker_topic:={namespace}/marker_pose',
                              '-p', f'result_save_path:={output}'], domain_id=domain_id)
            # Reproduce the real gamepad namespace mismatch inside this isolated
            # domain. No fixture publishes the old /feedback/tcp_pose topic.
            fixture = SyntheticCameraAndRobot(namespace, camera_frame, base_frame,
                                              '/piper_servo/feedback/tcp_pose')
            nodes.append(fixture)
            detector = ChessboardDetector(namespace + '/color', camera_frame, namespace + '/marker_pose',
                                          namespace + '/preview', (8, 11), 0.015)
            nodes.append(detector)
            collector = CheckedCalibration(output, camera_frame,
                                           capture_config={'base_frame': base_frame},
                                           camera_topic=namespace + '/color')
            nodes.append(collector)
            executor = SingleThreadedExecutor()
            executor.add_node(fixture)
            executor.add_node(detector)

            def spin_fixture():
                try:
                    executor.spin()
                except Exception as error:
                    worker_errors.append(error)

            worker = threading.Thread(target=spin_fixture, name='handeye-synthetic-fixture', daemon=True)
            worker.start()
            discovery_started = time.monotonic()
            fixture.start_robot_feedback_after(4.)
            collector.agx_arm_topic = resolve_robot_pose_topic(collector, 'auto')
            assert collector.agx_arm_topic == '/piper_servo/feedback/tcp_pose'
            assert fixture.robot_publisher_created_s is not None
            assert fixture.robot_publisher_created_s - discovery_started >= 4.
            assert time.monotonic() - discovery_started >= 4., 'Discovery must keep waiting beyond the initial 2-second window.'
            # Drive the production Enter/c/save path, including subscribe_message().
            with patch('builtins.input', side_effect=['', 'c']):
                try:
                    collector.get_poses()
                except SystemExit as error:
                    assert error.code in (0, None)
            assert not worker_errors, worker_errors
            verify_sample(output, fixture, camera_frame, base_frame)
            print(f'PASS: synthetic ROS domain {domain_id}; Image+CameraInfo → real chessboard detector '
                  '→ await TCP publisher delayed 4 seconds → auto-select gamepad TCP topic '
                  '→ stationary capture → samples.json/PNG/K/D/source stamps.')
        finally:
            signal.alarm(0)
            signal.signal(signal.SIGALRM, previous_alarm_handler)
            if executor is not None:
                executor.shutdown(timeout_sec=3.0)
            if worker is not None:
                worker.join(timeout=3.0)
                if worker.is_alive():
                    raise RuntimeError('The synthetic fixture executor did not stop.')
            for node in reversed(nodes):
                node.destroy_node()
            rclpy.try_shutdown()
            if previous_log_dir is None:
                os.environ.pop('ROS_LOG_DIR', None)
            else:
                os.environ['ROS_LOG_DIR'] = previous_log_dir


if __name__ == '__main__':
    run()
