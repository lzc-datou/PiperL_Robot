"""Replay ROS messages through the real sampler without a node, camera or CAN."""
import inspect
import json
import tempfile
from collections import OrderedDict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from rclpy.executors import ExternalShutdownException

from collector import CheckedCalibration, DEFAULT_CAPTURE, matrix, pose_matrix, stamp_ns, run as run_calibration


FRAME = 'handeye_d405_color_optical_frame'


def stamped(message, seconds, frame):
    nanoseconds = round(seconds * 1e9)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
    message.header.frame_id = frame
    return message


def robot_message(seconds, x=0.1, frame=''):
    message = stamped(PoseStamped(), seconds, frame)
    message.pose.orientation.w = 1.0
    message.pose.position.x = x
    return message


def camera_messages(seconds):
    marker = stamped(PoseStamped(), seconds, FRAME)
    marker.pose.position.z = 0.3
    marker.pose.orientation.w = 1.0
    image = stamped(CvBridge().cv2_to_imgmsg(np.full((24, 32, 3), 127, np.uint8), 'bgr8'), seconds, FRAME)
    info = stamped(CameraInfo(), seconds, FRAME)
    info.width, info.height = 32, 24
    info.k = [100.0, 0.0, 16.0, 0.0, 100.0, 12.0, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    info.distortion_model = 'plumb_bob'
    return marker, image, info


def sampler(output):
    node = object.__new__(CheckedCalibration)
    node.output, node.camera_frame, node.base_frame = Path(output), FRAME, 'base_link'
    node.camera_topic = '/handeye/handeye_d405/color'
    node.agx_arm_topic, node.marker_topic = '/feedback/tcp_pose', '/handeye/chessboard/pose'
    node.capture = dict(DEFAULT_CAPTURE)
    node.bridge = CvBridge()
    node.accepted_samples, node.agx_arm_poses, node.marker_poses = [], [], []
    node.used_frame_stamps, node._pending_sample = set(), None
    node.robots, node.markers, node.images, node.infos = (OrderedDict() for _ in range(4))
    node.robot_stamp_revisions = OrderedDict()
    node.mode, node.min_num = 'eye_in_hand', 20
    return node


def ready_candidate(node, moving=False):
    marker, image, info = camera_messages(101.105)
    for i in range(123):
        timestamp = 100.0 + i * 0.01
        robot = robot_message(timestamp, x=0.1 + i * (0.0001 if moving else 0.000001))
        node._cache_robot(robot)
    for message, cache in [(marker, node.markers), (image, node.images), (info, node.infos)]:
        node._cache(cache, message, 64)
    return marker


def reject(call, text):
    try:
        call()
    except (ValueError, RuntimeError) as error:
        assert text in str(error), str(error)
    else:
        raise AssertionError('Expected rejection: ' + text)


def check_candidates():
    with tempfile.TemporaryDirectory(prefix='handeye-candidates-') as directory:
        node = sampler(directory)
        marker = ready_candidate(node)
        _, robot = node._sample_candidate(marker, 100.0, 101.15)
        assert abs(robot.pose.position.x - 0.1001105) < 1e-12
        metadata = node._pending_sample
        assert metadata['interpolation']['bracket_gap_s'] < 0.011
        assert metadata['source']['marker']['nanosec'] == 105_000_000
        assert metadata['source']['robot_empty_frame_assumed_as'] == 'base_link'
        assert len(metadata['source']['robot_bracket']) == 2
        assert metadata['stationarity']['duration_s'] == 1.0
        assert metadata['camera_info']['k'][0] == 100.0
        png = cv2.imread(str(Path(directory) / metadata['image_path']))
        np.testing.assert_array_equal(png, np.full((24, 32, 3), 127, np.uint8))
        node.used_frame_stamps.add(stamp_ns(marker))
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '已经采过')
        node.used_frame_stamps.clear()
        reject(lambda: node._sample_candidate(marker, 100.0, 102.0), '过期')
        reject(lambda: node._sample_candidate(marker, 100.5, 101.15), '完整静止窗口')
        marker.header.frame_id = 'wrong_camera'
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '相机坐标系')
        marker.header.frame_id = FRAME
        info = node.infos[stamp_ns(marker)]
        info.header.frame_id = 'wrong_camera'
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), 'CameraInfo')
        info.header.frame_id = FRAME
        info.width += 1
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '尺寸')
        info.width -= 1
        node.images.clear()
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '同一曝光时间')
        node = sampler(directory)
        marker = ready_candidate(node, moving=True)
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '尚未停稳')
        node = sampler(directory)
        marker = ready_candidate(node)
        first_robot = next(iter(node.robots.values()))
        first_robot.header.frame_id = 'wrong_base'
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '机器人反馈 frame')
        node = sampler(directory)
        marker = ready_candidate(node)
        node.robots = OrderedDict((key, value) for key, value in node.robots.items() if key < stamp_ns(marker))
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '完整 1 秒')
        # Re-publishing an old source stamp does not build a stationary window.
        node = sampler(directory)
        marker, image, info = camera_messages(101.105)
        for _ in range(200):
            node._cache_robot(robot_message(101.0))
        for message, cache in [(marker, node.markers), (image, node.images), (info, node.infos)]:
            node._cache(cache, message, 64)
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '完整 1 秒')
        changed = robot_message(101.0, x=0.5)
        reject(lambda: node._cache_robot(changed), '超过停稳阈值')
    print('PASS: 同曝光帧配对、静止窗口、插值、源stamp/frame、内参和PNG归档及错误输入拒绝。')


def check_robot_stamp_revisions():
    with tempfile.TemporaryDirectory(prefix='handeye-sdk-stamp-') as directory:
        node = sampler(directory)
        marker = ready_candidate(node)
        timestamp = 101.10
        first = robot_message(timestamp, x=0.100110)
        node._cache_robot(first)
        jitter = robot_message(timestamp, x=0.100111)
        quaternion = Rotation.from_euler('z', 0.05, degrees=True).as_quat()
        jitter.pose.orientation.x, jitter.pose.orientation.y, jitter.pose.orientation.z, jitter.pose.orientation.w = map(float, quaternion)
        before_count = len(node.robots)
        node._cache_robot(jitter)
        assert len(node.robots) == before_count
        np.testing.assert_allclose(pose_matrix(node.robots[stamp_ns(jitter)]), pose_matrix(jitter))
        node._sample_candidate(marker, 100.0, 101.15)
        revisions = node._pending_sample['source']['robot_stamp_revisions']
        assert len(revisions) == 1
        revision = revisions[0]
        assert revision['revision_count'] == 1
        assert abs(revision['translation_from_first_max_mm'] - 0.001) < 1e-9
        assert abs(revision['rotation_from_first_max_deg'] - 0.05) < 1e-9
        assert revision['header']['nanosec'] == 100_000_000
        # Group anchor stays at the FIRST value. Small successive changes cannot creep past the limit.
        node = sampler(directory)
        node._cache_robot(robot_message(101.0, x=0.1))
        node._cache_robot(robot_message(101.0, x=0.1006))
        reject(lambda: node._cache_robot(robot_message(101.0, x=0.1012)), '超过停稳阈值')
        node = sampler(directory)
        node._cache_robot(robot_message(101.0))
        for angle in (0.12, 0.24):
            message = robot_message(101.0)
            quaternion = Rotation.from_euler('z', angle, degrees=True).as_quat()
            message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = map(float, quaternion)
            if angle == 0.12:
                node._cache_robot(message)
            else:
                reject(lambda: node._cache_robot(message), '超过停稳阈值')
        reject(lambda: node._cache_robot(robot_message(101.0, frame='other_base')), 'frame')
        # Hundreds of same-stamp revisions still cannot provide a one-second window.
        node = sampler(directory)
        marker, image, info = camera_messages(101.105)
        for i in range(200):
            node._cache_robot(robot_message(101.0, x=0.1 + (i % 2) * 1e-6))
        assert len(node.robots) == 1
        for message, cache in [(marker, node.markers), (image, node.images), (info, node.infos)]:
            node._cache(cache, message, 64)
        reject(lambda: node._sample_candidate(marker, 100.0, 101.15), '完整 1 秒')
        changed_marker, _, _ = camera_messages(101.105)
        changed_marker.pose.position.x = 0.000001
        reject(lambda: node._cache(node.markers, changed_marker, 64), '不同位姿')
        # Bounded caches evict both representations of the same SDK stamp.
        node = sampler(directory)
        for i in range(5):
            node._cache_robot(robot_message(101.0 + i * 0.01), limit=3)
        assert len(node.robots) == len(node.robot_stamp_revisions) == 3
        assert set(node.robots) == set(node.robot_stamp_revisions)
    print('PASS: SDK同stamp微小更新只计一次、保存修订量、首值锚定防累积漂移、frame/marker冲突和缓存同步淘汰。')


def check_subscription_flow():
    with tempfile.TemporaryDirectory(prefix='handeye-subscriptions-') as directory:
        node = sampler(directory)
        callbacks, destroyed = {}, []
        state = {'tick': 0, 'now': 100.0}
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(state['now'] * 1e9)))

        def subscribe(message_type, topic, callback, qos):
            inspect.signature(callback).bind(object())
            callbacks[topic] = callback
            return topic

        def spin(unused_node, timeout_sec):
            state['tick'] += 1
            timestamp = 100 + state['tick'] * 0.01
            state['now'] = timestamp + 0.005
            callbacks[node.agx_arm_topic](robot_message(timestamp, 0.1 + state['tick'] * 0.000001))
            if state['tick'] % 3 == 0:
                marker, image, info = camera_messages(timestamp - 0.005)
                # Deliberately deliver marker before image and CameraInfo.
                callbacks[node.marker_topic](marker)
                callbacks[node.camera_topic + '/image_raw'](image)
                callbacks[node.camera_topic + '/camera_info'](info)

        node.create_subscription, node.destroy_subscription = subscribe, destroyed.append
        with patch('collector.rclpy.ok', return_value=True), patch('collector.rclpy.spin_once', side_effect=spin):
            marker, robot = node.subscribe_message()
        assert state['tick'] >= 102
        assert len(destroyed) == 4 and len(callbacks) == 4
        assert marker.header.stamp == robot.header.stamp
        np.testing.assert_allclose(pose_matrix(robot)[:3, :3], np.eye(3))
        # A later subscription setup failure still destroys every earlier subscription.
        callbacks.clear()
        destroyed.clear()

        def fail_second(*args):
            if callbacks:
                raise RuntimeError('second subscription failed')
            return subscribe(*args)

        node.create_subscription = fail_second
        reject(node.subscribe_message, 'second subscription failed')
        assert len(destroyed) == 1
        assert 'second subscription failed' in (Path(directory) / 'sampling_error.txt').read_text()
    print('PASS: 真实采样循环接收四路乱序消息、等待停稳、精确配对、Jazzy回调签名和异常清理。')


def check_incremental_save_and_retry():
    with tempfile.TemporaryDirectory(prefix='handeye-incremental-') as directory:
        node = sampler(directory)
        marker = ready_candidate(node)
        marker, robot = node._sample_candidate(marker, 100.0, 101.15)
        pending = node._pending_sample
        operations = iter(['', '', 'd', '', 'c'])
        attempts = {'value': 0}

        def subscribe():
            attempts['value'] += 1
            if attempts['value'] == 1:
                raise RuntimeError('transient input timeout')
            node._pending_sample = dict(pending)
            return marker, robot

        def user_input(prompt):
            command = next(operations)
            path = Path(directory) / 'samples.json'
            if command == 'd':
                samples = json.loads(path.read_text())
                assert len(samples) == 1 and samples[0]['id'] == pending['id']
                assert 'source' in samples[0] and 'base_T_end' in samples[0]
            elif command == '' and attempts['value'] == 2:
                assert json.loads(path.read_text()) == []
            return command

        node.subscribe_message = subscribe
        with patch('collector.rclpy.ok', return_value=True), patch('builtins.input', side_effect=user_input):
            try:
                node.get_poses()
            except SystemExit as error:
                assert error.code == 0
        assert attempts['value'] == 3
        saved = json.loads((Path(directory) / 'samples.json').read_text())
        assert len(saved) == 1
        np.testing.assert_allclose(saved[0]['base_T_end'], matrix(node.agx_arm_poses[0]))
        assert not (Path(directory) / 'samples.json.tmp').exists()
        # Repeated stationary observations remain usable for diagnostics, but cannot be solved.
        node.agx_arm_poses *= 20
        node.marker_poses *= 20
        reject(node.process_handeye, '退化')
        assert not (Path(directory) / 'T_end_camera.json').exists()
    print('PASS: 超时只拒绝本次、交互继续、逐次原子保存、删除同步、c退出保留和退化求解拒绝。')


def check_timeout_diagnostics():
    for stale_robot in (None, robot_message(90.0)):
        with tempfile.TemporaryDirectory(prefix='handeye-input-diagnostics-') as directory:
            node = sampler(directory)
            node.agx_arm_topic = '/piper_servo/feedback/tcp_pose'
            callbacks, destroyed, state = {}, [], {'now': 100.0}
            node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(state['now'] * 1e9)))

            def subscribe(message_type, topic, callback, qos):
                callbacks[topic] = callback
                return topic

            def spin(unused_node, timeout_sec):
                state['now'] = 101.15
                if stale_robot is not None:
                    callbacks[node.agx_arm_topic](stale_robot)
                marker, image, info = camera_messages(101.105)
                callbacks[node.marker_topic](marker)
                callbacks[node.camera_topic + '/image_raw'](image)
                callbacks[node.camera_topic + '/camera_info'](info)

            node.create_subscription, node.destroy_subscription = subscribe, destroyed.append
            expected = node.agx_arm_topic if stale_robot is None else '反馈已到达'
            with patch('collector.rclpy.ok', return_value=True), \
                    patch('collector.time.monotonic', side_effect=[0.0, 0.0, 11.0]), \
                    patch('collector.rclpy.spin_once', side_effect=spin):
                reject(node.subscribe_message, expected)
            saved = json.loads((Path(directory) / 'input_diagnostics.json').read_text())
            assert saved['streams']['robot']['unique_messages'] == (0 if stale_robot is None else 1)
            assert saved['streams']['robot']['topic'] == node.agx_arm_topic
            assert saved['streams']['marker']['unique_messages'] == 1
            assert len(destroyed) == 4
    print('PASS: 超时诊断区分机器人话题无消息与源时间戳过期，保留四路消息统计和实际订阅名。')


def check_wait_cancellation_cleanup():
    cfg = {'min_samples': 20, 'robot_pose_topic': 'auto', 'marker_pose_topic': '/marker',
           'base_frame': 'base_link', 'camera_namespace': 'handeye', 'camera_name': 'handeye_d405'}
    for interruption in (KeyboardInterrupt(), ExternalShutdownException(), SystemExit(0)):
        with tempfile.TemporaryDirectory(prefix='handeye-cancel-') as directory:
            node = sampler(directory)
            destroyed = []
            node.destroy_node = lambda: destroyed.append(True)
            with patch('collector.rclpy.init'), patch('collector.rclpy.try_shutdown') as shutdown, \
                    patch('collector.CheckedCalibration', return_value=node), \
                    patch('collector.resolve_robot_pose_topic', side_effect=interruption):
                run_calibration(cfg, Path(directory), FRAME)
            assert destroyed == [True]
            shutdown.assert_called_once()
            assert json.loads((Path(directory) / 'samples.json').read_text()) == []
    print('PASS: 等待时 Ctrl+C/ROS关闭均正常保存并清理，不抛出退出堆栈。')


def run():
    check_candidates()
    check_robot_stamp_revisions()
    check_subscription_flow()
    check_incremental_save_and_retry()
    check_timeout_diagnostics()
    check_wait_cancellation_cleanup()


if __name__ == '__main__':
    run()
