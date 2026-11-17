"""Meaningful geometry/input checks with synthetic data, no ROS node or device."""
import tempfile
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation
from handeye_calibration_ros.agx_arm_calibration import AgxArmCalibrationNode, PoseMartix

from board import generate
from collector import CheckedCalibration, matrix, valid_pair


def make_pose(transform):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, transform[:3, 3])
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return PoseMartix(p)


def run():
    rng = np.random.default_rng(43)
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('xyz', [0.2, -0.3, 0.4]).as_matrix()
    expected[:3, 3] = [0.04, -0.02, 0.06]
    board = np.eye(4)
    board[:3, 3] = [0.45, 0.1, 0.2]
    robot, marker = [], []
    for _ in range(25):
        end = np.eye(4)
        end[:3, :3] = Rotation.from_euler('xyz', rng.uniform(-0.4, 0.4, size=3)).as_matrix()
        end[:3, 3] = rng.uniform(-0.2, 0.2, size=3)
        camera_marker = np.linalg.inv(expected) @ np.linalg.inv(end) @ board
        robot.append(make_pose(end))
        marker.append(make_pose(camera_marker))
    # Call the official solver, then the real project export/consistency path.
    fake = SimpleNamespace(agx_arm_poses=robot, marker_poses=marker)
    actual = matrix(AgxArmCalibrationNode.process_handeye(fake))
    np.testing.assert_allclose(actual, expected, atol=1e-8)
    with tempfile.TemporaryDirectory(prefix='piperl-handeye-test-') as temp:
        probe = object.__new__(CheckedCalibration)
        probe.agx_arm_poses, probe.marker_poses, probe.output = robot, marker, Path(temp)
        np.testing.assert_allclose(matrix(probe.process_handeye()), expected, atol=1e-8)
        assert (probe.output / 'samples.json').exists()
        generate(Path(temp))
        image = cv2.imread(str(Path(temp) / 'original_582_preview.png'))
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        _, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        assert ids is not None and ids.flatten().tolist() == [582]
    a, b = PoseStamped(), PoseStamped()
    for msg in (a, b):
        msg.header.stamp.sec = 100
        msg.pose.orientation.w = 1.0
    b.header.frame_id = 'handeye_d405_color_optical_frame'
    frame = b.header.frame_id
    assert valid_pair(a, b, 100.1, frame)
    assert not valid_pair(a, b, 101.0, frame)  # stale
    assert not valid_pair(a, b, 100.1, 'wrong_frame')
    b.header.stamp.nanosec = 200_000_000
    assert not valid_pair(a, b, 100.3, frame)  # excessive skew
    b.header.stamp.nanosec = 0
    b.pose.orientation.w = 0.0
    assert not valid_pair(a, b, 100.1, frame)  # invalid quaternion
    print('PASS: 官方眼在手上求解、矩阵方向、样本导出、一致性、标记 ID、过期/时间差/frame/四元数检查。')


if __name__ == '__main__':
    run()
