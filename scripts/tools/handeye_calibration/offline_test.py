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
from chessboard import board_settings, object_grid, ordered_corners, solve_pose


def check_chessboard():
    k = np.array([[620., 0, 320], [0, 620., 240], [0, 0, 1]])
    d = np.zeros(5)
    for columns, rows in ((8, 11), (9, 12)):
        pattern, size = board_settings({'columns': columns, 'rows': rows, 'square_size_mm': 15})
        points = object_grid(pattern, size)
        assert points.shape == (columns * rows, 3)
        np.testing.assert_allclose(points[-1], [(columns - 1) * 0.015, (rows - 1) * 0.015, 0])
        center = np.array([(columns - 1) / 2, (rows - 1) / 2, 0]) * size
        assert ordered_corners(np.full((480, 640), 255, np.uint8), pattern) is None
        for pitch in (0, 30, 50):
            for roll in (0, 75, 150):
                expected = Rotation.from_euler('xyz', [pitch, 10, roll], degrees=True)
                rvec = expected.as_rotvec()
                tvec = np.array([0., 0., 0.43]) - expected.as_matrix() @ center
                image = np.full((480, 640), 255, np.uint8)
                for y in range(-1, rows):
                    for x in range(-1, columns):
                        if (x + y) % 2:
                            continue
                        cell = np.array([[x, y, 0], [x + 1, y, 0],
                                         [x + 1, y + 1, 0], [x, y + 1, 0]], float) * size
                        pixels, _ = cv2.projectPoints(cell, rvec, tvec, k, d)
                        cv2.fillConvexPoly(image, np.rint(pixels.reshape(4, 2)).astype(np.int32), 0)
                corners = ordered_corners(image, pattern)
                assert corners is not None and len(corners) == columns * rows, (pattern, pitch, roll)
                result = solve_pose(points, corners, k, d)
                assert result is not None, (pattern, pitch, roll)
                rotation, translation, rms = result
                # Integer-pixel rendering introduces quantization; test the physical
                # scale and frame direction without claiming submillimeter accuracy.
                assert np.linalg.norm(translation - tvec) < 0.002, (pattern, pitch, roll)
                assert (Rotation.from_matrix(rotation).inv() * expected).magnitude() < np.deg2rad(1.0)
                assert rms < 1.0
                turned = ordered_corners(cv2.rotate(image, cv2.ROTATE_180), pattern)
                assert turned is not None
                np.testing.assert_allclose(
                    turned + corners, np.broadcast_to([639., 479.], corners.shape), atol=0.8)
                corrupted = corners.copy()
                corrupted[20, 0] += 40
                assert solve_pose(points, corrupted, k, d) is None
    print('PASS: 8×11/9×12 棋盘格、多轴姿态、毫米尺度、位姿方向、180° 原点、空白图与异常重投影拒绝。')


def make_pose(transform):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, transform[:3, 3])
    q = Rotation.from_matrix(transform[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return PoseMartix(p)


def run():
    from tests_quality import run as check_quality
    from tests_sampling import run as check_sampling
    from tests_feedback import run as check_feedback
    from tests_combined import run as check_combined
    check_feedback()
    check_quality()
    check_sampling()
    check_combined()
    check_chessboard()
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
