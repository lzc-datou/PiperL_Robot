"""Fixed-axis RPY conversion and Cartesian pose error in task units."""
import math
import warnings


def quaternion_from_rpy_deg(rpy):
    from scipy.spatial.transform import Rotation

    # Extrinsic xyz: Rz(yaw) @ Ry(pitch) @ Rx(roll), in ROS xyzw order.
    return Rotation.from_euler('xyz', rpy, degrees=True).as_quat().tolist()


def rpy_deg_from_quaternion(xyzw):
    from scipy.spatial.transform import Rotation

    # At gimbal lock SciPy returns an equivalent rotation with yaw = 0.
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', message='Gimbal lock detected', category=UserWarning)
        return Rotation.from_quat(xyzw).as_euler('xyz', degrees=True).tolist()


def orientation_error_rad(first, second):
    """Shortest rotation angle between two unit quaternions."""
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return 2 * math.acos(min(1.0, dot))


def pose_errors(actual, target):
    distance = math.dist(actual['position_m'], target['position_m'])
    return distance, orientation_error_rad(actual['orientation_xyzw'], target['orientation_xyzw'])
