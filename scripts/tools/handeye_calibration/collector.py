"""Use AgileX's solver/UI; check live pose pairs and export accepted sample data."""
import json
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from handeye_calibration_ros.agx_arm_calibration import AgxArmCalibrationNode


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def valid_pair(robot, marker, now, camera_frame):
    for msg in (robot, marker):
        age = now - stamp(msg)
        p, q = msg.pose.position, msg.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 0.01:
            return False
        if not 0 <= age <= 0.5:
            return False
    return abs(stamp(robot) - stamp(marker)) <= 0.10 and marker.header.frame_id == camera_frame


class CheckedCalibration(AgxArmCalibrationNode):
    def __init__(self, output, camera_frame):
        super().__init__()
        self.output = output
        self.camera_frame = camera_frame

    def subscribe_message(self):
        # Original get_poses() calls this only when the operator presses Enter.
        # Receive both inputs concurrently instead of sequential one-shot subscriptions.
        latest = {}
        subscriptions = [self.create_subscription(PoseStamped, topic,
                         lambda msg, key=key: latest.__setitem__(key, msg), qos_profile_sensor_data)
                         for key, topic in [('robot', self.agx_arm_topic), ('marker', self.marker_topic)]]
        deadline = time.monotonic() + 5.0
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if set(latest) == {'robot', 'marker'}:
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if valid_pair(latest['robot'], latest['marker'], now, self.camera_frame):
                        return latest['marker'], latest['robot']
            message = '5 秒内未收到新鲜、时间差 ≤100 ms 且相机 frame 正确的位姿对；请检查反馈/标记。'
            print(message, flush=True)
            raise RuntimeError(message)
        finally:
            for sub in subscriptions:
                self.destroy_subscription(sub)

    def process_handeye(self):
        # Upstream maintains these lists, including removals via 'd'.
        samples = [{'base_T_end': matrix(a).tolist(), 'camera_T_marker': matrix(b).tolist()}
                   for a, b in zip(self.agx_arm_poses, self.marker_poses)]
        (self.output / 'samples.json').write_text(json.dumps(samples, indent=2) + '\n')
        result = super().process_handeye()
        transform = matrix(result)
        if not np.isfinite(transform).all():
            raise RuntimeError('标定结果不是有限数值，请重新采集包含多轴旋转的姿态。')
        (self.output / 'T_end_camera.json').write_text(json.dumps(transform.tolist(), indent=2) + '\n')
        boards = np.array([matrix(a) @ transform @ matrix(b)
                           for a, b in zip(self.agx_arm_poses, self.marker_poses)])
        spread_mm = np.linalg.norm(boards[:, :3, 3] - np.mean(boards[:, :3, 3], axis=0), axis=1) * 1000
        rotations = Rotation.from_matrix(boards[:, :3, :3])
        spread_deg = np.degrees((rotations.mean().inv() * rotations).magnitude())
        report = {'samples': len(samples), 'translation_spread_max_mm': float(max(spread_mm)),
                  'translation_spread_rms_mm': float(np.sqrt(np.mean(spread_mm ** 2))),
                  'rotation_spread_max_deg': float(max(spread_deg)),
                  'note': 'Training-sample consistency only; not independent accuracy validation.'}
        (self.output / 'consistency.json').write_text(json.dumps(report, indent=2) + '\n')
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
    node = CheckedCalibration(output, camera_frame)
    try:
        node.run()
    except (Exception, SystemExit) as error:
        print(f'采样/求解结束：{error}；若未生成结果文件，请检查输入后重新采集。', flush=True)
        raise
    finally:
        # Preserve upstream-accepted samples even if an input error aborts the session.
        samples = [{'base_T_end': matrix(a).tolist(), 'camera_T_marker': matrix(b).tolist()}
                   for a, b in zip(node.agx_arm_poses, node.marker_poses)]
        (output / 'samples.json').write_text(json.dumps(samples, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
