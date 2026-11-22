"""Repeatable single-process FK/Cartesian controller closed loop; no ROS transport."""
import json
from pathlib import Path
from types import SimpleNamespace

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from scripts.task.servo_tracking.cartesian_control import CartesianController
from scripts.task.servo_tracking.core import RobotModel, damped_joint_velocity, load_config
from scripts.task.visual_servo.geometry import TargetGeometry, calibration, chessboard_target, matrix, settings, stabilize_goal, SIM_FRAME
from scripts.task.visual_servo.observation import read_pose
from scripts.task.visual_servo.scene import BoardScene, K, render_board
from scripts.tools.handeye_calibration.chessboard import BoardSearch, ChessboardDetector, object_grid

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


def scenario(offset, tilt, roll=0., images=False):
    cfg = settings()
    servo = load_config(cfg['servo_config'])
    control = dict(servo['control'], linear_speed=cfg['linear_speed_mps'],
                   angular_speed=np.deg2rad(cfg['angular_speed_deg_s']))
    controller = CartesianController(control)
    model = RobotModel(URDF.read_text(), tip='link6')
    q = np.array(servo['initial_joints'])
    initial = matrix(model.pose(q))
    ec, _, _ = calibration(cfg, None, sim=True)
    scene = BoardScene(cfg, ec, initial)
    tracker = TargetGeometry(cfg, ec)
    board = np.eye(4)
    board[:3, :3] = (Rotation.from_euler('x', tilt, degrees=True) *
                     Rotation.from_euler('z', roll, degrees=True)).as_matrix()
    board[:3, 3] = offset
    actual_center = scene.origin + initial[:3, :3] @ board[:3, 3]
    actual_normal = initial[:3, :3] @ board[:3, 2]
    poses = []
    detector = SimpleNamespace(frame=SIM_FRAME, pattern=cfg['pattern'], bridge=CvBridge(), search=BoardSearch(tracking=True),
        points=object_grid(cfg['pattern'], cfg['size']), poses=SimpleNamespace(publish=poses.append),
        projection=None, preview=SimpleNamespace(get_subscription_count=lambda: 0),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)))
    info = CameraInfo(width=640, height=480, k=K.ravel().tolist(), d=[0.]*5, distortion_model='plumb_bob')
    info.header.frame_id = SIM_FRAME
    settled, first_reentry, verified, frames = [], None, 0, 0
    held, maximum_hold_drift, simultaneous = None, 0., False
    command = np.zeros(6)
    goal = None
    for step in range(600):
        t = (step+1)*.01
        actual = model.pose(q)
        be = matrix(actual)
        if step % 4 == 0:
            frames += 1
            cg = scene.camera_board(be, board)
            if images:
                msg = detector.bridge.cv2_to_imgmsg(render_board(cg, cfg), encoding='bgr8')
                msg.header.frame_id = SIM_FRAME
                msg.header.stamp.sec = 100 + step//100
                msg.header.stamp.nanosec = (step%100)*10000000
                info.header.stamp = msg.header.stamp
                ChessboardDetector.detect(detector, msg, info)
                assert poses and poses[-1].header == msg.header, f'image lost at t={t:.2f}'
                cg = read_pose(poses.pop().pose)
            raw = tracker.measure(be, chessboard_target(cg, cfg))
            goal = stabilize_goal(raw, goal, cfg)
            verified += 1
            target_time = t
            if goal.switched:
                controller.reset_angular_reference(goal.pose[3:])
            if not goal.correcting:
                if first_reentry is None:
                    first_reentry = t
                if held is None:
                    held = goal.pose[3:].copy()
                maximum_hold_drift = max(maximum_hold_drift, float(
                    (Rotation.from_quat(held).inv()*Rotation.from_quat(goal.pose[3:])).magnitude()))
            else:
                held = None
        command = controller.update(goal.pose, actual, now=t, target_time=target_time,
                                    speed_scale=.5, actual_time=t)
        simultaneous |= bool(goal.correcting and np.linalg.norm(command[:3]) > .001 and
                             np.linalg.norm(command[3:]) > .001)
        q += .01*damped_joint_velocity(q, command, model, servo, scale=.5)
        model.check_joints(q, margin=servo['joint_margin'])
        if t >= 4:
            observed = be[:3, :3].T @ (actual_center - be[:3, 3])
            alpha = np.rad2deg(np.arccos(np.clip(be[:3, 2] @ actual_normal, -1., 1.)))
            settled.append([abs(np.linalg.norm(observed)-cfg['distance_m'])*1000,
                            np.linalg.norm(observed[:2])*1000, alpha])
    p95 = np.percentile(settled, 95, axis=0)
    assert p95[0] <= 10 and p95[1] <= 5, p95
    assert maximum_hold_drift < 1e-8
    if tilt > 5:
        assert simultaneous and first_reentry is not None and p95[2] <= 5.1, (first_reentry, p95)
    return dict(step_m=offset, tilt_deg=tilt, roll_deg=roll, image_input=images,
                distance_p95_mm=p95[0], lateral_p95_mm=p95[1], alpha_p95_deg=p95[2],
                first_in_band_s=first_reentry, valid_frames=verified, total_frames=frames,
                hold_reference_drift_rad=maximum_hold_drift)


def main():
    cv2.setNumThreads(2)
    results = [scenario((np.eye(3)[axis]*sign*.03).tolist(), 3) for axis in range(3)
               for sign in (-1, 1)]
    results += [scenario([0., 0., 0.], 3, roll) for roll in (-30, 30)]
    results.append(scenario([.018, .008, .005], 8))
    results.append(scenario([.018, .008, .005], 8, images=True))
    out = ROOT/'outputs/tasks/visual_servo/kinematics.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=2))
    for r in results:
        print(f"{r['step_m']} tilt={r['tilt_deg']}° image={r['image_input']} "
              f"P95 {r['distance_p95_mm']:.2f} mm, {r['lateral_p95_mm']:.2f} mm, "
              f"{r['alpha_p95_deg']:.2f}°, first in band {r['first_in_band_s']:.2f} s")


if __name__ == '__main__':
    main()
