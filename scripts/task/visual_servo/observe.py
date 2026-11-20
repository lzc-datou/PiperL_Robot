"""Passive preview. No driver, motion clients or command publishers."""
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scripts.tools.handeye_calibration.feedback import ROBOT_POSE_TOPICS, select_robot_pose_topic
from .geometry import calibration, TargetGeometry, POSE_TOPIC
from .observation import Observations, read_pose, stamp_seconds


class Preview(Node):
    def __init__(self, cfg, directory):
        super().__init__('visual_servo_observer')
        self.geometry = None
        try:
            ec, _, _ = calibration(cfg, directory)
            self.geometry = TargetGeometry(cfg, ec)
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f'{exc}；仅显示相机系位置')
        self.cfg = cfg
        self.inputs = Observations(cfg['frame'], cfg)
        self.message = '等待棋盘观测'
        self.stamp = 0.
        self.create_subscription(PoseStamped, POSE_TOPIC, self.board, 1)
        self.tcp_topic = self.tcp_sub = None
        self.create_timer(.5, self.select_feedback)
        self.create_timer(.3, self.display)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def display(self):
        stale = self.now()-self.stamp > self.cfg['max_age_s']
        self.get_logger().info(('等待新鲜观测 | ' if stale else '')+self.message)

    def select_feedback(self):
        sources = {topic: self.get_publishers_info_by_topic(topic) for topic in ROBOT_POSE_TOPICS}
        try:
            topic = select_robot_pose_topic('auto', sources)
        except ValueError:
            topic = None
        if topic != self.tcp_topic:
            if self.tcp_sub is not None:
                self.destroy_subscription(self.tcp_sub)
            self.inputs.feedback.clear()
            self.tcp_topic = topic
            self.tcp_sub = self.create_subscription(PoseStamped, topic, self.feedback, 10) if topic else None

    def feedback(self, msg):
        if msg.header.frame_id != 'base_link':
            return
        stamp = stamp_seconds(msg)
        if not self.inputs.feedback or self.inputs.feedback[-1][0] < stamp:
            try:
                self.inputs.feedback.append((stamp, read_pose(msg.pose)))
                self.base_preview()
            except ValueError:
                pass

    def base_preview(self):
        if self.geometry and self.inputs.pending:
            result = self.inputs.match(self.now(), time.monotonic())
            if result:
                _, _, be, target, _ = result
                goal = self.geometry.measure(be, target)
                self.message += ' | 基座系候选 TCP ' + np.array2string(goal.pose, precision=3)

    def board(self, msg):
        try:
            if self.count_publishers(POSE_TOPIC) != 1:
                raise ValueError('棋盘位姿来源冲突')
            self.inputs.receive(msg, self.now(), time.monotonic())
            self.stamp, _, target = self.inputs.pending
            self.message = f'相机系棋盘中心 {np.round(target.center, 3)} m'
            if self.geometry:
                # Angle is invariant to the choice of base; do not latch a
                # camera-only orientation into the base-frame tracking geometry.
                local = TargetGeometry(self.cfg, self.geometry.ec)
                local.sign = self.geometry.sign
                goal = local.measure(np.eye(4), target)
                self.geometry.sign = local.sign
                center_tcp = local.ec[:3, :3] @ target.center + local.ec[:3, 3]
                self.message += f' | TCP 距离 {np.linalg.norm(center_tcp)*1000:.1f} mm，横偏 {np.linalg.norm(center_tcp[:2])*1000:.1f} mm'
                self.message += f' | 偏差 {goal.alpha:.2f}° | '+('纠偏' if goal.correcting else '只跟位置')
                self.base_preview()
        except ValueError as exc:
            self.message = str(exc)


def run(cfg, directory):
    rclpy.init(args=[])
    node = Preview(cfg, directory)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
