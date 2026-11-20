"""Independent board in base_link. The virtual camera follows measured mock joints."""
import json
import sys
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Header, String

from servo_tracking.core import JOINTS, RobotModel
from servo_tracking.node import pose_message
from scripts.tools.handeye_calibration.chessboard import ChessboardDetector, spin_detector
from .geometry import calibration, matrix, pose7, settings, SIM_FRAME, SIM_TOPIC
from .observation import read_pose

COLOR = '/piper_servo/sim/camera/color'
K = np.array([[620., 0., 320.], [0., 620., 240.], [0., 0., 1.]])


class BoardScene:
    def __init__(self, cfg, ec, initial_tcp):
        self.ec = ec
        self.tool = initial_tcp
        nx, ny = cfg['pattern']
        self.center = np.array([nx-1, ny-1, 0.])*cfg['size']/2
        self.origin = initial_tcp[:3, 3] + initial_tcp[:3, 2]*cfg['distance_m']

    def camera_board(self, measured_tcp, offset):
        # Translation and rotation are relative to the initial TCP axes.
        bh = self.tool @ offset
        bh[:3, 3] += self.origin-self.tool[:3, 3]
        bg = bh.copy()
        bg[:3, 3] -= bg[:3, :3] @ self.center
        return np.linalg.solve(measured_tcp @ self.ec, bg)


def demo_offset(t):
    transform = np.eye(4)
    transform[:3, 3] = [.018*np.sin(t*.36), .012*np.sin(t*.4), .008*np.sin(t*.22)]
    transform[:3, :3] = Rotation.from_euler('xz', [3+5*np.sin(t*.35), 22*np.sin(t*.3)],
                                            degrees=True).as_matrix()
    return transform


def render_board(cg, cfg):
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    if cg[2, 3] <= 0:
        return image
    r = Rotation.from_matrix(cg[:3, :3]).as_rotvec()
    for y in range(-1, cfg['pattern'][1]):
        for x in range(-1, cfg['pattern'][0]):
            if (x+y) % 2:
                continue
            square = (np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]])
                      + [x, y, 0.])*cfg['size']
            pixels, _ = cv2.projectPoints(square, r, cg[:3, 3], K, None)
            cv2.fillConvexPoly(image, np.rint(pixels[:, 0, :]*16).astype(np.int32),
                               (0, 0, 0), lineType=cv2.LINE_AA, shift=4)
    return image


class Fixture(Node):
    def __init__(self):
        super().__init__('sim_board')
        self.cfg = settings(self.declare_parameter('visual_config', '').value)
        self.model = RobotModel(self.declare_parameter('robot_description', '').value)
        self.ec, _, _ = calibration(self.cfg, None, sim=True)
        self.images = self.declare_parameter('sim_input', 'pose').value == 'image'
        self.scene = self.feedback = self.manual = self.started = None
        self.visible = True
        self.last_stamp = None
        self.create_subscription(JointState, 'commanded_joint_states', self.joints, 1)
        self.create_subscription(String, 'status', self.status, 1)
        self.create_subscription(Pose, 'sim/board_offset', self.move, 1)
        self.create_subscription(Bool, 'sim/visible', self.visibility, 1)
        if self.images:
            self.bridge = CvBridge()
            self.image = self.create_publisher(Image, COLOR+'/image_raw', qos_profile_sensor_data)
            self.info = self.create_publisher(CameraInfo, COLOR+'/camera_info', qos_profile_sensor_data)
        else:
            self.pose = self.create_publisher(PoseStamped, SIM_TOPIC, 1)
            self.misses = self.create_publisher(Header, SIM_TOPIC + "/not_visible", 1)
        self.create_timer(1/25, self.expose)

    def joints(self, msg):
        try:
            q = [msg.position[msg.name.index(name)] for name in JOINTS]
            self.feedback = (msg.header.stamp, matrix(self.model.pose(q)))
        except (ValueError, IndexError):
            return
        if self.scene is None:
            self.scene = BoardScene(self.cfg, self.ec, self.feedback[1])

    def status(self, msg):
        if json.loads(msg.data)['active'] and self.started is None:
            self.started = time.monotonic()

    def move(self, msg):
        try:
            self.manual = read_pose(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def visibility(self, msg):
        self.visible = msg.data

    def expose(self):
        if self.feedback is None or self.feedback[0] == self.last_stamp:
            return
        stamp, be = self.feedback
        self.last_stamp = stamp
        t = time.monotonic()-self.started if self.started else 0.
        cg = self.scene.camera_board(be, demo_offset(t) if self.manual is None else self.manual)
        if self.images:
            bgr = render_board(cg, self.cfg) if self.visible else np.full((480, 640, 3), 255, np.uint8)
            msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            msg.header.stamp, msg.header.frame_id = stamp, SIM_FRAME
            info = CameraInfo(header=msg.header, height=480, width=640,
                distortion_model='plumb_bob', d=[0.]*5, k=K.ravel().tolist())
            self.info.publish(info)
            self.image.publish(msg)
        elif self.visible:
            msg = PoseStamped(pose=pose_message(pose7(cg)))
            msg.header.stamp, msg.header.frame_id = stamp, SIM_FRAME
            self.pose.publish(msg)
        else:
            self.misses.publish(Header(stamp=stamp, frame_id=SIM_FRAME))


def main():
    rclpy.init()
    if sys.argv[1:2] == ['detect']:
        cfg = settings(sys.argv[2])
        cv2.setNumThreads(2)
        node = ChessboardDetector(COLOR, SIM_FRAME, SIM_TOPIC, '/piper_servo/sim/result',
                                  cfg['pattern'], cfg['size'], tracking=True)
    else:
        node = Fixture()
    try:
        if isinstance(node, ChessboardDetector):
            spin_detector(node)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
