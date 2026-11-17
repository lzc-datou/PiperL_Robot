"""Reuse official planning APIs without constructing an execution/control client."""
import threading
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from scripts.task.common.moveit_client import MoveItClient
from scripts.task.common.planning_scene import PlanningScene
from scripts.task.common.ros_context import RosContext, robot_state
from scripts.task.common.schema import JOINTS

PREFIX = '/obstacle_avoidance'


class PlanningClient(RosContext):
    planner_id = 'ObstacleDemo'
    connect_model = MoveItClient.connect_model
    check_joints = MoveItClient.check_joints
    check_state = MoveItClient.check_state
    plan = MoveItClient.plan
    publish_plan = MoveItClient.publish_plan

    def __init__(self, settings, start):
        super().__init__('obstacle_preview')
        self.settings = settings
        self.cancel_requested = threading.Event()
        self.limits = {}
        self.start = tuple(start)
        self.planning = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.validity = self.node.create_client(GetStateValidity, '/check_state_validity')
        self.scene = PlanningScene(self, prefix='obstacle_avoidance/')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display = self.node.create_publisher(DisplayTrajectory, PREFIX+'/display_planned_path', qos)
        self.markers = self.node.create_publisher(MarkerArray, PREFIX+'/markers', qos)
        self.joints = self.node.create_publisher(JointState, PREFIX+'/demo_joint_states', 10)
        self.timer = self.node.create_timer(.05, self.publish_start)
        self.preview_message = None

    def tick(self, *_):
        if self.cancel_requested.is_set():
            raise InterruptedError('演示已取消')

    def publish_start(self):
        msg = JointState(name=list(JOINTS), position=list(self.start))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.joints.publish(msg)

    def state_validity(self, q):
        self.check_joints(q)
        request = GetStateValidity.Request(group_name=self.settings['group'], robot_state=robot_state(q))
        return self.call(self.validity, request, tick=self.tick)

    def clear_preview(self):
        self.preview_message = None
        self.display.publish(DisplayTrajectory())
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def show(self, trajectory, path, endpoints, baseline=False):
        self.publish_plan(self.start, [trajectory] if hasattr(trajectory, "joint_trajectory") else trajectory)
        array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        def marker(kind, identifier):
            msg = Marker(type=kind, action=Marker.ADD, ns='obstacle_demo', id=identifier)
            msg.header.frame_id = self.settings['frame_id']
            msg.pose.orientation.w = 1.
            msg.color.a = 1.
            return msg
        line = marker(Marker.LINE_STRIP, 0)
        line.scale.x = .004
        line.color.r, line.color.g, line.color.b = (1., .15, .1) if baseline else (.1, .85, .35)
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in path]
        array.markers.append(line)
        names = getattr(self, 'waypoint_names', [chr(65+i) for i in range(len(endpoints))])
        for index, (name, xyz) in enumerate(zip(names, endpoints)):
            ball = marker(Marker.SPHERE, 2*index+1)
            ball.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
            ball.scale.x = ball.scale.y = ball.scale.z = .025
            ball.color.r, ball.color.g, ball.color.b = (0.2, .6, 1.) if index == 0 else (1., .8, .1)
            array.markers.append(ball)
            label = marker(Marker.TEXT_VIEW_FACING, 2*index+2)
            label.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]+.04))
            label.scale.z = .035
            label.text = name
            label.color.r = label.color.g = label.color.b = 1.
            array.markers.append(label)
        label = marker(Marker.TEXT_VIEW_FACING, 2*len(endpoints)+1)
        label.pose.position = Point(x=.12, y=0., z=.78)
        label.scale.z = .025
        label.text = getattr(self, 'baseline_label', 'COLLISION') if baseline else 'AVOIDANCE'
        label.color.r, label.color.g, label.color.b = (1., .3, .2) if baseline else (.2, 1., .5)
        array.markers.append(label)
        self.markers.publish(array)
