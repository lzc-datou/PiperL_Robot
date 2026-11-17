"""Six-axis target interaction and Cartesian feedback controller."""
import json
import time
import signal
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Pose, Point, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64, Bool, ColorRGBA
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit_msgs.srv import GetPositionIK, ServoCommandType
from moveit_msgs.msg import DisplayRobotState, ObjectColor
from control_msgs.msg import JointJog, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from piper_servo_tracking.srv import PlanJointTarget, JointPlanCommand
from .core import JOINTS, FreshJoints, RobotModel, normalized_pose, pose_error, load_config, surface_vertex_colors, joint_speed_limits, damped_joint_velocity
from .cartesian_control import CartesianController
from .execution_feedback import ExecutionObserver
from .ik_selection import make_ik_seeds, select_ik_candidate


def pose_values(p):
    return normalized_pose([p.position.x, p.position.y, p.position.z,
                            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def pose_message(p):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, p[:3])
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, p[3:])
    return msg


class Backend(Node):
    def __init__(self):
        super().__init__('servo_backend')
        self.declare_parameter('config_file', '')
        self.declare_parameter('control_config_file', '')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('mode', 'sim')
        self.config = load_config(self.get_parameter('config_file').value,
                                  self.get_parameter('control_config_file').value or None)
        self.controller = CartesianController(self.config['control'])
        control = self.config['control']
        self.execution_observer = ExecutionObserver(freshness=control['execution_feedback_timeout'],
            stable_window=control['observer_stable_window'], saturation_window=control['observer_saturation_window'],
            linear_noise=control['observer_linear_noise'], angular_noise=control['observer_angular_noise'],
            relative_noise=control['observer_relative_noise'])
        self.execution_source_stamps = {}
        self.mode = self.get_parameter('mode').value
        self.model = RobotModel(self.get_parameter('robot_description').value)
        self.feedback = FreshJoints()
        self.target = None
        self.approved_target = None
        self.approved_target_time = 0.
        self.approved_revision = -1
        self.ik_context = None
        self.verification_gap_since = None
        self.revision, self.valid_revision = 0, -1
        self.valid, self.pending = False, None
        self.active = self.dragging = self.switched = False
        self.drag_client = ''
        self.drag_seen = self.panel_seen = self.pending_since = 0.
        self.ik_seed_revision = self.ik_pending_revision = -1
        self.ik_seeds, self.ik_seed_index = [], 0
        self.ik_best_q, self.ik_best_margin = None, -1.
        self.switch_future = None
        self.pause_revision = -1
        self.resume_future = None
        self.resume_request_id, self.resume_requested_at = 0, 0.
        self.last_resume_attempt = 0.
        self.start_pending = False
        self.aligned_revision = -1
        self.preview_q = None
        self.ghost_revision = -1
        self.ghost_message = None
        self.control_mode = 'servo'
        self.joint_epoch = 0
        self.joint_id = ''
        self.joint_purpose = ''
        self.joint_revision = -1
        self.joint_busy = self.plan_ready = False
        self.joint_wait = None
        self.joint_future = None
        self.guard_can_plan = False
        self.settle_since, self.settle_q = 0., None
        self.hold_future = None
        self.hold_revision = -1
        self.hold_requested_at = 0.
        self.hold_reason = ''
        self.motion_recovery_required = self.recovery_hold = False
        self.recovery_attempt_revision = -1
        self.recovery_since = self.last_target_change = 0.
        self.stall_since, self.stall_error, self.stall_q = 0., None, None
        self.scale = self.config['default_speed_scale']
        self.guard_state, self.guard_reason = 'idle', '等待反馈'
        self.reason, self.reachability = '等待反馈', '未检查'
        self.workspace_visible = True
        self.ik = self.create_client(GetPositionIK, 'compute_ik')
        self.switch = self.create_client(ServoCommandType, 'servo_node/switch_command_type')
        self.gate = self.create_client(SetBool, 'guard/enable')
        self.hold_gate = self.create_client(Trigger, 'guard/hold')
        self.joint_plan = self.create_client(PlanJointTarget, 'guard/plan_joint_target')
        self.joint_command = self.create_client(JointPlanCommand, 'guard/joint_plan_command')
        self.twist = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 1)
        self.jog = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 1)
        self.heartbeat = self.create_publisher(Float64, 'backend/heartbeat', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, 'status', qos)
        self.workspace_pub = self.create_publisher(MarkerArray, 'workspace', qos)
        self.ghost_pub = self.create_publisher(DisplayRobotState, 'target_robot_state', qos)
        self.create_subscription(JointState, 'feedback/joint_states' if self.mode == 'real' else 'commanded_joint_states', self.on_feedback, 1)
        self.create_subscription(String, 'guard/status', self.on_guard, 1)
        self.create_subscription(JointTrajectory, 'servo/trajectory', self.on_servo_trajectory, 1)
        self.create_subscription(JointTrajectoryControllerState, 'arm_controller/controller_state', self.on_controller_state, 1)
        self.create_subscription(Float64, 'panel/heartbeat', self.on_panel, 1)
        self.create_subscription(Float64, 'speed_scale', self.on_speed, 1)
        self.create_subscription(Bool, 'workspace_visible', self.on_workspace, 1)
        self.create_service(Trigger, 'start', self.start)
        self.create_service(Trigger, 'stop', self.stop_service)
        self.create_service(Trigger, 'reset', self.reset_service)
        self.create_service(Trigger, 'home', self.home_service)
        self.create_service(SetBool, 'control_mode', self.mode_service)
        self.create_service(Trigger, 'plan', self.plan_service)
        self.create_service(Trigger, 'execute', self.execute_service)
        self.server = InteractiveMarkerServer(self, 'target')
        started = time.monotonic()
        self.workspace_triangles, resolution = self.model.workspace_surface(
            int(self.config['workspace_samples']), self.config['workspace_resolution'])
        self.workspace_colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=.999)
                                 for c in surface_vertex_colors(self.workspace_triangles)]
        self.get_logger().info(f'Workspace surface: {len(self.workspace_triangles)} triangles, '
                               f'{resolution:.3f} m, {time.monotonic()-started:.2f} s')
        self.publish_workspace()
        self.create_timer(1./self.config['rate'], self.tick)
        self.create_timer(.05, self.validation_tick)
        self.create_timer(.1, self.publish_status)
        # The official RobotState display subscribes with volatile QoS. Replay the
        # cached visual state so a late/re-enabled RViz also receives the target.
        self.create_timer(.5, self.republish_ghost)

    def on_panel(self, msg):
        if 0 <= time.time()-msg.data < .3:
            self.panel_seen = time.monotonic()

    def on_speed(self, msg):
        if np.isfinite(msg.data) and .1 <= msg.data <= 1.:
            if abs(msg.data-self.scale) > 1e-6 and (self.joint_busy or self.plan_ready):
                self.stop_tracking('速度已改变，请重新规划')
            self.scale = msg.data

    def on_workspace(self, msg):
        self.workspace_visible = msg.data
        self.publish_workspace()

    def on_feedback(self, msg):
        previous_stamp = self.feedback.stamp
        self.feedback.update(msg.name, msg.position, msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9)
        observer = getattr(self, 'execution_observer', None)
        if observer is not None and self.feedback.stamp > previous_stamp and not self.feedback.error:
            try:
                q = [msg.position[msg.name.index(name)] for name in JOINTS]
                # Raw joint positions cover real firmware as well as sim;
                # do not trust absent/placeholder velocity fields as zero speed.
                observer.observe_joints('measured', time.monotonic(), q, self.model)
            except (ValueError, IndexError):
                pass  # An incomplete source never refreshes execution authority.

    def on_servo_trajectory(self, msg):
        if not msg.points or len(msg.points[0].velocities) != len(msg.joint_names):
            self.execution_observer.observe('servo', time.monotonic(), [np.nan]*6)
            return
        try:
            if not Backend.fresh_execution_source(self, 'servo', msg.header.stamp):
                return
            indices = [msg.joint_names.index(name) for name in JOINTS]
            point = msg.points[0]
            q = [point.positions[i] for i in indices]
            velocity = [point.velocities[i] for i in indices]
            self.execution_observer.observe_joints('servo', time.monotonic(), q, self.model, velocity)
        except (ValueError, IndexError):
            self.execution_observer.observe('servo', time.monotonic(), [np.nan]*6)

    def on_controller_state(self, msg):
        try:
            if not Backend.fresh_execution_source(self, 'controller', msg.header.stamp):
                return
            reference = msg.reference
            indices = [msg.joint_names.index(name) for name in JOINTS]
            q = [reference.positions[i] for i in indices]
            velocity = [reference.velocities[i] for i in indices]
            self.execution_observer.observe_joints('controller', time.monotonic(), q, self.model, velocity)
        except (AttributeError, ValueError, IndexError):
            self.execution_observer.observe('controller', time.monotonic(), [np.nan]*6)

    def fresh_execution_source(self, stage, stamp):
        value = stamp.sec + stamp.nanosec*1e-9
        previous = self.execution_source_stamps.get(stage, 0.)
        # Native Servo stamps its buffered trajectory in the near future.
        # This is a command observation, not evidence of measured motion.
        if not np.isfinite(value) or not -.15 <= time.time()-value <= .15 or value <= previous:
            return False
        self.execution_source_stamps[stage] = value
        return True

    def raw_actual(self):
        # FK and visual reset use fresh measurements even when motion is blocked.
        # Never clip feedback or turn a visual reset into zero calibration.
        q = self.feedback.get(self.config['feedback_timeout'])
        return q, self.model.pose(q)

    def actual(self):
        q, pose = self.raw_actual()
        self.model.check_feedback(q, zero_tolerance=(
            self.config.get('zero_feedback_tolerance', 0.) if self.mode == 'real' else 0.))
        return q, pose

    def on_guard(self, msg):
        data = json.loads(msg.data)
        previous = self.guard_state
        self.guard_state, self.guard_reason = data['state'], data['reason']
        self.guard_can_plan = data.get('can_plan', False)
        if self.guard_state == 'running' and previous != 'running':
            Backend.reset_controller(self)
            # The authoritative state supersedes a delayed enable reply.
            self.resume_future = None
            self.resume_request_id = getattr(self, 'resume_request_id', 0) + 1
            self.recovery_hold = False
        if (self.active or getattr(self, 'joint_busy', False) or getattr(self, 'plan_ready', False)) and self.guard_state == 'fault' and previous != 'fault':
            self.stop_tracking(self.guard_reason)
        elif self.active and self.guard_state == 'paused' and previous != 'paused':
            # Native Servo has already halted. Keep the requested full pose,
            # then solve a bounded joint branch without reusing old authority.
            self.pause_revision = self.revision
            # A native pause already closes forwarding. A concurrent hold may
            # now be rejected because Servo is no longer running; its late
            # reply must not cancel this safely paused session.
            self.hold_future = None
            self.hold_reason = ''
            self.resume_future = None
            self.resume_request_id = getattr(self, 'resume_request_id', 0) + 1
            Backend.require_motion_recovery(self, self.guard_reason, request_hold=False)
        elif self.active and self.guard_state == 'holding' and previous != 'holding':
            Backend.reset_controller(self)
            # Lock the settled session to its old revision. Feedback noise alone
            # cannot re-authorize motion; a newly edited, verified goal can.
            self.pause_revision = (self.hold_revision if self.hold_future is not None and
                                   self.hold_revision >= 0 else self.revision)
            self.hold_future = None
            self.settle_since, self.settle_q = 0., None
            self.reason = (getattr(self, 'hold_reason', '') or
                           ('已到位 · 保持目标' if self.valid else '目标不可用：' + self.reachability))
            if self.valid and self.revision > self.pause_revision:
                self.reason = '等待恢复跟踪'
            if getattr(self, 'motion_recovery_required', False):
                self.reason = '等待调整关节姿态'
            self.publish_twist(np.zeros(6))
        if getattr(self, 'joint_id', '') and data.get('plan_id') == self.joint_id:
            if self.guard_state == 'joint_ready':
                self.joint_busy = False
                self.plan_ready = bool(data.get('plan_ready')) and self.joint_revision == self.revision
                self.reason = '规划完成，点击执行' if self.plan_ready else '计划已失效，请重新规划'
            elif self.guard_state == 'joint_done':
                purpose = self.joint_purpose
                self.joint_id = self.joint_purpose = ''
                self.joint_busy = self.plan_ready = False
                self.reason = '单点已到达'
                if purpose == 'ready' and self.active and self.control_mode == 'servo':
                    self.finish_ready()
                elif purpose == 'limit_recovery' and self.active:
                    # The official joint plan reached the complete requested
                    # pose. Stay closed instead of starting another Cartesian
                    # motion or sending a singular goal through startup-ready.
                    self.motion_recovery_required = False
                    self.recovery_hold = True
                    self.pause_revision = self.revision
                    self.approved_target = None
                    self.reason = self.hold_reason = '已到位 · 保持目标'
                elif purpose == 'home':
                    # Home is a user-requested motion, never zero calibration.
                    # Align the next target to measured FK and leave Servo off.
                    self.active = self.start_pending = False
                    try:
                        self.align()
                        self.reason = '已回零'
                    except ValueError:
                        self.target = self.approved_target = None
                        self.valid, self.valid_revision = False, -1
                        self.reason = '回零已结束，等待实时反馈对齐目标'
            elif self.guard_state.startswith('joint_'):
                self.joint_busy = True
                self.reason = ('调整关节姿态：' if self.joint_purpose == 'limit_recovery' else '') + self.guard_reason

    def finish_ready(self):
        # Preparation must never silently replace a user goal or immediately pull
        # the arm back to its singular startup hold target.
        try:
            q, _ = self.actual()
            if not self.valid or self.valid_revision != self.revision or self.preview_q is None:
                raise ValueError('准备位已到达；原目标保留，请调整或重置目标后开启跟踪')
            self.joint_busy = True
            self.joint_wait = (q.copy(), 'handoff', time.monotonic())
            self.reason = '准备位已到达，等待控制交接'
        except ValueError as exc:
            self.stop_tracking(str(exc))

    def marker(self, pose):
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name, marker.description = 'tcp_target', 'TCP · XYZ / RPY'
        marker.pose, marker.scale = pose_message(pose), .18
        visual = InteractiveMarkerControl()
        visual.name, visual.description = 'move_free', '拖动目标球平移；箭头控制深度'
        visual.always_visible = True
        # Camera-facing MOVE_PLANE works with an ordinary left drag, no modifier keys.
        visual.orientation.w = 1.
        visual.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        visual.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.
        sphere.scale.x = sphere.scale.y = sphere.scale.z = .06
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1., .75, .1, .8
        visual.markers.append(sphere)
        marker.controls.append(visual)
        for axis, quaternion in [('x', (1., 0., 0., 1.)), ('y', (0., 0., 1., 1.)), ('z', (0., -1., 0., 1.))]:
            for name, mode in [('move', InteractiveMarkerControl.MOVE_AXIS), ('rotate', InteractiveMarkerControl.ROTATE_AXIS)]:
                control = InteractiveMarkerControl()
                control.name = f'{name}_{axis}'
                # RViz controls act along/about their local X axis; fixed orientation uses base axes.
                q = np.asarray(quaternion)/np.linalg.norm(quaternion)
                control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = map(float, q)
                control.orientation_mode = InteractiveMarkerControl.FIXED
                control.interaction_mode = mode
                marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self.on_marker)
        self.server.applyChanges()
        self.marker_color = 'checking'

    def align(self):
        Backend.reset_controller(self)
        q, actual = self.raw_actual()
        self.target = actual.copy()
        self.last_target_change = self.approved_target_time = time.monotonic()
        self.revision += 1
        # This only aligns the display with measured FK. A tolerated zero-boundary
        # measurement is a hold placeholder, never a newly authorized joint goal.
        self.aligned_revision = self.valid_revision = self.revision
        self.approved_revision = self.revision
        try:
            self.actual()
            self.valid = True
            self.approved_target = self.target.copy()
            self.reachability = '当前位置'
            self.reason = '目标已对齐实测位置'
        except ValueError as exc:
            self.valid = False
            self.approved_target = None
            self.reachability = str(exc)
            self.reason = '已对齐实测；运动不可用：' + str(exc)
        self.preview_q = q.copy()
        self.publish_ghost(q)
        self.marker(self.target)

    def publish_ghost(self, q=None):
        msg = DisplayRobotState()
        msg.hide = q is None
        if q is None and getattr(self, 'ghost_message', None) is not None:
            # MoveIt's display converts the state before honoring hide; retain
            # the last complete visual state instead of emitting empty joints.
            msg.state = deepcopy(self.ghost_message.state)
        if q is not None:
            msg.state.joint_state.header.frame_id = 'base_link'
            msg.state.joint_state.header.stamp = self.get_clock().now().to_msg()
            msg.state.joint_state.name = JOINTS
            msg.state.joint_state.position = list(map(float, q))
            msg.state.is_diff = False
            msg.highlight_links = [ObjectColor(id=name,
                color=ColorRGBA(r=1., g=.85, b=.05, a=1.)) for name in self.model.link_names]
            self.ghost_revision = self.revision
        else:
            self.ghost_revision = -1
        # Visualisation only. This state is never published to a control/feedback topic.
        self.ghost_message = msg
        self.ghost_pub.publish(msg)

    def republish_ghost(self):
        if self.ghost_message is not None:
            self.ghost_pub.publish(self.ghost_message)

    def on_marker(self, msg):
        if msg.marker_name != 'tcp_target' or msg.header.frame_id != 'base_link':
            return
        if msg.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            if self.dragging and msg.client_id != self.drag_client:
                self.stop_tracking('检测到其他拖动客户端')
                return
            self.dragging = True
            Backend.reset_controller(self)
            self.drag_client = msg.client_id
            self.drag_seen = time.monotonic()
        elif msg.client_id == self.drag_client and msg.event_type == InteractiveMarkerFeedback.KEEP_ALIVE:
            self.drag_seen = time.monotonic()
        elif msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self.dragging and msg.client_id != self.drag_client:
                self.stop_tracking('检测到其他拖动客户端')
                return
            self.update_target(msg.pose)
        elif msg.event_type == InteractiveMarkerFeedback.MOUSE_UP and msg.client_id == self.drag_client:
            # Release ends mouse capture, not the tracking session. Validate the final pose too.
            self.update_target(msg.pose)
            self.dragging = False

    def update_target(self, pose):
        try:
            target = pose_values(pose)
            self.drag_seen = time.monotonic()
            if self.target is not None and np.allclose(target, self.target, rtol=0, atol=1e-9):
                return
            if getattr(self, 'joint_busy', False) or getattr(self, 'plan_ready', False):
                retarget_recovery = (self.active and self.control_mode == 'servo' and
                                     getattr(self, 'joint_purpose', '') == 'limit_recovery')
                self.stop_tracking('目标已改变，等待旧调整停止' if retarget_recovery else
                                   '目标已改变，请重新规划' if self.control_mode == 'point' else
                                   '准备动作已取消，请重新开启')
                if retarget_recovery:
                    # A changed target may retire the old action, never re-use it.
                    # Wait for Guard's terminal/closed barrier before replanning
                    # the newest verified target after this drag is released.
                    self.active, self.motion_recovery_required = True, True
                    self.recovery_since = time.monotonic()
            if self.verification_gap_since is None:
                self.verification_gap_since = time.monotonic()
            self.target = target
            self.revision += 1
            self.last_target_change = time.monotonic()
            Backend.clear_stall_window(self)
            # Keep the in-flight snapshot: a valid result may be a useful
            # streaming checkpoint even when the mouse has advanced again.
            if self.pending is None:
                Backend.clear_ik_seeds(self)
            self.settle_since, self.settle_q = 0., None
            self.valid = False
            self.reachability = '检查中'
            if np.linalg.norm(target[:3]) > self.model.radius:
                self.reject('目标超出可达范围')
        except ValueError as exc:
            self.stop_tracking(str(exc))

    def start(self, request, response):
        try:
            if getattr(self, 'control_mode', 'servo') != 'servo':
                raise ValueError('请切换到伺服模式')
            if self.active:
                raise ValueError('跟踪已开启')
            if time.monotonic()-self.panel_seen > .5:
                raise ValueError('RViz 面板未连接')
            if not self.switched or not self.ik.service_is_ready() or not self.gate.service_is_ready():
                raise ValueError('等待 Servo / MoveIt 就绪')
            q, actual = self.actual()
            if self.target is None:
                raise ValueError('等待目标初始化')
            if self.aligned_revision == self.revision:
                dp, dr = pose_error(self.target, actual)
                if (np.linalg.norm(dp) > self.config['position_tolerance'] or
                        np.linalg.norm(dr) > self.config['orientation_tolerance']):
                    # Feedback changed since reset: recheck the existing target, never move it.
                    self.valid = False
                    self.valid_revision = -1
                    self.aligned_revision = -1
                    self.reachability = '检查中'
            if self.valid_revision == self.revision and not self.valid:
                raise ValueError('目标不可用：' + self.reachability + '；请调整或重置目标')
            self.active, self.dragging = True, False
            if self.valid and self.valid_revision == self.revision:
                self.begin_tracking()
            else:
                self.start_pending = True
                self.reason = '正在检查目标'
            response.success, response.message = True, self.reason
        except ValueError as exc:
            if self.active:
                self.stop_tracking(str(exc))
            self.reason = str(exc)
            response.success, response.message = False, str(exc)
        return response

    def begin_tracking(self):
        self.start_pending = False
        self.actual()
        self.reason = '正在准备'
        self.settle_since, self.settle_q = 0., None
        future = self.gate.call_async(SetBool.Request(data=True))
        epoch = getattr(self, 'joint_epoch', 0)
        future.add_done_callback(lambda f: self.start_done(f) if epoch == getattr(self, 'joint_epoch', 0) else None)

    def mode_service(self, request, response):
        mode = 'point' if request.data else 'servo'
        if mode != self.control_mode:
            self.stop_tracking('已切换到单点' if mode == 'point' else '已切换到伺服')
            self.control_mode = mode
        response.success, response.message = True, self.reason
        return response

    def plan_service(self, request, response):
        try:
            if self.control_mode != 'point':
                raise ValueError('请切换到单点模式')
            if self.joint_busy:
                raise ValueError('当前动作未结束')
            if time.monotonic()-self.panel_seen > .5:
                raise ValueError('RViz 面板未连接')
            q, _ = self.actual()
            if not self.valid or self.valid_revision != self.revision or self.ghost_revision != self.revision:
                raise ValueError('请先设置可达目标：' + self.reachability)
            if self.preview_q is not None and np.max(np.abs(self.preview_q-q)) <= .002:
                self.reason = '已在目标位置，无需执行'
                response.success, response.message = True, self.reason
                return response
            self.queue_joint_plan(self.preview_q, 'point')
            response.success, response.message = True, self.reason
        except ValueError as exc:
            self.reason = str(exc)
            response.success, response.message = False, str(exc)
        return response

    def queue_joint_plan(self, joints, purpose):
        Backend.reset_controller(self)
        if not self.joint_plan.service_is_ready() or not self.joint_command.service_is_ready():
            raise ValueError('等待关节规划服务')
        self.model.check_joints(joints, 0. if purpose == 'home' else self.config['joint_margin'])
        self.joint_epoch += 1
        self.joint_id = ''
        self.joint_revision, self.joint_purpose = self.revision, purpose
        self.plan_ready, self.joint_busy = False, True
        self.joint_future = None
        self.guard_can_plan = False
        self.joint_wait = (np.asarray(joints).copy(), purpose, time.monotonic())
        self.reason = ('正在规划准备姿态' if purpose == 'ready' else
                       '调整关节姿态' if purpose == 'limit_recovery' else
                       '正在准备回零' if purpose == 'home' else '正在准备单点规划')
        self.publish_twist(np.zeros(6))
        # Close/cancel any old controller command before asking the independent
        # guard to plan. The watchdog advertises when stop/hold is acknowledged.
        self.gate.call_async(SetBool.Request(data=False))

    def joint_request_tick(self):
        if self.joint_wait is None:
            return
        joints, purpose, since = self.joint_wait
        if time.monotonic()-since > 6.:
            self.stop_tracking('等待控制器停止确认超时')
            return
        if not self.guard_can_plan:
            return
        self.joint_wait = None
        if purpose == 'handoff':
            self.joint_busy = False
            self.begin_tracking()
            return
        req = PlanJointTarget.Request(joints=joints.tolist(), speed_scale=float(self.scale),
            purpose=purpose, target_revision=self.joint_revision,
            auto_execute=purpose in ('ready', 'limit_recovery', 'home'))
        epoch = self.joint_epoch
        self.joint_future = self.joint_plan.call_async(req)
        self.joint_future.add_done_callback(lambda f: self.joint_plan_done(f, epoch))

    def joint_plan_done(self, future, epoch):
        try:
            result = future.result()
            if epoch != self.joint_epoch:
                if result.success and result.plan_id:
                    self.joint_command.call_async(JointPlanCommand.Request(plan_id=result.plan_id, command='cancel'))
                return
            self.joint_future = None
            if not result.success:
                self.stop_tracking(result.message)
                return
            self.joint_id = result.plan_id
            self.reason = result.message
        except Exception as exc:
            if epoch == self.joint_epoch:
                self.stop_tracking('关节规划请求失败：' + str(exc))

    def execute_service(self, request, response):
        try:
            if self.control_mode != 'point' or not self.plan_ready or not self.joint_id:
                raise ValueError('请先规划单点目标')
            if self.joint_revision != self.revision or not self.valid:
                raise ValueError('目标已改变，请重新规划')
            if time.monotonic()-self.panel_seen > .5:
                raise ValueError('RViz 面板未连接')
            self.plan_ready, self.joint_busy = False, True
            epoch = self.joint_epoch
            future = self.joint_command.call_async(JointPlanCommand.Request(plan_id=self.joint_id, command='execute'))
            future.add_done_callback(lambda f: self.joint_execute_done(f, epoch))
            self.reason = '正在准备单点执行'
            response.success, response.message = True, self.reason
        except ValueError as exc:
            self.reason = str(exc)
            response.success, response.message = False, str(exc)
        return response

    def joint_execute_done(self, future, epoch):
        if epoch != self.joint_epoch:
            return
        try:
            result = future.result()
            if not result.success:
                self.stop_tracking(result.message)
        except Exception as exc:
            self.stop_tracking('单点执行请求失败：' + str(exc))

    def clear_stall_window(self):
        self.stall_since, self.stall_error, self.stall_q = 0., None, None

    def require_motion_recovery(self, reason, request_hold=True):
        Backend.reset_controller(self)
        self.motion_recovery_required, self.recovery_hold = True, False
        self.recovery_since = time.monotonic()
        self.start_pending = False
        self.approved_target = None
        self.valid, self.valid_revision = False, -1
        self.preview_q = None
        self.verification_gap_since = None
        Backend.clear_ik_seeds(self, cancel_pending=True)
        Backend.clear_stall_window(self)
        self.publish_ghost()
        self.publish_twist(np.zeros(6))
        self.reachability = '检查中'
        self.reason = '等待调整关节姿态：' + reason
        if request_hold and self.guard_state == 'running' and self.hold_future is None:
            if not self.hold_gate.service_is_ready():
                raise ValueError('关节调整保持服务不可用')
            self.hold_reason = self.reason
            self.hold_revision, self.hold_requested_at = self.revision, time.monotonic()
            epoch = self.joint_epoch
            self.hold_future = self.hold_gate.call_async(Trigger.Request())
            self.hold_future.add_done_callback(lambda f: self.hold_done(f, epoch))

    def recovery_tick(self, q, actual):
        now = time.monotonic()
        if getattr(self, 'motion_recovery_required', False):
            Backend.clear_stall_window(self)
            if (self.dragging or not self.valid or self.valid_revision != self.revision or
                    self.preview_q is None or getattr(self, 'joint_busy', False) or
                    now-max(getattr(self, 'last_target_change', 0.), self.recovery_since) < .15):
                return
            if getattr(self, 'recovery_attempt_revision', -1) == self.revision:
                self.reason = '当前目标已调整一次，请继续调整目标'
                return
            if (not self.guard_can_plan or self.guard_state not in ('paused', 'holding', 'joint_done', 'idle') or
                    not self.joint_plan.service_is_ready() or not self.joint_command.service_is_ready()):
                return
            self.recovery_attempt_revision = self.revision
            self.queue_joint_plan(self.preview_q, 'limit_recovery')
            return
        if (self.guard_state != 'running' or self.dragging or
                not getattr(self, 'valid', False) or self.valid_revision != self.revision or
                self.approved_target is None or
                now-getattr(self, 'last_target_change', 0.) < .15):
            Backend.clear_stall_window(self)
            return
        dp, dr = pose_error(self.approved_target, actual)
        error = float(np.hypot(np.linalg.norm(dp)/self.config['hold_position_tolerance'],
                               np.linalg.norm(dr)/self.config['hold_orientation_tolerance']))
        if error <= 1.:
            Backend.clear_stall_window(self)
        elif (not getattr(self, 'stall_since', 0.) or
              np.max(np.abs(q-self.stall_q)) >= .003 or error < self.stall_error*.99):
            self.stall_since, self.stall_error, self.stall_q = now, error, q.copy()
        elif now-self.stall_since >= 1.5:
            Backend.require_motion_recovery(self, '目标跟踪停滞，重新选择关节分支')

    def maybe_resume(self, q, actual):
        if getattr(self, 'motion_recovery_required', False):
            return
        approved_revision = getattr(self, 'approved_revision', self.valid_revision)
        if (self.resume_future is not None or self.approved_target is None or
                approved_revision <= self.pause_revision or
                time.monotonic()-self.last_resume_attempt < (.1 if self.guard_state == 'holding' else .5)):
            return
        if self.gate.service_is_ready():
            self.last_resume_attempt = time.monotonic()
            self.resume_requested_at = self.last_resume_attempt
            self.resume_request_id = getattr(self, 'resume_request_id', 0) + 1
            request_id, epoch = self.resume_request_id, getattr(self, 'joint_epoch', 0)
            self.resume_future = self.gate.call_async(SetBool.Request(data=True))
            self.resume_future.add_done_callback(lambda f: self.resume_done(f, request_id, epoch))

    def resume_done(self, future, request_id=None, epoch=None):
        if (self.resume_future is not future or
                (request_id is not None and request_id != self.resume_request_id) or
                (epoch is not None and epoch != getattr(self, 'joint_epoch', 0))):
            return
        self.resume_future = None
        self.resume_request_id = getattr(self, 'resume_request_id', 0) + 1
        try:
            result = future.result()
            self.reason = '正在准备' if result.success else result.message
        except Exception as exc:
            self.reason = '恢复未完成：' + str(exc)

    def start_done(self, future):
        try:
            result = future.result()
            if not result.success:
                self.stop_tracking(result.message)
        except Exception as exc:
            self.stop_tracking('启动失败：' + str(exc))

    def stop_tracking(self, reason):
        Backend.reset_controller(self)
        self.active = self.dragging = False
        self.start_pending = False
        self.valid = False
        self.valid_revision = -1
        self.approved_target = None
        Backend.clear_ik_seeds(self, cancel_pending=True)
        # A pending service may complete later, but the independent guard's stop epoch
        # prevents it from opening the driver gate again.
        self.resume_future = None
        self.resume_request_id = getattr(self, 'resume_request_id', 0) + 1
        self.motion_recovery_required = self.recovery_hold = False
        self.recovery_attempt_revision = -1
        Backend.clear_stall_window(self)
        self.joint_epoch = getattr(self, 'joint_epoch', 0) + 1
        self.joint_id = self.joint_purpose = ''
        self.joint_busy = self.plan_ready = False
        self.joint_wait = self.joint_future = None
        self.guard_can_plan = False
        self.hold_future = None
        self.hold_revision = -1
        self.hold_reason = ''
        self.settle_since, self.settle_q = 0., None
        self.reason = reason
        self.publish_twist(np.zeros(6))
        if self.gate.service_is_ready():
            self.gate.call_async(SetBool.Request(data=False))

    def stop_service(self, request, response):
        self.stop_tracking('已停止')
        response.success, response.message = True, '已停止'
        return response

    def reset_service(self, request, response):
        # Reset cancels tracking first. It cannot generate motion or re-open a gate.
        self.stop_tracking('目标已重置')
        try:
            self.align()
            response.success, response.message = True, self.reason
        except ValueError as exc:
            self.reason = str(exc)
            response.success, response.message = False, str(exc)
        return response

    def home_service(self, request, response):
        # Revoke the old session before queueing. The independent guard still
        # requires fresh diagnostics, a closed gate and retired old actions.
        self.stop_tracking('正在准备回零')
        try:
            if time.monotonic()-self.panel_seen > .5:
                raise ValueError('RViz 面板未连接')
            self.actual()
            self.queue_joint_plan(np.zeros(6), 'home')
            response.success, response.message = True, self.reason
        except (ValueError, RuntimeError) as exc:
            self.reason = str(exc)
            response.success, response.message = False, self.reason
        return response

    def validation_tick(self):
        if (self.guard_state == 'paused' and self.revision <= self.pause_revision and
                not getattr(self, 'motion_recovery_required', False)):
            return
        if self.pending is not None:
            if time.monotonic()-self.pending_since > self.config['ik_timeout']:
                pending_revision = getattr(self, 'ik_pending_revision', self.revision)
                Backend.clear_ik_seeds(self, cancel_pending=True)
                if pending_revision == self.revision:
                    self.reject('逆解检查超时')
            return
        if self.target is None or self.valid_revision == self.revision or not self.ik.service_is_ready():
            return
        try:
            q, _ = self.actual()
            if np.linalg.norm(self.target[:3]) > self.model.radius:
                raise ValueError('目标超出可达范围')
            tolerance = self.config.get('zero_feedback_tolerance', 0.) if self.mode == 'real' else 0.
            reference = self.model.planning_joints(q, zero_tolerance=tolerance)
            self.ik_seeds = make_ik_seeds(reference, self.model.lower, self.model.upper,
                previous=getattr(self, 'preview_q', None), ready=self.config['ready_joints'])
            self.ik_candidates = []
            self.ik_reference = reference.copy()
            self.ik_seed_revision, self.ik_seed_index = self.revision, 0
            self.ik_best_q, self.ik_best_margin = None, -1.
            self.ik_context = dict(target=self.target.copy(),
                target_time=getattr(self, 'last_target_change', time.monotonic()),
                revision=self.revision, epoch=getattr(self, 'joint_epoch', 0))
            self.ik_collect_all = (not self.active or getattr(self, 'control_mode', 'servo') == 'point' or
                                   getattr(self, 'motion_recovery_required', False))
            Backend.submit_ik_seed(self)
        except ValueError as exc:
            self.reject(str(exc))

    def clear_ik_seeds(self, cancel_pending=False):
        self.ik_seed_revision, self.ik_seeds, self.ik_seed_index = -1, [], 0
        self.ik_best_q, self.ik_best_margin = None, -1.
        if cancel_pending:
            pending = getattr(self, 'pending', None)
            self.pending = None
            self.ik_pending_revision = -1
            self.ik_context = None
            if pending is not None:
                pending.cancel()
        elif getattr(self, 'pending', None) is None:
            self.ik_pending_revision = -1

    def submit_ik_seed(self):
        # Seed normalization never updates feedback or authorizes the returned
        # pose. Each result still passes collision-aware IK, strict bounds and FK.
        context = getattr(self, 'ik_context', None)
        live_checkpoint = (getattr(self, 'active', False) and context is not None and
            getattr(self, 'control_mode', 'servo') == 'servo' and
            context['epoch'] == getattr(self, 'joint_epoch', 0) and
            self.ik_seed_revision > self.pause_revision and
            time.monotonic()-context['target_time'] < self.config['ik_timeout'])
        if self.pending is not None or (self.ik_seed_revision != self.revision and not live_checkpoint):
            return
        self.actual()  # The fallback may only run with fresh, accepted feedback.
        seed = self.ik_seeds[self.ik_seed_index]
        self.ik_seed_index += 1
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'tcp_link'
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = seed.tolist()
        req.ik_request.pose_stamped.header.frame_id = 'base_link'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = pose_message(context['target'] if context is not None else self.target)
        req.ik_request.timeout.nanosec = 30_000_000
        self.pending_since = time.monotonic()
        self.ik_pending_revision = self.ik_seed_revision
        self.pending = self.ik.call_async(req)
        self.pending.add_done_callback(lambda f, rev=self.ik_seed_revision: self.ik_done(f, rev))

    def ik_done(self, future, revision):
        if future.cancelled() or self.pending is not future:
            return
        self.pending = None
        self.ik_pending_revision = -1
        context = getattr(self, 'ik_context', None)
        streaming = (revision != self.revision and self.active and
            getattr(self, 'control_mode', 'servo') == 'servo' and context is not None and
            context['epoch'] == getattr(self, 'joint_epoch', 0) and
            revision > getattr(self, 'pause_revision', -1) and
            not getattr(self, 'motion_recovery_required', False) and
            time.monotonic()-context['target_time'] <= self.config['ik_timeout']+.05)
        if revision != self.revision and not streaming:
            if getattr(self, 'ik_seed_revision', -1) == revision:
                Backend.clear_ik_seeds(self)
            return  # A result for a previous target can never authorize a newer target.
        try:
            result = future.result()
            if result.error_code.val != result.error_code.SUCCESS:
                raise ValueError('目标不可达或碰撞')
            js = result.solution.joint_state
            q = [js.position[js.name.index(name)] for name in JOINTS]
            self.model.check_joints(q)
            try:
                self.model.check_joints(q, self.config['joint_margin'])
            except ValueError:
                raise ValueError('目标处于关节限位保护区')
            checked_target = context['target'] if context is not None else self.target
            dp, dr = pose_error(checked_target, self.model.pose(q))
            if np.linalg.norm(dp) > .003 or np.linalg.norm(dr) > .03:
                raise ValueError('逆解未满足目标位姿')
            if (getattr(self, 'ik_collect_all', False) and not streaming and
                    getattr(self, 'ik_seed_revision', -1) == revision):
                q = np.asarray(q)
                self.ik_candidates.append(q.copy())
                self.ik_best_q = select_ik_candidate(self.ik_candidates, self.ik_reference,
                    self.model.lower, self.model.upper, margin=self.config['joint_margin'])
                if self.ik_seed_index < len(self.ik_seeds):
                    Backend.submit_ik_seed(self)
                    return
                q = self.ik_best_q.copy()
            Backend.accept_ik(self, q, revision,
                target=checked_target, target_time=context['target_time'] if context is not None else None)
        except (ValueError, IndexError, RuntimeError) as exc:
            if streaming:
                if (self.ik_seed_index < len(self.ik_seeds) and
                        time.monotonic()-context['target_time'] < self.config['ik_timeout']):
                    Backend.submit_ik_seed(self)
                    return
                # This rejected checkpoint cannot validate the newer GUI goal.
                # Revoke old motion, then allow a new request for the new goal.
                self.reject(str(exc))
                self.valid_revision = revision
                self.pause_revision = self.hold_revision = revision
                return
            if (getattr(self, 'ik_seed_revision', -1) == revision and
                    self.ik_seed_index < len(self.ik_seeds)):
                try:
                    Backend.submit_ik_seed(self)
                    return
                except (ValueError, IndexError, RuntimeError) as retry_exc:
                    self.reject(str(retry_exc))
                    return
            if (getattr(self, 'ik_seed_revision', -1) == revision and self.ik_best_q is not None):
                Backend.accept_ik(self, self.ik_best_q.copy(), revision)
                return
            self.reject(str(exc))

    def accept_ik(self, q, revision, target=None, target_time=None):
        current_revision = getattr(self, 'revision', revision)
        self.valid, self.valid_revision = revision == current_revision, revision
        self.approved_revision = revision
        self.approved_target = (self.target if target is None else target).copy()
        self.approved_target_time = (getattr(self, 'last_target_change', time.monotonic())
                                     if target_time is None else target_time)
        self.verification_gap_since = None if self.valid else time.monotonic()
        self.reachability = '可达' if self.valid else '跟踪已验证目标 · 新目标检查中'
        self.preview_q = np.asarray(q).copy()
        Backend.clear_ik_seeds(self)
        self.publish_ghost(q)
        self.ghost_revision = revision
        if self.active and self.start_pending:
            self.begin_tracking()

    def reject(self, reason):
        Backend.reset_controller(self)
        self.valid = False
        self.valid_revision = self.revision
        self.reachability = reason
        self.approved_target = None
        self.preview_q = None
        self.verification_gap_since = None
        Backend.clear_ik_seeds(self, cancel_pending=True)
        self.publish_ghost()
        self.publish_twist(np.zeros(6))
        if not self.active:
            return
        self.reason = '目标不可用：' + reason
        if (getattr(self, 'control_mode', 'servo') != 'servo' or
                getattr(self, 'joint_busy', False) or getattr(self, 'plan_ready', False) or
                self.guard_state not in ('running', 'holding', 'paused')):
            self.stop_tracking(self.reason)
            self.valid_revision = self.revision
            return
        # Retain the session, revoke the old target immediately, and lock this
        # rejected revision. Only a newly edited and verified goal can resume.
        self.start_pending = False
        self.pause_revision = self.hold_revision = self.revision
        self.hold_reason = self.reason
        self.settle_since, self.settle_q = 0., None
        if self.guard_state == 'running' and self.hold_future is None:
            try:
                if not self.hold_gate.service_is_ready():
                    raise ValueError('目标保持服务不可用')
                self.hold_requested_at = time.monotonic()
                epoch = self.joint_epoch
                self.hold_future = self.hold_gate.call_async(Trigger.Request())
                self.hold_future.add_done_callback(lambda f: self.hold_done(f, epoch))
            except (ValueError, RuntimeError) as exc:
                self.stop_tracking('目标保持失败：' + str(exc))

    def publish_twist(self, values):
        values = np.asarray(values, dtype=float)
        jog = getattr(self, 'jog', None)
        velocity = np.zeros(6)
        effective = np.zeros(6)
        if jog is not None and np.any(values):
            try:
                q, _ = self.actual()
                velocity = damped_joint_velocity(q, values, self.model, self.config, self.scale)
                effective = self.model.jacobian(q) @ velocity
            except ValueError:
                values = np.zeros(6)
        # Requested Cartesian velocity remains a diagnostic. Native Servo is
        # explicitly in JOINT_JOG mode and consumes only this bounded qdot.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, values[:3])
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, values[3:])
        self.twist.publish(msg)
        if jog is not None:
            command = JointJog()
            command.header = msg.header
            command.joint_names, command.velocities = JOINTS, velocity.tolist()
            jog.publish(command)
            observer = getattr(self, 'execution_observer', None)
            if observer is not None and self.active and self.guard_state == 'running':
                when = self.controller.last_command_time
                previous = self.controller.last_command_twist
                if when is not None and np.allclose(previous, values, rtol=0., atol=1e-12):
                    observer.record_request(when, values, effective)

    def reset_controller(self):
        controller = getattr(self, 'controller', None)
        if controller is not None:
            controller.reset()
        observer = getattr(self, 'execution_observer', None)
        if observer is not None:
            observer.reset()

    def control_twist(self, actual):
        # The algorithm sees only an IK-approved target and measured FK.
        # It never receives unchecked GUI poses or outputs joint commands.
        now = time.monotonic()
        kwargs = dict(now=now, target_time=self.approved_target_time,
                      speed_scale=self.scale, feedforward_active=self.dragging)
        observer = getattr(self, 'execution_observer', None)
        if observer is not None:
            kwargs['execution_feedback'] = observer.feedback(now)
        feedback = getattr(self, 'feedback', None)
        source_time = getattr(feedback, 'stamp', None)
        if isinstance(source_time, (float, int)) and np.isfinite(source_time) and source_time > 0.:
            kwargs['actual_time'] = float(source_time)
        return self.controller.update(self.approved_target, actual, **kwargs)

    def maybe_hold(self, q, actual):
        if (self.dragging or not self.valid or self.valid_revision != self.revision or
                self.approved_target is None or self.hold_future is not None):
            self.settle_since, self.settle_q = 0., None
            return
        dp, dr = pose_error(self.approved_target, actual)
        now = time.monotonic()
        motion_settled = getattr(getattr(self, 'controller', None), 'motion_settled', True)
        if isinstance(motion_settled, (bool, np.bool_)) and not motion_settled:
            self.settle_since, self.settle_q = 0., None
            return
        inside = (np.linalg.norm(dp) <= self.config['hold_position_tolerance'] and
                  np.linalg.norm(dr) <= self.config['hold_orientation_tolerance'])
        if not inside:
            self.settle_since, self.settle_q = 0., None
        elif self.settle_q is None or np.max(np.abs(q-self.settle_q)) > self.config['hold_joint_stability']:
            self.settle_since, self.settle_q = now, q.copy()
        elif now-self.settle_since >= self.config['hold_settle_time']:
            if not self.hold_gate.service_is_ready():
                raise ValueError('到位保持服务不可用')
            self.hold_revision, self.hold_requested_at = self.revision, now
            self.hold_reason = '已到位 · 保持目标'
            epoch = self.joint_epoch
            self.hold_future = self.hold_gate.call_async(Trigger.Request())
            self.hold_future.add_done_callback(lambda f: self.hold_done(f, epoch))

    def hold_done(self, future, epoch):
        if epoch != self.joint_epoch or self.hold_future is not future:
            return
        try:
            result = future.result()
            if not result.success:
                self.stop_tracking('到位保持失败：' + result.message)
        except Exception as exc:
            self.stop_tracking('到位保持请求失败：' + str(exc))

    def tick(self):
        self.heartbeat.publish(Float64(data=time.time()))
        if not self.switched:
            if self.switch_future is None and self.switch.service_is_ready():
                self.switch_future = self.switch.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG))
            if self.switch_future is not None and self.switch_future.done():
                try:
                    self.switched = self.switch_future.result().success
                except Exception:
                    pass
                self.switch_future = None
            return
        command = np.zeros(6)
        try:
            if self.target is None:
                self.align()
                if self.valid:
                    self.reason = '待启动'
            q, actual = self.actual()
            if self.active or getattr(self, 'joint_busy', False) or getattr(self, 'plan_ready', False):
                if time.monotonic()-self.panel_seen > .6:
                    raise ValueError('RViz 面板连接中断')
                if self.dragging and time.monotonic()-self.drag_seen > 1.2:
                    raise ValueError('拖动操作连接中断')
            if getattr(self, 'joint_wait', None) is not None:
                self.joint_request_tick()
            if self.active and not getattr(self, 'joint_busy', False):
                if (getattr(self, 'resume_future', None) is not None and
                        time.monotonic()-getattr(self, 'resume_requested_at',
                                                 getattr(self, 'last_resume_attempt', 0.)) > 1.):
                    raise ValueError('恢复响应超时')
                if (getattr(self, 'hold_future', None) is not None and
                        time.monotonic()-self.hold_requested_at > 1.):
                    raise ValueError('保持响应超时')
                Backend.recovery_tick(self, q, actual)
                if (not getattr(self, 'joint_busy', False) and
                        (self.guard_state in ('paused', 'holding') or
                         (self.guard_state == 'joint_done' and getattr(self, 'recovery_hold', False)))):
                    self.maybe_resume(q, actual)
                if (self.approved_target is not None and self.guard_state == 'running' and
                        not getattr(self, 'motion_recovery_required', False)):
                    if self.verification_gap_since is not None and time.monotonic()-self.verification_gap_since > self.config['ik_timeout']+.05:
                        self.reject('等待目标验证，请继续调整目标')
                    else:
                        # Follow only the most recent *verified* pose while a newer candidate is checked.
                        # This avoids zero-command pulses on every 30–60 Hz mouse update.
                        self.maybe_hold(q, actual)
                        if self.hold_future is None:
                            command = Backend.control_twist(self, actual)
        except ValueError as exc:
            if self.active or getattr(self, 'joint_busy', False) or getattr(self, 'plan_ready', False):
                self.stop_tracking(str(exc))
            elif self.target is None:
                self.reason = str(exc)
        self.publish_twist(command)

    def publish_workspace(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.ns, marker.id = 'position_workspace_approximation', 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD if self.workspace_visible else Marker.DELETE
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, .85, .35, self.config['workspace_alpha']
        if self.workspace_visible:
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                             for p in self.workspace_triangles.reshape(-1, 3)]
            # RViz multiplies global and vertex alpha. Vertex alpha <1 enables blending.
            marker.colors = self.workspace_colors
        self.workspace_pub.publish(MarkerArray(markers=[marker]))

    def publish_status(self):
        error_mm = error_deg = condition = None
        singularity_level = 'unknown'
        feedback_bounds, feedback_error = [], ''
        feedback_accepted = False
        try:
            q, actual = self.raw_actual()
            feedback_bounds = self.model.joint_bound_violations(q)
            try:
                self.actual()
                feedback_accepted = True
            except ValueError as exc:
                feedback_error = str(exc)
            singularity_level = 'disabled'
            if self.target is not None:
                dp, dr = pose_error(self.target, actual)
                error_mm, error_deg = float(np.linalg.norm(dp)*1000), float(np.rad2deg(np.linalg.norm(dr)))
        except ValueError as exc:
            feedback_error = str(exc)
        moving = self.dragging or (error_mm is not None and
            (error_mm > self.config['position_tolerance']*1000 or
             error_deg > np.rad2deg(self.config['orientation_tolerance'])))
        state = ('跟踪中' if moving else '跟踪已开启 · 保持目标') if self.active and self.guard_state == 'running' else self.reason
        if self.active and self.guard_state == 'running' and getattr(self, 'hold_future', None) is not None:
            state = getattr(self, 'hold_reason', '') or self.reason
        if self.active and self.guard_state == 'paused':
            state = self.reason
        if self.active and self.guard_state == 'holding':
            state = self.reason if self.revision > self.pause_revision else (
                getattr(self, 'hold_reason', '') or
                ('已到位 · 保持目标' if self.valid else '目标不可用：' + self.reachability))
        if self.active and self.guard_state in ('pausing', 'syncing', 'resetting', 'opening'):
            state = '正在准备'
        measured_velocity = getattr(self.controller, 'measured_velocity', None)
        motion_known = isinstance(measured_velocity, np.ndarray) and measured_velocity.shape == (6,)
        braking = getattr(self.controller, 'settling_braking', False)
        self.status.publish(String(data=json.dumps(dict(mode=self.mode, active=self.active, state=state,
            control_mode=self.control_mode, busy=self.joint_busy, plan_ready=self.plan_ready, plan_id=self.joint_id,
            reachability=self.reachability, position_error_mm=error_mm, orientation_error_deg=error_deg,
            speed_scale=self.scale, workspace_visible=self.workspace_visible,
            linear_speed_limit_mps=self.config['linear_speed']*self.scale,
            angular_speed_limit_radps=self.config['angular_speed']*self.scale,
            joint_velocity_limits_radps=joint_speed_limits(self.config).tolist(),
            hold_position_tolerance_mm=self.config['hold_position_tolerance']*1000,
            hold_orientation_tolerance_deg=float(np.rad2deg(self.config['hold_orientation_tolerance'])),
            guard_state=self.guard_state, singularity_level=singularity_level,
            feedback_accepted=feedback_accepted, feedback_error=feedback_error,
            feedback_bounds=feedback_bounds,
            target_revision=self.revision, ghost_revision=self.ghost_revision,
            control_algorithm=self.config.get('control', {}).get('algorithm', 'p'),
            antiwindup_state=self.controller.antiwindup_state,
            settling_braking=bool(braking) if isinstance(braking, (bool, np.bool_)) else False,
            measured_tcp_linear_speed_mps=float(np.linalg.norm(measured_velocity[:3])) if motion_known else None,
            measured_tcp_angular_speed_radps=float(np.linalg.norm(measured_velocity[3:])) if motion_known else None,
            execution_feedback=getattr(getattr(self, 'execution_observer', None), 'reason', 'unavailable'),
            singularity_condition=condition), ensure_ascii=False)))
        # Colour the actual pickable sphere; no non-interactive sphere covering it.
        # Never replace controls during a drag: that would interrupt RViz mouse capture.
        if self.target is not None and not self.dragging:
            color = 'valid' if self.valid else ('checking' if self.reachability == '检查中' else 'invalid')
            if color != self.marker_color:
                marker = self.server.get('tcp_target')
                if marker is not None:
                    sphere = marker.controls[0].markers[0]
                    sphere.color.r = .1 if color == 'valid' else 1.
                    sphere.color.g = .85 if color == 'valid' else (.7 if color == 'checking' else .1)
                    sphere.color.b, sphere.color.a = .15, 1.
                    self.server.insert(marker)
                    self.server.applyChanges()
                    self.marker_color = color


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Backend()
    exiting = False
    def request_exit(signum, frame):
        nonlocal exiting
        exiting = True
    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)
    try:
        while rclpy.ok() and not exiting:
            rclpy.spin_once(node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_tracking('任务退出')
            until = time.monotonic()+.3
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
