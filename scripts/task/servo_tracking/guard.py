"""Independent watchdog and command gate; forwards, never interpolates joints."""
import json
import time
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String, Float64
from piper_servo_tracking.msg import GamepadInput
from std_srvs.srv import SetBool, Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import ServoStatus
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from .core import JOINTS, FreshJoints, RobotModel, load_config, validate_trajectory, validate_arm_mode, joint_speed_limits, HARDWARE_JOINT_SPEED_LIMITS
from .joint_motion import JointMotion, JOINT_STATES, JOINT_FORWARD_STATES
from .stop_observer import StopObserver


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


PREPARING_STATES = ('pausing', 'syncing', 'resetting', 'opening')
ACTIVE_STATES = PREPARING_STATES + ('running', 'paused', 'holding') + JOINT_STATES


def service_result(future):
    try:
        response = future.result()
        return bool(response.success), getattr(response, 'message', '')
    except Exception as exc:
        return False, str(exc)


class Guard(Node):
    def __init__(self):
        super().__init__('servo_guard')
        self.declare_parameter('config_file', '')
        self.declare_parameter('control_config_file', '')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('gamepad_mode', False)
        self.declare_parameter('gamepad_input_timeout_s', .20)
        self.declare_parameter('gamepad_gui_timeout_s', .30)
        self.declare_parameter('require_verified_gamepad_link', True)
        self.declare_parameter('require_verified_gamepad_mapping', True)
        self.declare_parameter('gamepad_stop_settle_window_s', .30)
        self.declare_parameter('gamepad_stop_joint_span_rad', .003)
        self.config = load_config(self.get_parameter('config_file').value,
                                  self.get_parameter('control_config_file').value)
        self.model = RobotModel(self.get_parameter('robot_description').value)
        self.real = self.get_parameter('mode').value == 'real'
        self.gamepad_mode = bool(self.get_parameter('gamepad_mode').value)
        self.gamepad_input_timeout = float(self.get_parameter('gamepad_input_timeout_s').value)
        self.gamepad_gui_timeout = float(self.get_parameter('gamepad_gui_timeout_s').value)
        self.require_verified_gamepad_link = bool(self.get_parameter('require_verified_gamepad_link').value)
        self.require_verified_gamepad_mapping = bool(self.get_parameter('require_verified_gamepad_mapping').value)
        self.gamepad_input = None
        self.gamepad_received = 0.
        self.gamepad_identity = None
        self.gamepad_sequence = -1
        self.gamepad_error = '等待手柄输入' if self.gamepad_mode else ''
        self.gui_heartbeat = 0.
        self.feedback, self.commanded = FreshJoints(), FreshJoints()
        self.state, self.reason = 'idle', '待启动'
        self.pause_cause = ''
        self.heartbeat = self.last_trajectory = 0.
        self.epoch = 0
        self.stage_time = self.stable_since = 0.
        self.stable_q = None
        self.sync_source_stamps = None
        self.sync_samples = 0
        self.diagnostics = {}
        self.closed = not self.real
        self.closing = None
        self.stop_future = None
        self.hold_pending = False
        self.stop_observer = StopObserver(
            float(self.get_parameter('gamepad_stop_settle_window_s').value) if self.gamepad_mode else self.config['hold_settle_time'],
            float(self.get_parameter('gamepad_stop_joint_span_rad').value) if self.gamepad_mode else self.config['hold_joint_stability'])
        self.stop_requested_at = 0.
        self.stop_generation = 0
        self.stop_has_been_requested = False
        self.stop_settled = False
        self.stop_confirmed = False
        self.stop_hold_required = False
        self.servo_code = None
        self.servo_received = 0.
        self.last_forwarded = None
        self.last_forwarded_time = 0.
        self.action_active = False
        self.output = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 1)
        # Only this watchdog forwards the mock controller's result to the official driver.
        self.driver_output = self.create_publisher(JointState, 'control/joint_states', 1) if self.real else None
        self.status = self.create_publisher(String, 'guard/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, 'feedback/joint_states' if self.real else 'commanded_joint_states', self.on_feedback, 1)
        self.create_subscription(JointState, 'commanded_joint_states', self.on_commanded, 1)
        self.create_subscription(JointTrajectory, 'servo/trajectory', self.on_trajectory, 1)
        self.create_subscription(Float64, 'backend/heartbeat', self.on_heartbeat, 1)
        if self.gamepad_mode:
            self.create_subscription(GamepadInput, 'gamepad/input', self.on_gamepad_input, 10)
            self.create_subscription(Float64, 'gamepad/ui_heartbeat', self.on_gamepad_ui_heartbeat, 1)
        self.create_subscription(ServoStatus, 'servo_node/status', self.on_servo, 1)
        self.cancel_action = self.create_client(CancelGoal, 'arm_controller/follow_joint_trajectory/_action/cancel_goal')
        self.create_subscription(GoalStatusArray, 'arm_controller/follow_joint_trajectory/_action/status',
                                 self.on_action_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.real:
            for kind in ('motor', 'arm'):
                self.create_subscription(DiagnosticArray, f'feedback/{kind}_status',
                                         lambda msg, k=kind: self.on_diagnostic(k, msg), 1)
        self.gate = self.create_client(SetBool, 'control_enable') if self.real else None
        self.hold = self.create_client(Empty, 'emergency_stop') if self.real else None
        self.pause_servo = self.create_client(SetBool, 'servo_node/pause_servo')
        self.joint = JointMotion(self)
        self.create_service(SetBool, 'guard/enable', self.enable)
        self.create_service(Trigger, 'guard/hold', self.hold_service)
        self.create_timer(.02, self.tick)
        self.create_timer(.1, self.publish_status)

    def on_heartbeat(self, msg):
        if 0 <= time.time()-msg.data < .3:
            self.heartbeat = time.monotonic()

    def on_gamepad_ui_heartbeat(self, msg):
        if 0 <= time.time()-msg.data < self.gamepad_gui_timeout:
            self.gui_heartbeat = time.monotonic()

    def on_gamepad_input(self, msg):
        now = time.monotonic()
        try:
            axes = np.asarray(msg.axes, dtype=float)
            if (not msg.guid or int(msg.generation) < 1 or len(msg.axes) != 4 or len(msg.dpad) != 4 or
                    not np.isfinite(axes).all() or np.any(axes < -1.001) or np.any(axes > 1.001) or
                    any(v not in (0, 1) for v in msg.dpad)):
                raise ValueError('手柄输入格式或数值无效')
            identity = (msg.guid, int(msg.generation))
            if self.gamepad_identity is not None and identity != self.gamepad_identity:
                self.gamepad_identity = identity
                self.gamepad_sequence = -1
                self.gamepad_input = None
                self.gamepad_received = 0.
                self.gamepad_error = '手柄连接代数变化；等待新鲜输入'
                if self.state in ACTIVE_STATES:
                    self.stop(self.gamepad_error, fault=True)
                return
            if self.gamepad_identity is None:
                self.gamepad_identity = identity
            if int(msg.sequence) <= self.gamepad_sequence:
                raise ValueError('手柄输入序号重复或倒退')
            if not msg.connected:
                raise ValueError('手柄设备已断开')
            if self.real and ((self.require_verified_gamepad_link and not msg.link_verified) or
                              (self.require_verified_gamepad_mapping and not msg.mapping_verified) or msg.source == 'fixture'):
                raise ValueError('该连接方式/轴映射尚未通过实机验证')
            self.gamepad_sequence = int(msg.sequence)
            self.gamepad_input, self.gamepad_received, self.gamepad_error = msg, now, ''
        except (ValueError, TypeError) as exc:
            self.gamepad_input = None
            self.gamepad_received = 0.
            self.gamepad_error = str(exc)
            if self.state in ACTIVE_STATES:
                self.stop(self.gamepad_error, fault=True)

    def gamepad_authority(self, require_lb=True):
        if not self.gamepad_mode:
            return
        msg = self.gamepad_input
        if msg is None or time.monotonic()-self.gamepad_received > self.gamepad_input_timeout:
            raise ValueError('手柄输入租约已过期：'+(self.gamepad_error or '输入超时'))
        if self.real and ((self.require_verified_gamepad_link and not msg.link_verified) or
                          (self.require_verified_gamepad_mapping and not msg.mapping_verified) or msg.source == 'fixture'):
            raise ValueError('该连接方式/轴映射尚未通过实机验证')
        if time.monotonic()-self.gui_heartbeat > self.gamepad_gui_timeout:
            raise ValueError('Qt 界面心跳超时')
        if require_lb and (not msg.lb or msg.b):
            raise ValueError('LB 持续许可已撤销或 B 已停止')
        if self.count_publishers('gamepad/input') != 1:
            raise ValueError('检测到多个手柄输入来源')

    def on_action_status(self, msg):
        joint = getattr(self, 'joint', None)
        if joint is not None:
            joint.observe_status(msg)
        self.action_active = any(s.status in (1, 2, 3) and
                                 (joint is None or not joint.owns_action(s.goal_info.goal_id))
                                 for s in msg.status_list)
        if self.action_active:
            if self.cancel_action.service_is_ready():
                self.cancel_action.call_async(CancelGoal.Request())
            if self.state in ACTIVE_STATES:
                self.stop('检测到其他轨迹任务', fault=True)

    def on_servo(self, msg):
        self.servo_code, self.servo_received = int(msg.code), time.monotonic()
        if self.state in JOINT_STATES:
            # Servo is deliberately paused; its stale Cartesian status cannot
            # authorize or invalidate the independently validated joint action.
            return
        if self.state == 'running' and msg.code in (2, 6):
            # Code 2 cannot be produced by the configured native JOINT_JOG
            # path. Fail closed if an unexpected Cartesian mode is selected;
            # never reinterpret its halted output as an executable command.
            label = '运动学约束暂停' if msg.code == 2 else '关节边界暂停'
            self.stop(label + '：' + msg.message, paused=True,
                      pause_cause='singularity' if msg.code == 2 else 'joint_bound')
        elif self.state in ACTIVE_STATES and msg.code in (-1, 5):
            self.stop('Servo 停止：' + msg.message, fault=True)

    def on_diagnostic(self, kind, msg):
        self.diagnostics[kind] = (time.monotonic(), msg)

    def health(self):
        q = self.feedback.get(self.config['feedback_timeout'])
        self.model.check_feedback(q, zero_tolerance=(
            self.config.get('zero_feedback_tolerance', 0.) if self.real else 0.))
        if self.real:
            for kind in ('motor', 'arm'):
                if kind not in self.diagnostics:
                    raise ValueError('等待实机状态反馈')
                received, msg = self.diagnostics[kind]
                if time.monotonic()-received > 1.5 or not -.1 <= time.time()-stamp_seconds(msg.header.stamp) <= 1.5:
                    raise ValueError('实机诊断反馈超时')
                statuses = {s.name: s for s in msg.status}
                for name in JOINTS if kind == 'motor' else ['arm']:
                    s = statuses.get(name)
                    if s is None or s.level != s.OK:
                        raise ValueError(f'{name} 状态异常')
                    if kind == 'motor' and {v.key: v.value for v in s.values}.get('enabled') != 'true':
                        raise ValueError('实机未使能，请使用官方使能服务')
                    if kind == 'arm':
                        validate_arm_mode({v.key: v.value for v in s.values})
        return q

    def on_feedback(self, msg):
        previous = self.feedback.stamp
        self.feedback.update(msg.name, msg.position, stamp_seconds(msg.header.stamp))
        if (self.stop_has_been_requested and self.feedback.stamp > previous and
                not self.feedback.error):
            try:
                q = self.feedback.get(self.config['feedback_timeout'])
                self.stop_observer.observe(self.feedback.stamp, time.monotonic(), q)
            except ValueError:
                pass

    def planning_start(self):
        raw = self.health()
        tolerance = self.config.get('zero_feedback_tolerance', 0.) if self.real else 0.
        reference = self.model.planning_joints(raw, zero_tolerance=tolerance)
        if np.any(reference != raw):
            self.get_logger().warning('零位边界规划参考规范化（非实测关节角）; ' + json.dumps({
                'raw_joints': raw.tolist(), 'planning_reference': reference.tolist(),
                'delta_rad': (reference-raw).tolist(), 'feedback_stamp': self.feedback.stamp,
                'feedback_source_age': time.time()-self.feedback.stamp,
                'feedback_received_age': time.monotonic()-self.feedback.received,
            }, ensure_ascii=False, allow_nan=False))
        return reference

    def on_commanded(self, msg):
        self.commanded.update(msg.name, msg.position, stamp_seconds(msg.header.stamp))
        if not self.real or self.state not in ('running',) + JOINT_FORWARD_STATES:
            return
        try:
            actual = self.health()
            q = self.commanded.get(self.config['feedback_timeout'])
            try:
                self.model.check_joints(q)
            except ValueError:
                joint = getattr(self, 'joint', None)
                tolerance = float(self.config.get('zero_feedback_tolerance', 0.))
                if (self.last_forwarded is None and self.state == 'joint_executing'
                        and joint is not None and joint.startup_hold_window(time.monotonic())):
                    # JTC can publish the old internal raw-feedback hold before
                    # processing our strict plan's first knot. Never forward it.
                    self.model.check_feedback(q, zero_tolerance=tolerance)
                    if np.all(np.abs(q-actual) <= tolerance+1e-12):
                        return
                raise
            now = time.monotonic()
            if self.last_forwarded is not None:
                dt = min(now-self.last_forwarded_time, .05)
                limits = np.minimum(joint_speed_limits(self.config)*1.5, HARDWARE_JOINT_SPEED_LIMITS)
                if np.any(np.abs(q-self.last_forwarded) > limits * dt + .001):
                    raise ValueError('控制器关节指令跳变')
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name, out.position = JOINTS, q.tolist()
            self.driver_output.publish(out)
            self.last_forwarded, self.last_forwarded_time = q, now
        except ValueError as exc:
            self.stop(str(exc), fault=True)

    def on_trajectory(self, msg):
        if self.state != 'running':
            return
        try:
            actual = self.health()
            validate_trajectory(msg, actual, self.model, self.config, time.time())
            self.output.publish(msg)
            self.last_trajectory = time.monotonic()
        except ValueError as exc:
            self.stop(str(exc), fault=True)

    def controller_hold(self, q):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions, point.velocities = q.tolist(), [0.] * 6
        point.time_from_start.nanosec = 20_000_000
        msg.points = [point]
        # Zero header means start immediately; this command bypasses Servo only to hold/reset.
        self.output.publish(msg)

    def condition_snapshot(self, q=None):
        """Compatibility logger for measured joints; it performs no κ calculation."""
        result = {'joints': None}
        try:
            q = self.feedback.get(self.config['feedback_timeout']) if q is None else q
            result['joints'] = [float(v) if np.isfinite(v) else None for v in q]
        except (ValueError, TypeError, AttributeError):
            pass
        return json.dumps(result, ensure_ascii=False, allow_nan=False)

    def request_pause(self):
        # A pause is safe even after stop; it never opens the real control gate.
        if self.pause_servo.service_is_ready():
            return self.pause_servo.call_async(SetBool.Request(data=True))
        return None

    def preparation_health(self):
        # Service replies may arrive between watchdog ticks; recheck authority
        # before any reply can advance toward opening the real command gate.
        q = self.health()
        self.gamepad_authority(require_lb=True)
        # Kinematic velocity inversion is regularized before native JointJog;
        # this gate validates authority and feedback without a κ threshold.
        if getattr(self, 'joint', None) is not None and self.joint.has_unresolved_actions():
            raise ValueError('等待旧关节动作终态确认')
        if time.monotonic()-self.heartbeat > .4:
            raise ValueError('控制任务心跳超时')
        if self.action_active:
            raise ValueError('检测到其他轨迹任务')
        if self.count_publishers('arm_controller/joint_trajectory') != 1:
            raise ValueError('检测到其他轨迹控制来源')
        if self.real and self.count_publishers('control/joint_states') != 1:
            raise ValueError('检测到其他实机控制来源')
        if self.gamepad_mode and self.count_publishers('servo_node/delta_joint_cmds') != 1:
            raise ValueError('检测到其他 Servo JointJog 输入来源')
        return q

    def stop(self, reason, fault=False, paused=False, holding=False, pause_cause=''):
        was_active = self.state in ACTIVE_STATES
        self.epoch += 1
        self.state, self.reason = ('fault' if fault else 'paused' if paused else
                                   'holding' if holding else 'idle'), reason
        self.pause_cause = pause_cause if self.state == 'paused' else ''
        self.stop_requested_at = time.monotonic()
        self.stop_generation += 1
        self.stop_has_been_requested = True
        self.stop_settled = self.stop_confirmed = False
        self.stop_hold_required = bool(self.real and was_active)
        self.stop_observer.reset()
        if getattr(self, 'joint', None) is not None:
            self.joint.invalidate()
        self.last_forwarded = None
        self.stable_since, self.stable_q = 0., None
        self.get_logger().warning(f'停止跟踪 [{self.state}] {reason}; {self.condition_snapshot()}')
        self.request_pause()
        try:
            self.controller_hold(self.feedback.get(self.config['feedback_timeout']))
        except ValueError:
            # With stale real feedback, hold the mock at its own last position; never send it to hardware.
            try:
                self.controller_hold(self.commanded.get(self.config['feedback_timeout']))
            except ValueError:
                pass
        if self.real:
            self.closed = False
            if self.gate.service_is_ready():
                self.closing = self.gate.call_async(SetBool.Request(data=False))
            if was_active:
                self.hold_pending = True
                self.stop_future = None
            if was_active and self.hold.service_is_ready():
                # Official service requests holding current measured joints. It is not a certified E-stop.
                self.stop_future = self.hold.call_async(Empty.Request())

    def hold_service(self, request, response):
        """Latch a settled session with forwarding closed; only enable can resume."""
        try:
            if self.state != 'running':
                raise ValueError('仅跟踪运行中可进入到位保持')
            self.preparation_health()
            self.stop('已到位，保持中', holding=True)
            response.success, response.message = True, self.reason
        except ValueError as exc:
            response.success, response.message = False, str(exc)
            if self.state == 'running':
                self.stop(str(exc), fault=True)
        return response

    def enable(self, req, response):
        if not req.data:
            self.stop('已停止')
            response.success, response.message = True, self.reason
            return response
        try:
            if self.gamepad_mode and self.stop_has_been_requested and not self.stop_confirmed:
                raise ValueError('等待新鲜实测关节反馈确认停稳')
            if self.state in PREPARING_STATES + ('running',) + JOINT_STATES:
                raise ValueError('跟踪已启动')
            if self.action_active:
                raise ValueError('等待其他轨迹任务取消')
            if time.monotonic()-self.heartbeat > .35:
                raise ValueError('控制任务未连接')
            if (not self.closed or (self.hold_pending and self.stop_future is None)
                    or (self.stop_future is not None and not self.stop_future.done())):
                raise ValueError('等待停止服务确认')
            if self.stop_future is not None and self.stop_future.exception() is not None:
                raise ValueError('实机保持服务失败，请检查驱动后重新启动任务')
            actual = self.preparation_health()
            if self.output.get_subscription_count() != 1:
                raise ValueError(f'关节控制器未就绪：订阅数={self.output.get_subscription_count()}')
            if not self.pause_servo.service_is_ready():
                raise ValueError('Servo 暂停/重置服务不可用')
            if self.real and (not self.gate.service_is_ready() or not self.hold.service_is_ready()):
                raise ValueError('实机停止/控制服务不可用')
            if getattr(self, 'joint', None) is not None:
                self.joint.invalidate()
                self.epoch += 1
            self.stop_has_been_requested = False
            self.stop_settled = self.stop_confirmed = False
            self.stop_observer.reset()
            self.state, self.reason = 'pausing', '暂停 Servo 准备对齐'
            self.stage_time = time.monotonic()
            self.stable_since, self.stable_q = 0., None
            self.sync_source_stamps, self.sync_samples = None, 0
            self.get_logger().info('准备开启跟踪; ' + self.condition_snapshot(actual))
            future = self.pause_servo.call_async(SetBool.Request(data=True))
            future.add_done_callback(lambda f, e=self.epoch: self.paused_servo(f, e))
            response.success, response.message = True, '正在准备'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def paused_servo(self, future, epoch):
        if epoch != self.epoch or self.state != 'pausing':
            return
        success, message = service_result(future)
        if not success:
            self.stop('Servo 暂停失败' + (': ' + message if message else ''), fault=True)
            return
        try:
            actual = self.preparation_health()
            self.controller_hold(actual)
            self.state, self.reason = 'syncing', '对齐控制器'
            self.stage_time = time.monotonic()
            self.stable_since, self.stable_q = 0., actual
            self.sync_source_stamps = (self.feedback.stamp, self.commanded.stamp)
            self.sync_samples = 0
        except ValueError as exc:
            self.stop(str(exc), fault=True)

    def reset_servo(self, future, epoch):
        if epoch != self.epoch or self.state != 'resetting':
            # The server may already have applied this old unpause request.
            # Reassert pause, and revoke any newer session rather than guessing its order.
            if self.state in PREPARING_STATES + ('running',) + JOINT_STATES:
                self.stop('Servo 重置响应已过期，请重新开启跟踪', fault=True)
            else:
                self.request_pause()
            return
        success, message = service_result(future)
        if not success:
            self.stop('Servo 重置失败' + (': ' + message if message else ''), fault=True)
            return
        try:
            q = self.preparation_health()
            if np.max(np.abs(self.commanded.get(self.config['feedback_timeout'])-q)) > .003:
                raise ValueError('Servo 重置期间关节位置变化，请重新开启跟踪')
            if self.real:
                if not self.closed or not self.gate.service_is_ready():
                    raise ValueError('实机控制门未确认关闭')
                self.state, self.reason = 'opening', '开启控制门'
                self.stage_time = time.monotonic()
                self.closed = False
                opened = self.gate.call_async(SetBool.Request(data=True))
                opened.add_done_callback(lambda f, e=self.epoch: self.opened(f, e))
            else:
                self.enter_running(q)
        except ValueError as exc:
            self.stop(str(exc), fault=True)

    def enter_running(self, q=None):
        q = self.preparation_health()
        self.state, self.reason = 'running', '跟踪已开启'
        self.last_trajectory = time.monotonic()
        # MoveIt Servo may not publish status while paused. Start its freshness
        # lease when the session opens, then require a real status within 0.5 s.
        self.servo_received = time.monotonic()
        self.get_logger().info('跟踪已开启; ' + self.condition_snapshot(q))

    def opened(self, future, epoch):
        if epoch != self.epoch or self.state != 'opening':
            if self.state in PREPARING_STATES + ('running',) + JOINT_STATES:
                self.stop('控制门响应已过期，请重新开启跟踪', fault=True)
            elif self.real:
                self.closed = False
                self.closing = self.gate.call_async(SetBool.Request(data=False))
            return
        success, message = service_result(future)
        if not success:
            self.stop('控制门开启失败' + (': ' + message if message else ''), fault=True)
            return
        try:
            self.enter_running()
        except ValueError as exc:
            self.stop(str(exc), fault=True)

    def tick(self):
        now = time.monotonic()
        closed_states = ('idle', 'fault', 'paused', 'holding', 'joint_done', 'joint_finishing')
        if self.real and self.hold_pending and self.state in closed_states:
            if self.stop_future is None and self.hold.service_is_ready():
                self.stop_future = self.hold.call_async(Empty.Request())
            elif self.stop_future is not None and self.stop_future.done():
                if self.stop_future.exception() is None:
                    self.hold_pending = False
        if self.real and self.closing is not None and self.closing.done():
            try:
                self.closed = self.closing.result().success
            except Exception:
                self.closed = False
            self.closing = None
        if self.real and self.state in closed_states and not self.closed and self.closing is None:
            if self.gate.service_is_ready():
                self.closing = self.gate.call_async(SetBool.Request(data=False))
        if self.stop_has_been_requested:
            self.stop_settled = self.stop_observer.confirmed(now, self.config['feedback_timeout'])
            hold_done = (not self.stop_hold_required or
                         (not self.hold_pending and self.stop_future is not None and
                          self.stop_future.done() and self.stop_future.exception() is None))
            gate_done = not self.real or (self.closed and self.closing is None)
            self.stop_confirmed = bool(self.stop_settled and hold_done and gate_done)
        if (getattr(self, 'joint', None) is not None and self.joint.has_unresolved_actions()
                and self.state in closed_states):
            # A send-goal response can arrive after the first stop hold. Keep
            # replacing the mock trajectory until that old action is terminal.
            # Real forwarding stays closed throughout this retirement barrier.
            try:
                self.controller_hold(self.feedback.get(self.config['feedback_timeout']))
            except ValueError:
                pass
        if self.state not in ACTIVE_STATES:
            return
        try:
            if self.gamepad_mode and self.state in PREPARING_STATES + ('running',):
                sample = self.gamepad_input
                if sample is not None and sample.b:
                    self.stop('手柄 B 停止锁定', fault=True)
                    return
                if sample is not None and not sample.lb:
                    self.stop('已松开 LB，撤销运动许可', paused=True)
                    return
                self.gamepad_authority(require_lb=True)
            q = self.health()
            if now-self.heartbeat > .4:
                raise ValueError('控制任务心跳超时')
            if self.count_publishers('arm_controller/joint_trajectory') != 1:
                raise ValueError('检测到其他轨迹控制来源')
            if self.real and self.count_publishers('control/joint_states') != 1:
                raise ValueError('检测到其他实机控制来源')
            if self.state in JOINT_STATES:
                self.joint.tick(q, now)
                return
            if self.state in ('paused', 'holding'):
                # A valid Servo status must never restart a paused/held session.
                return
            if self.state in PREPARING_STATES:
                if now-self.stage_time > 3.:
                    raise ValueError('Servo 准备阶段超时：' + self.state)
                if self.state == 'syncing':
                    commanded = self.commanded.get(self.config['feedback_timeout'])
                    stamps = (self.feedback.stamp, self.commanded.stamp)
                    if not all(isinstance(stamp, (float, int)) and np.isfinite(stamp) and stamp > 0
                               for stamp in stamps):
                        raise ValueError('控制器同步反馈时间戳无效')
                    if np.max(np.abs(commanded-q)) > .003 or np.max(np.abs(q-self.stable_q)) > .002:
                        self.controller_hold(q)
                        self.stable_since, self.stable_q = 0., q
                        self.sync_source_stamps, self.sync_samples = stamps, 0
                    else:
                        previous = self.sync_source_stamps
                        if previous is None or all(new > old for new, old in zip(stamps, previous)):
                            self.sync_source_stamps = stamps
                            self.sync_samples += 1
                        if not self.stable_since:
                            self.stable_since = now
                        elif (self.sync_samples >= 2 and now-self.stable_since+1e-9 >=
                              self.config.get('controller_sync_settle_time', .08)):
                            if not self.pause_servo.service_is_ready():
                                raise ValueError('Servo 暂停/重置服务不可用')
                            # The official unpause service clears Servo's rolling window.
                            self.state, self.reason = 'resetting', '重置 Servo 状态'
                            self.stage_time = now
                            future = self.pause_servo.call_async(SetBool.Request(data=False))
                            future.add_done_callback(lambda f, e=self.epoch: self.reset_servo(f, e))
            elif now-self.last_trajectory > .35:
                raise ValueError('Servo 输出中断')
            elif now-self.servo_received > .5:
                raise ValueError('Servo 状态超时')
        except (ValueError, RuntimeError) as exc:
            self.stop(str(exc), fault=True)

    def publish_status(self):
        status = {'state': self.state, 'reason': self.reason,
                  'pause_cause': getattr(self, 'pause_cause', '') if self.state == 'paused' else '',
                  'gate_closed': self.closed, 'mode': 'real' if self.real else 'sim',
                  'other_action_active': self.action_active,
                  'gamepad_mode': self.gamepad_mode,
                  'input_age_s': None if not self.gamepad_received else time.monotonic()-self.gamepad_received,
                  'input_error': self.gamepad_error,
                  'stop_settled': self.stop_settled,
                  'stop_confirmed': self.stop_confirmed,
                  'stop_generation': self.stop_generation,
                  'stop_requested_at': self.stop_requested_at,
                  'stop_sample_count': len(self.stop_observer.samples)}
        status.update(self.joint.status())
        self.status.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    # Keep ROS alive long enough to send the final hold even on repeated launch signals.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Guard()
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
            node.stop('任务退出')
            until = time.monotonic() + .5
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
        node.destroy_node()
        node.joint.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
