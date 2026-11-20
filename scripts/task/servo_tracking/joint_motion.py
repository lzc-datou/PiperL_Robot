"""Guard-owned MoveIt joint plans executed by the official JTC action server.

There is no joint interpolation here. MoveIt plans and time-parameterizes; JTC
samples the returned trajectory, and Guard checks/forwards its measured output.
"""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import math
import time
import uuid
import numpy as np
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
from .core import JOINTS


JOINT_BUSY_STATES = ('joint_pausing', 'joint_planning', 'joint_syncing',
                     'joint_validating', 'joint_opening', 'joint_executing',
                     'joint_settling', 'joint_finishing')
JOINT_STATES = JOINT_BUSY_STATES + ('joint_ready',)
JOINT_FORWARD_STATES = ('joint_executing', 'joint_settling')
# Only explicit kinematic cap failures may request a slower *official* plan.
# Format, timestamps, positions, endpoint and stop-velocity errors stay fatal.
REPLAN_LIMIT_ERRORS = frozenset((
    '关节规划轨迹点速度越限', '关节规划轨迹点加速度越限',
    '关节规划相邻点隐含速度越限', '关节规划多项式速度越限',
    '关节规划多项式加速度越限'))


class HomeCanceled(ValueError):
    """The operator released the home gesture; this is not a device fault."""


def validate_plan_math(trajectory, actual, goal, model, config, scale):
    """Worker entry point: pure trajectory mathematics, never ROS/Node I/O."""
    from .core import validate_joint_plan
    return validate_joint_plan(trajectory, actual, goal, model, config, scale)


def joint_plan_request(start, goal, planning_time, velocity_scale, acceleration_scale):
    request = GetMotionPlan.Request()
    motion = request.motion_plan_request
    motion.group_name, motion.pipeline_id = 'arm', 'ompl'
    motion.start_state.joint_state.name = JOINTS
    motion.start_state.joint_state.position = start.tolist()
    motion.start_state.is_diff = False
    constraints = Constraints()
    constraints.joint_constraints = [JointConstraint(
        joint_name=name, position=float(value), tolerance_above=1e-4,
        tolerance_below=1e-4, weight=1.) for name, value in zip(JOINTS, goal)]
    motion.goal_constraints = [constraints]
    motion.max_velocity_scaling_factor = velocity_scale
    motion.max_acceleration_scaling_factor = acceleration_scale
    motion.allowed_planning_time = planning_time
    motion.num_planning_attempts = 3
    return request


class JointMotion:
    def __init__(self, guard):
        from piper_servo_tracking.srv import PlanJointTarget, JointPlanCommand
        self.guard = guard
        self.planner = guard.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.controller = ActionClient(guard, FollowJointTrajectory,
                                       'arm_controller/follow_joint_trajectory')
        if not getattr(guard, 'visual_mode', False):
            guard.create_service(PlanJointTarget, 'guard/plan_joint_target', self.plan)
            guard.create_service(JointPlanCommand, 'guard/joint_plan_command', self.command)
        self.plan_id, self.purpose = '', ''
        self.target_revision = 0
        self.ready = False
        self.trajectory = self.start_q = self.goal_q = None
        self.duration = self.scale = self.planned_at = 0.
        self.auto_execute = False
        self.replans, self.planning_headroom, self.acceleration_headroom = 0, .8, .25
        self.plan_deadline = 0.
        self.pending_pause = self.pending_plan = self.pending_goal = None
        self.pending_validation = None
        self.validation_epoch = 0
        self.validation_plan_id = ''
        self.validated_identity = self.validated_trajectory = None
        self.validator = ThreadPoolExecutor(max_workers=1, thread_name_prefix='joint_plan_math')
        self.goal_handle = self.result_future = None
        self.owned_uuid = None
        # Keep actions across stop until an actual terminal result/status arrives.
        # A cancel-service acknowledgement alone is not terminal confirmation.
        self.actions = {}
        # Execution permission and action ownership have different lifetimes.
        # Retain just the UUIDs for this Guard's lifetime: result/status/goal
        # replies travel independently and old status snapshots may arrive after
        # invalidate() or even after a terminal result. They are still ours.
        self.issued_action_ids = set()
        self.execution_deadline = self.settle_since = 0.
        self.execution_epoch = -1
        self.settle_q = None

    def busy(self):
        return self.guard.state in JOINT_BUSY_STATES or self.has_unresolved_actions()

    def can_plan(self):
        g = self.guard
        if g.state not in ('idle', 'fault', 'paused', 'holding', 'joint_done', 'joint_ready'):
            return False
        if not g.closed or g.hold_pending or g.action_active or self.has_unresolved_actions():
            return False
        if g.stop_future is not None:
            if not g.stop_future.done():
                return False
            try:
                if g.stop_future.exception() is not None:
                    return False
            except Exception:
                return False
        return True

    def status(self):
        return dict(plan_id=self.plan_id, plan_ready=self.ready,
                    purpose=self.purpose, target_revision=self.target_revision,
                    joint_busy=self.busy(), can_plan=self.can_plan(),
                    pending_joint_actions=len(self.actions))

    def owns_action(self, goal_id):
        return bytes(goal_id.uuid) in self.issued_action_ids

    def startup_hold_window(self, now):
        return (self.guard.state == 'joint_executing' and self.execution_epoch == self.guard.epoch
                and self.owned_uuid is not None and 0 <= now-self.guard.stage_time <= .10
                and (self.pending_goal is not None or self.goal_handle is not None))

    def has_unresolved_actions(self):
        return bool(self.actions)

    def observe_status(self, message):
        for status in message.status_list:
            if status.status in (4, 5, 6):
                self.actions.pop(bytes(status.goal_info.goal_id.uuid), None)

    def terminal(self, future, action_id):
        try:
            if future.result().status in (4, 5, 6):
                self.actions.pop(action_id, None)
        except Exception as exc:
            # Transport failure does not prove the controller stopped this action.
            self.guard.get_logger().warning('关节动作终态尚未确认：' + str(exc))

    def invalidate(self):
        """Cancel only our action; a late accepted goal is canceled by its callback."""
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as exc:
                self.guard.get_logger().warning('关节动作取消请求失败：' + str(exc))
        for future in (self.pending_pause, self.pending_plan, self.pending_validation):
            if future is not None and not future.done():
                future.cancel()
        self.plan_id, self.purpose, self.target_revision = '', '', 0
        self.ready = False
        self.trajectory = self.start_q = self.goal_q = None
        self.pending_pause = self.pending_plan = self.pending_goal = None
        self.pending_validation = None
        self.validated_identity = self.validated_trajectory = None
        self.goal_handle = self.result_future = None
        self.owned_uuid = None
        self.execution_epoch = -1
        self.settle_since, self.settle_q = 0., None

    def shutdown(self):
        self.validator.shutdown(wait=False, cancel_futures=True)

    def current(self, epoch, plan_id, *states):
        return (epoch == self.guard.epoch and plan_id == self.plan_id
                and self.guard.state in states)

    def phase(self, state, reason):
        self.guard.state, self.guard.reason = state, reason
        self.guard.stage_time = time.monotonic()

    def fail(self, reason, cause=None):
        if isinstance(cause, HomeCanceled):
            self.guard.stop(reason, paused=True, pause_cause='gamepad_home_canceled')
        else:
            self.guard.stop(reason, fault=True)

    def plan(self, request, response):
        g = self.guard
        response.success, response.plan_id = False, ''
        try:
            if not self.can_plan():
                raise ValueError('请先停止跟踪并等待控制门和保持服务确认')
            if request.purpose not in ('ready', 'point', 'limit_recovery', 'home'):
                raise ValueError('未知关节规划用途')
            if g.gamepad_mode:
                if request.purpose != 'home':
                    raise ValueError('手柄会话只允许显式回零关节计划')
                if int(request.guard_epoch) != g.epoch:
                    raise ValueError('回零许可已过期；请松开 LB/Y 后重新请求')
                g.gamepad_authority(require_home=True)
            if bool(request.auto_execute) != (request.purpose in ('ready', 'limit_recovery', 'home')):
                raise ValueError('准备或关节调整自动执行，单点计划须单独确认执行')
            scale = float(request.speed_scale)
            if not math.isfinite(scale) or not .1 <= scale <= 1.:
                raise ValueError('速度比例必须在 10%～100% 之间')
            goal = g.model.check_joints(request.joints,
                                        0. if request.purpose == 'home' else g.config['joint_margin'])
            if request.purpose == 'home' and np.max(np.abs(goal)) > 1e-12:
                raise ValueError('回零目标必须为六轴 0°')
            g.preparation_health()
            actual = g.planning_start()
            if g.output.get_subscription_count() != 1:
                raise ValueError('关节控制器未就绪')
            if not self.planner.service_is_ready() or not self.controller.server_is_ready():
                raise ValueError('MoveIt 规划或关节执行服务不可用')
            if not g.pause_servo.service_is_ready():
                raise ValueError('Servo 暂停服务不可用')
            if g.real and (not g.gate.service_is_ready() or not g.hold.service_is_ready()):
                raise ValueError('实机停止/控制服务不可用')
            self.invalidate()
            g.epoch += 1
            self.plan_id, self.purpose = uuid.uuid4().hex, request.purpose
            self.target_revision = int(request.target_revision)
            self.start_q, self.goal_q = actual.copy(), np.asarray(goal).copy()
            self.scale, self.auto_execute = scale, bool(request.auto_execute)
            self.replans, self.planning_headroom, self.acceleration_headroom = 0, .8, .25
            budget = 3. * (float(g.config.get('planning_time', 5.)) +
                           float(g.config.get('plan_validation_timeout', 5.)) + 2.)
            self.plan_deadline = time.monotonic()+budget
            self.phase('joint_pausing', '暂停 Servo，准备关节规划')
            g.get_logger().info(f'关节规划 {self.plan_id} purpose={self.purpose} '
                                f'revision={self.target_revision}; {g.condition_snapshot(actual)}')
            epoch, plan_id = g.epoch, self.plan_id
            self.pending_pause = g.pause_servo.call_async(SetBool.Request(data=True))
            self.pending_pause.add_done_callback(lambda f: self.paused(f, epoch, plan_id))
            response.success, response.message, response.plan_id = True, '正在规划', self.plan_id
        except (ValueError, TypeError, IndexError) as exc:
            response.message = str(exc)
        return response

    def paused(self, future, epoch, plan_id):
        if not self.current(epoch, plan_id, 'joint_pausing'):
            return
        self.pending_pause = None
        try:
            if not future.result().success:
                raise ValueError('Servo 暂停失败')
            q = self.guard.preparation_health()
            if np.max(np.abs(q-self.start_q)) > .005:
                raise ValueError('规划起点已变化，请重新规划')
            self.request_plan(epoch, plan_id)
        except Exception as exc:
            self.fail('关节规划准备失败：' + str(exc), exc)

    def request_plan(self, epoch, plan_id):
        g = self.guard
        q = g.preparation_health()
        if not g.closed or np.max(np.abs(q-self.start_q)) > .005:
            raise ValueError('关节规划起点或控制门状态已变化，请重新规划')
        remaining = self.plan_deadline-time.monotonic()
        if remaining <= 0:
            raise ValueError('关节路径规划和检查累计超时')
        # Leave headroom for JTC quintic extrema between MoveIt/TOTG knots.
        # Validation still uses the operator's full cap; do not alter the plan.
        # TOTG can leave a very short final segment whose JTC acceleration peaks
        # near three times its supplied knot acceleration. Reserve that margin
        # in the official planner, while retaining the strict operator cap below.
        request = joint_plan_request(self.start_q, self.goal_q,
            min(float(g.config.get('planning_time', 5.)), remaining),
            self.scale * self.planning_headroom, self.scale * self.acceleration_headroom)
        self.phase('joint_planning', 'MoveIt 正在规划关节路径' if not self.replans else
                   '重新规划更平缓路径')
        self.pending_plan = self.planner.call_async(request)
        self.pending_plan.add_done_callback(lambda f: self.planned(f, epoch, plan_id))

    def planned(self, future, epoch, plan_id):
        if not self.current(epoch, plan_id, 'joint_planning') or self.pending_plan is not future:
            return
        self.pending_plan = None
        try:
            result = future.result().motion_plan_response
            if result.error_code.val != 1:
                raise ValueError(f'MoveIt 规划失败（{result.error_code.val}）')
            if result.trajectory.multi_dof_joint_trajectory.points:
                raise ValueError('关节规划包含不支持的浮动关节')
            q = self.guard.preparation_health()
            if np.max(np.abs(q-self.start_q)) > .005:
                raise ValueError('规划期间起点已变化，请重新规划')
            trajectory = deepcopy(result.trajectory.joint_trajectory)
            if trajectory.points:
                first, last = trajectory.points[0], trajectory.points[-1]
                self.guard.get_logger().info(
                    f'MoveIt 轨迹端点 {plan_id}: '
                    f'first t={first.time_from_start.sec + first.time_from_start.nanosec*1e-9:.9f}, '
                    f'v={list(first.velocities)}, a={list(first.accelerations)}; '
                    f'last t={last.time_from_start.sec + last.time_from_start.nanosec*1e-9:.9f}, '
                    f'v={list(last.velocities)}, a={list(last.accelerations)}')
            # Keep the private plan immutable after this copy. The single worker
            # only reads it; dispatch always sends another deep copy to JTC.
            self.trajectory, self.ready = trajectory, False
            self.validation_epoch, self.validation_plan_id = epoch, plan_id
            self.phase('joint_validating', '正在检查关节路径连续速度和加速度')
            validation_config = deepcopy(self.guard.config)
            if self.purpose == 'home':
                # Zero is a valid hard-bound endpoint for J2/J3. Only this
                # explicit home plan omits the goal's soft margin; every path
                # knot, polynomial extremum and speed/acceleration stays checked.
                validation_config['joint_margin'] = 0.
            self.pending_validation = self.validator.submit(
                validate_plan_math, trajectory, self.start_q.copy(), self.goal_q.copy(),
                self.guard.model, validation_config, self.scale)
        except Exception as exc:
            self.fail('关节规划未通过检查：' + str(exc), exc)

    def validated(self):
        future = self.pending_validation
        if future is None or not future.done():
            return
        epoch, plan_id = self.validation_epoch, self.validation_plan_id
        self.pending_validation = None
        if not self.current(epoch, plan_id, 'joint_validating'):
            return
        try:
            self.duration = future.result()
            # Main-thread freshness/authority checks happen *after* all math.
            q = self.guard.preparation_health()
            if np.max(np.abs(q-self.start_q)) > .005:
                raise ValueError('路径检查期间起点已变化，请重新规划')
            self.validated_identity = (self.plan_id, self.target_revision, self.purpose, self.scale)
            self.validated_trajectory = self.trajectory
            self.ready, self.planned_at = True, time.monotonic()
            self.phase('joint_ready', '关节路径已就绪，等待执行')
            self.guard.get_logger().info(f'关节计划 {plan_id} ready: {len(self.trajectory.points)} points, '
                                          f'{self.duration:.3f}s, scale={self.scale:.2f}')
            if self.auto_execute:
                self.begin_execution()
        except Exception as exc:
            if isinstance(exc, ValueError) and str(exc) in REPLAN_LIMIT_ERRORS and self.replans < 2:
                try:
                    self.replans += 1
                    self.planning_headroom *= .6
                    self.acceleration_headroom *= .6
                    self.ready, self.trajectory = False, None
                    self.validated_identity = self.validated_trajectory = None
                    self.guard.get_logger().warning(
                        f'关节计划 {plan_id} {exc}; 官方重新规划 {self.replans}/2, '
                        f'velocity_headroom={self.planning_headroom:.3f}, '
                        f'acceleration_headroom={self.acceleration_headroom:.3f}')
                    self.request_plan(epoch, plan_id)
                    return
                except Exception as retry_exc:
                    self.fail('关节路径重新规划失败：' + str(retry_exc), retry_exc)
                    return
            self.fail('关节路径检查失败：' + str(exc), exc)

    def command(self, request, response):
        response.success = False
        if not self.plan_id or request.plan_id != self.plan_id:
            response.message = '计划已过期，请重新规划'
            return response
        if request.command == 'cancel':
            self.guard.stop('已取消关节计划')
            response.success, response.message = True, '已取消关节计划'
        elif request.command == 'execute':
            try:
                self.begin_execution()
                response.success, response.message = True, '正在准备执行'
            except (ValueError, RuntimeError) as exc:
                self.fail(str(exc), exc)
                response.message = str(exc)
        else:
            response.message = '未知关节计划命令'
        return response

    def check_start(self):
        g = self.guard
        q = g.preparation_health()
        if not g.closed or g.hold_pending:
            raise ValueError('等待实机控制门和保持服务确认')
        if np.max(np.abs(q-self.start_q)) > .005:
            raise ValueError('真实关节起点已变化，请重新规划')
        return q

    def begin_execution(self):
        if self.guard.state != 'joint_ready' or not self.ready or self.trajectory is None:
            raise ValueError('没有可执行的关节计划')
        if time.monotonic()-self.planned_at > 30.:
            raise ValueError('关节计划等待过久，请重新规划')
        q = self.check_start()
        if (self.validated_trajectory is not self.trajectory or self.validated_identity !=
                (self.plan_id, self.target_revision, self.purpose, self.scale)):
            raise ValueError('已验证计划身份发生变化，请重新规划')
        if not self.controller.server_is_ready():
            raise ValueError('关节执行服务不可用')
        self.ready = False
        self.guard.controller_hold(q)
        self.guard.stable_since, self.guard.stable_q = 0., q.copy()
        self.phase('joint_syncing', '对齐关节控制器')

    def open_gate(self):
        g = self.guard
        self.check_start()
        if g.real:
            if not g.gate.service_is_ready():
                raise ValueError('实机控制门服务不可用')
            self.phase('joint_opening', '开启关节执行控制门')
            g.closed = False
            epoch, plan_id = g.epoch, self.plan_id
            future = g.gate.call_async(SetBool.Request(data=True))
            future.add_done_callback(lambda f: self.opened(f, epoch, plan_id))
        else:
            self.dispatch()

    def opened(self, future, epoch, plan_id):
        g = self.guard
        if not self.current(epoch, plan_id, 'joint_opening'):
            # An old service may physically have opened the gate after stop.
            if g.state in JOINT_STATES or g.state in ('pausing', 'syncing', 'resetting', 'opening', 'running'):
                g.stop('关节控制门响应已过期，请重新启动', fault=True)
            elif g.real:
                g.closed = False
                g.closing = g.gate.call_async(SetBool.Request(data=False))
            return
        try:
            if not future.result().success:
                raise ValueError('关节执行控制门开启失败')
            self.dispatch()
        except Exception as exc:
            self.fail('关节执行准备失败：' + str(exc), exc)

    def dispatch(self):
        g = self.guard
        q = g.preparation_health()
        if np.max(np.abs(q-self.start_q)) > .005:
            raise ValueError('执行前真实关节起点已变化，请重新规划')
        if np.max(np.abs(g.commanded.get(g.config['feedback_timeout'])-q)) > .003:
            raise ValueError('执行前控制器未对齐真实关节')
        if not self.controller.server_is_ready():
            raise ValueError('关节执行服务不可用')
        self.owned_uuid = uuid.uuid4().bytes  # Registered before the action can publish status.
        action_id = self.owned_uuid
        self.issued_action_ids.add(action_id)
        self.actions[action_id] = None
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = deepcopy(self.trajectory)
        # Zero starts immediately; retain every MoveIt position/derivative/time knot unchanged.
        goal.trajectory.header.stamp.sec = goal.trajectory.header.stamp.nanosec = 0
        self.phase('joint_executing', '正在执行关节路径')
        self.execution_epoch = g.epoch
        self.execution_deadline = time.monotonic()+self.duration+float(g.config.get('plan_settle_timeout', 5.))
        epoch, plan_id = g.epoch, self.plan_id
        self.pending_goal = self.controller.send_goal_async(goal, goal_uuid=UUID(uuid=list(self.owned_uuid)))
        self.pending_goal.add_done_callback(lambda f: self.accepted(f, epoch, plan_id, action_id))

    def accepted(self, future, epoch, plan_id, action_id):
        try:
            handle = future.result()
        except Exception as exc:
            if self.current(epoch, plan_id, 'joint_executing'):
                self.fail('关节动作请求失败：' + str(exc), exc)
            return
        if not handle.accepted:
            self.actions.pop(action_id, None)
        else:
            # A terminal status can precede this send-goal response. Acceptance
            # cannot turn an already terminal goal back into an unresolved one.
            if action_id in self.actions:
                self.actions[action_id] = handle
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda f: self.terminal(f, action_id))
        if not self.current(epoch, plan_id, 'joint_executing'):
            if handle.accepted:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.guard.get_logger().warning('旧关节动作取消请求失败：' + str(exc))
            return
        self.pending_goal = None
        if not handle.accepted:
            self.fail('关节控制器拒绝执行计划')
            return
        self.goal_handle = handle
        self.result_future = result_future
        self.result_future.add_done_callback(lambda f: self.executed(f, epoch, plan_id))

    def executed(self, future, epoch, plan_id):
        if not self.current(epoch, plan_id, 'joint_executing'):
            return
        try:
            result = future.result()
            if result.status != 4 or result.result.error_code != 0:
                raise ValueError(f'关节执行失败：status={result.status}, '
                                 f'code={result.result.error_code}, {result.result.error_string}')
            self.phase('joint_settling', '等待真实关节稳定到位')
            self.settle_since, self.settle_q = 0., None
        except Exception as exc:
            self.fail(str(exc), exc)

    def finish(self):
        # Preserve completion identity for Backend, but revoke the executable plan.
        completed = self.plan_id, self.purpose, self.target_revision
        goal = self.goal_q.copy()
        self.guard.stop('关节运动已到位，正在关闭控制门')
        self.plan_id, self.purpose, self.target_revision = completed
        self.goal_q = goal
        self.phase('joint_finishing', '关节运动已到位，等待停止确认')

    def tick(self, q, now):
        g = self.guard
        elapsed = now-g.stage_time
        if g.state in ('joint_pausing', 'joint_planning', 'joint_validating') and now > self.plan_deadline:
            raise ValueError('关节路径规划和检查累计超时')
        if g.state in ('joint_pausing', 'joint_syncing', 'joint_opening', 'joint_finishing') and elapsed > 3.:
            raise ValueError('关节准备/停止确认超时：' + g.state)
        if g.state == 'joint_planning' and elapsed > float(g.config.get('planning_time', 5.))+2.:
            raise ValueError('关节路径规划超时')
        if g.state == 'joint_validating':
            if elapsed > float(g.config.get('plan_validation_timeout', 5.)):
                raise ValueError('关节路径连续限制检查超时')
            self.validated()
        elif g.state == 'joint_ready':
            if now-self.planned_at > 30.:
                raise ValueError('关节计划已过期，请重新规划')
            if np.max(np.abs(q-self.start_q)) > .005:
                raise ValueError('规划后真实关节起点已变化，请重新规划')
        elif g.state == 'joint_syncing':
            self.check_start()
            commanded = g.commanded.get(g.config['feedback_timeout'])
            if np.max(np.abs(commanded-q)) > .003 or np.max(np.abs(q-g.stable_q)) > .002:
                g.controller_hold(q)
                g.stable_since, g.stable_q = 0., q.copy()
            elif not g.stable_since:
                g.stable_since = now
            elif now-g.stable_since > .3:
                self.open_gate()
        elif g.state in JOINT_FORWARD_STATES:
            if now > self.execution_deadline:
                raise ValueError('真实关节执行/到位超时')
            if g.state == 'joint_executing' and self.goal_handle is None and elapsed > 3.:
                raise ValueError('关节动作接受确认超时')
            if g.state == 'joint_settling':
                commanded = g.commanded.get(g.config['feedback_timeout'])
                if (np.max(np.abs(q-self.goal_q)) > .01 or np.max(np.abs(commanded-q)) > .01
                        or (self.settle_q is not None and np.max(np.abs(q-self.settle_q)) > .002)):
                    self.settle_since, self.settle_q = 0., q.copy()
                elif not self.settle_since:
                    self.settle_since, self.settle_q = now, q.copy()
                elif now-self.settle_since > .3:
                    self.finish()
        elif g.state == 'joint_finishing' and g.closed and not g.hold_pending:
            if np.max(np.abs(q-self.goal_q)) > .01:
                raise ValueError('停止确认期间真实关节偏离目标')
            g.state = 'joint_done'
            g.reason = ('准备姿态已到位' if self.purpose == 'ready' else
                        '关节调整已到位' if self.purpose == 'limit_recovery' else
                        '已回零' if self.purpose == 'home' else '目标点已到位')
            g.get_logger().info(f'关节运动 {self.plan_id} complete; {g.condition_snapshot(q)}')
