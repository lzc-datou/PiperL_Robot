"""Chessboard target source for the project's Servo, MoveIt IK and independent Guard."""
import json
from pathlib import Path
import time

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
from piper_servo_tracking.srv import VisionMotion
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Float64, Header
from visualization_msgs.msg import Marker, MarkerArray

from servo_tracking.cartesian_control import CartesianController
from servo_tracking.core import JOINTS, capped, damped_joint_velocity, joint_speed_limits, pose_error, vector
from servo_tracking.node import Backend, main, pose_message
from scripts.tools.gamepad_control.motion import braking_speed, limit_joint_velocity
from scripts.tools.handeye_calibration.chessboard import PROJECTION_QOS
from .geometry import (TargetGeometry, WristReach, calibration, joint_boundary, matrix, nearby_poses, pose7,
                       settings, workspace_reason, smooth_center, stabilize_goal, POSE_TOPIC, SIM_TOPIC)
from .observation import CenterVelocity, Observations, position_feedforward, stamp_seconds
from .progress import FrozenProgress
from .velocity import stopping_envelope, tracking_step
from .visibility import BoardView

LABEL = ('位置跟踪', '位置跟踪 · 姿态纠偏')
DELAY_HOLD = '棋盘位姿持续迟到'
LOST_HOLD = '棋盘暂不可见'
ARRIVED_HOLD = '已到达最后可见目标'
NEARBY_HOLD = '已到达丢板前已验证的接近点'
STALLED_HOLD = '最后可见目标受限，续行无有效进展'
IK_HOLD = '当前目标暂不可达'
VISION_HOLDS = (DELAY_HOLD, LOST_HOLD, ARRIVED_HOLD, NEARBY_HOLD, STALLED_HOLD, IK_HOLD)
NATIVE_BOUND = 'Servo 关节边界'
GUARD_PREPARING = ('pausing', 'syncing', 'resetting', 'prepared')


class VisualBackend(Backend):
    visual_mode = True

    def __init__(self):
        super().__init__()
        self.visual = settings(self.declare_parameter('visual_config', '').value)
        self.scale = self.visual['default_speed_scale']
        directory = self.declare_parameter('handeye_dir', '').value
        ec, frame, metadata = calibration(self.visual, directory, self.mode == 'sim')
        self.reach = WristReach(self.model)
        self.geometry = TargetGeometry(self.visual, ec)
        self.inputs = Observations(frame, self.visual)
        self.center_velocity = CenterVelocity(self.visual)
        self.feedforward_samples = {}
        self.linear_feedforward = np.zeros(3)
        self.topic = SIM_TOPIC if self.mode == 'sim' else POSE_TOPIC
        self.view = None
        self.view_source_ok = False
        self.view_margin = None
        self.view_limited = False
        self.source_ok = False
        self.board_gid = None
        self.miss_source_ok = False
        self.miss_stamp = self.miss_received = self.start_after = 0.
        self.frozen_revision = None
        self.frozen_progress = FrozenProgress(self.visual)
        self.input_reason = '等候相机与关节反馈'
        self.guard_info = {}
        self.raw_goal = self.goal = None
        self.filtered_center = None
        self.stamp = self.received = self.gap = 0.
        self.waiting_frame = False
        self.deadlines = {}
        self.exposure_stamps = {}
        self.boundary = ''
        self.retreat = self.generation = 0
        self.retreat_stamp = 0.
        self.preparing = self.reopening = False
        self.hold_origin = None
        self.retreat_axes = []
        self.bound_approach = None
        self.bound_scale = 1.
        self.joint_constraint_scale = 1.
        self.workspace_constraint_scale = 1.
        self.unverified_since = None
        self.ik_wait_reason = ''
        self.nearby_revision, self.nearby_motion = -1, ''
        self.approach_progress = FrozenProgress(self.visual)
        self.approach_goal = None
        self.motion_future = self.first_step = None
        self.marker_color = 'checking'
        self.measured = None
        self.last_command = np.zeros(6)
        self.last_joint_speed = np.zeros(6)
        self.joint_velocity_anchor = self.measured_joint_motion = self.queued_joint_motion = None
        self.joint_brake_reason = ''
        self.brake_sample = None
        self.recovery_weight = 1.
        self.linear_until = 0.
        self.previous_linear = np.zeros(3)
        self.vision_gate = self.create_client(VisionMotion, 'guard/vision_motion')
        self.validity = self.create_client(GetStateValidity, 'check_state_validity')
        self.markers = self.create_publisher(MarkerArray, 'vision/markers', 1)
        self.log_dir = Path(self.declare_parameter('visual_log_dir', '').value)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        (self.log_dir/'configuration.json').write_text(json.dumps(dict(
            visual=self.visual, extrinsic=metadata, frame=frame, mode=self.mode,
            tcp_offset=self.config['tcp_offset'], topic=self.topic), ensure_ascii=False, indent=2))
        self.log = (self.log_dir/'tracking.jsonl').open('a', buffering=1)
        self.create_subscription(PoseStamped, self.topic, self.on_board, 1)
        self.create_subscription(Header, self.topic + '/not_visible', self.on_miss, 1)
        self.create_subscription(CameraInfo, self.topic + '/camera_info', self.on_camera_info, PROJECTION_QOS)
        self.create_subscription(JointState, 'guard/trajectory_tail', self.on_accepted_trajectory, 1)
        self.create_timer(.05, self.check_source)
        self.create_timer(.1, self.diagnostics)

    def clock_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_source(self):
        board = self.get_publishers_info_by_topic(self.topic)
        reason = ''
        if len(board) != 1:
            reason = f'棋盘来源异常，位姿发布者 {len(board)} 个，期望 1 个'
        elif self.mode == 'real':
            joints = self.get_publishers_info_by_topic('/piper_servo/feedback/joint_states')
            if board[0].node_name != 'handeye_chessboard':
                reason = '棋盘来源异常，发布节点为 '+board[0].node_name
            elif len(joints) != 1 or joints[0].node_name != 'agx_arm_ctrl_single_node':
                reason = '关节反馈来源异常，等待唯一的实机驱动'
        if not reason:
            gid = bytes(board[0].endpoint_gid)
            if self.active and self.board_gid is not None and self.board_gid != gid:
                reason = '棋盘来源异常，位姿发布者已更换，请重新开始'
            self.board_gid = gid
        self.source_ok = not reason
        self.miss_source_ok = False
        self.view_source_ok = False
        if self.source_ok:
            missing = self.get_publishers_info_by_topic(self.topic + '/not_visible')
            self.miss_source_ok = (len(missing) == 1 and missing[0].node_name == board[0].node_name
                                   and missing[0].node_namespace == board[0].node_namespace)
            projection = self.get_publishers_info_by_topic(self.topic + '/camera_info')
            self.view_source_ok = (len(projection) == 1 and projection[0].node_name == board[0].node_name
                                   and projection[0].node_namespace == board[0].node_namespace)
        if not self.source_ok:
            self.input_reason = reason
            if self.active:
                self.stop_tracking(reason)

    def on_camera_info(self, info):
        if info.header.frame_id != self.inputs.frame:
            return
        try:
            self.view = BoardView(info, self.geometry.ec, self.visual)
        except ValueError:
            self.view = None

    def fresh(self):
        return (self.source_ok and self.goal is not None and self.miss_stamp <= self.stamp and
                0 <= self.clock_s()-self.stamp <= self.visual['max_age_s'] and
                0 <= time.monotonic()-self.received <= self.visual['max_age_s'])

    def frozen_ready(self):
        # Only processed, advancing negative exposures can authorize the same
        # fixed base_link goal. Silence, a late image or a mechanical hold cannot.
        return (self.frozen_revision is not None and self.frozen_revision == self.revision
                and self.active and self.source_ok and self.miss_source_ok and not self.boundary
                and 0 <= self.clock_s()-self.miss_stamp <= self.visual['max_age_s']
                and 0 <= time.monotonic()-self.miss_received <= self.visual['max_age_s'])

    def observation_deadline(self, stamp, received):
        now = time.monotonic()
        return min(received, now-(self.clock_s()-stamp))+self.visual['max_age_s']

    def on_miss(self, header):
        stamp = header.stamp.sec + header.stamp.nanosec*1e-9
        now = self.clock_s()
        if (not self.source_ok or not self.miss_source_ok or header.frame_id != self.inputs.frame
                or stamp <= max(self.miss_stamp, self.inputs.last_input, self.stamp)
                or not 0 <= now-stamp <= self.visual['max_age_s']):
            return
        if self.inputs.pending is not None:
            # Finish matching the preceding visible exposure to joint feedback.
            # Subsequent blank frames can then freeze that exact pose.
            return
        if self.frozen_revision is not None and not self.frozen_ready():
            self.observation_ready()
        self.miss_stamp, self.miss_received = stamp, time.monotonic()
        self.retreat = 0
        self.reset_feedforward()
        if (self.frozen_revision is None and self.active and self.goal is not None and self.guard_state == 'running'
                and not self.preparing and not self.reopening
                and not self.boundary and not self.ik_wait_reason and not self.input_reason
                and self.inputs.last_input <= self.stamp
                and stamp-self.stamp <= self.visual['tracking_loss_timeout_s']):
            self.frozen_revision = self.revision
            self.frozen_progress.reset()
            self.waiting_frame = False
            self.linear_until = 0.
            self.settle_q = None
            self.event('target_frozen', '棋盘暂不可见，继续前往最后可见目标')

    def revoke_visual_command(self):
        self.brake_sample = None
        self.filtered_center = None
        self.approach_goal = None
        self.reset_feedforward()
        self.clear_ik_seeds(cancel_pending=True)
        self.first_step = self.approved_target = None
        self.approved_revision = self.valid_revision = -1
        self.valid = False
        self.deadlines.clear()
        self.exposure_stamps.clear()
        self.linear_until = 0.
        self.retreat = 0
        self.reset_controller()
        self.last_command = np.zeros(6)
        self.publish_twist(self.last_command)

    def observation_ready(self):
        """Permission to calculate a nonzero command on this tick."""
        if not self.source_ok:
            raise ValueError(self.input_reason or '棋盘来源异常')
        if self.frozen_revision is not None:
            if self.frozen_ready():
                self.waiting_frame = False
                return True
            deadline = self.observation_deadline(self.miss_stamp, self.miss_received)
            if (self.miss_source_ok and self.frozen_revision == self.revision and
                    time.monotonic() < deadline+self.visual['brake_window_s'] and
                    self.clock_s() >= self.miss_stamp):
                self.wait_for_frame()
            else:
                self.enter_boundary(LOST_HOLD)
            return False
        if self.fresh():
            return True
        now, received_now = self.clock_s(), time.monotonic()
        age, silence = now-self.stamp, received_now-self.received
        limit = self.visual['tracking_loss_timeout_s']
        expired = age > limit or silence > limit
        delayed = expired and self.inputs.delayed_stream_alive(self.stamp, now, received_now)
        if min(age, silence) < 0:
            raise ValueError(f'观测时间基准异常：图像年龄 {age*1000:.0f} ms，请等待时钟稳定后重新开始')
        if self.goal is None:
            return False
        if self.brake_weight() > 0.:
            self.wait_for_frame()
            return False
        if not self.waiting_frame or self.brake_sample is not None:
            self.waiting_frame = True
            self.recovery_weight = 0.
            self.revoke_visual_command()
            self.unverified_since = None
        if self.reopening:
            # Cancel an in-flight open on the first stale frame; keep any
            # mechanical retreat direction and obtain a new stop generation.
            self.enter_boundary(self.boundary or LOST_HOLD)
        elif expired and not self.boundary:
            reason = DELAY_HOLD if delayed else LOST_HOLD
            self.enter_boundary(reason)
        elif expired and self.boundary == DELAY_HOLD and not delayed:
            self.boundary = LOST_HOLD
            self.event('target_lost', LOST_HOLD)
        return False

    def wait_for_frame(self):
        if not self.waiting_frame:
            self.controller.pause_updates()
            self.reset_feedforward()
        self.waiting_frame = True

    def on_feedback(self, msg):
        previous = self.feedback.stamp
        super().on_feedback(msg)
        if self.feedback.stamp > previous and not self.feedback.error:
            stamp, q = self.feedback.stamp, self.feedback.q
            self.observe_joint_velocity(stamp, q)
            self.inputs.feedback.append((stamp, matrix(self.model.pose(q))))
            if self.inputs.pending:
                self.consume()

    def observe_joint_velocity(self, stamp, q):
        if self.mode == 'real':
            try:
                q = self.model.planning_joints(q, self.config.get('zero_feedback_tolerance', 0.))
            except ValueError:
                self.joint_velocity_anchor = self.measured_joint_motion = None
                return
        anchor = self.joint_velocity_anchor
        if anchor is None or stamp-anchor[0] > self.config['command_timeout']:
            self.joint_velocity_anchor = stamp, q
            self.measured_joint_motion = None
        elif stamp-anchor[0] >= .5/self.config['rate']:
            self.measured_joint_motion = (q-anchor[1])/(stamp-anchor[0]), stamp, self.feedback.received
            self.joint_velocity_anchor = stamp, q

    def on_accepted_trajectory(self, msg):
        # Guard derives this velocity from the position-only knots it sent.
        if self.guard_state != 'running' or list(msg.name) != JOINTS:
            return
        stamp = stamp_seconds(msg)
        timeout = self.config['command_timeout']
        if (not -timeout <= self.clock_s()-stamp <= timeout or
                (self.queued_joint_motion is not None and stamp <= self.queued_joint_motion[2])):
            return
        try:
            q = self.model.check_joints(msg.position)
            velocity = vector(msg.velocity, len(JOINTS))
        except ValueError:
            return
        self.queued_joint_motion = q, velocity, stamp, time.monotonic()

    def on_board(self, msg):
        if not self.source_ok:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if stamp <= max(self.miss_stamp, self.start_after):
            return
        # A returning packet must not mask a real outage between callbacks.
        self.check_previous_observation()
        try:
            self.inputs.receive(msg, self.clock_s(), time.monotonic())
            self.consume()
        except ValueError as exc:
            self.input_reason = str(exc)
            self.retreat = 0
            if self.frozen_revision is not None:
                self.enter_boundary(DELAY_HOLD)

    def check_previous_observation(self):
        if self.active and self.goal is not None:
            try:
                self.observation_ready()
            except ValueError as exc:
                self.stop_tracking(str(exc))

    def consume(self):
        try:
            match = self.inputs.match(self.clock_s(), time.monotonic())
            if match is None:
                return
            stamp, received, be, observation, gap = match
            # Apply the same watchdog even if this callback beats the control
            # timer. An identical new pose must not renew an expired IK result.
            self.check_previous_observation()
            if stamp <= max(self.miss_stamp, self.start_after):
                return
            if self.boundary in VISION_HOLDS and stamp-self.stamp > self.visual['tracking_loss_timeout_s']:
                self.geometry.reset()
            raw = self.geometry.measure(be, observation)
        except ValueError as exc:
            self.input_reason = str(exc)
            self.retreat = 0
            if self.frozen_revision is not None:
                self.enter_boundary(DELAY_HOLD)
            return
        if self.boundary and not self.allows_pending_retreat_target() and self.retreat_stamp != self.stamp:
            self.retreat = 0
        gap_since_exposure = stamp-self.stamp
        previous = self.goal if (self.raw_goal is not None and
                                 0 < gap_since_exposure <= self.visual['tracking_loss_timeout_s']) else None
        reason = self.candidate_boundary(raw.pose)
        candidate = raw
        following = (self.active and self.guard_state == 'running' and
                     not self.start_pending and not self.boundary and not self.waiting_frame and not reason)
        if (following and self.frozen_revision is None and self.filtered_center is not None and
                0 < gap_since_exposure <= self.visual['max_age_s']):
            candidate = smooth_center(raw, self.filtered_center, gap_since_exposure, self.visual['center_filter_s'])
        self.filtered_center = candidate.center.copy() if following else None
        goal = raw if reason else stabilize_goal(candidate, previous, self.visual)
        # A raw sample crossing either side of a workspace face takes priority
        # over a latched sub-millimetre target on the other side of that face.
        if goal.pose is not raw.pose and self.candidate_boundary(goal.pose):
            goal = raw
            self.filtered_center = None
        self.raw_goal = raw
        self.stamp, self.received, self.gap = stamp, received, gap
        self.goal, self.measured, self.input_reason = goal, be, ''
        self.waiting_frame = False
        if self.frozen_revision is not None:
            # Bridge only the first fresh IK handover to the already verified fixed goal.
            if self.approved_revision == self.frozen_revision and self.approved_target is not None:
                self.deadlines[self.approved_revision] = self.observation_deadline(stamp, received)
            self.frozen_revision = None
            self.frozen_progress.reset()
            self.settle_q = None
            self.event('target_reacquired', '棋盘重新可见，正在验证新目标')
        if goal.switched:
            self.brake_sample = None
            # Carry only the already-authorized translation through the short
            # IK handover. Its original observation deadline cannot be renewed.
            if self.active and self.guard_state == 'running' and self.approval_fresh():
                self.previous_linear = self.last_command[:3].copy()
                self.linear_until = self.deadlines[self.approved_revision]
            self.clear_ik_seeds(cancel_pending=True)
            self.approved_target = None
            self.valid_revision = -1
            self.controller.reset_angular_reference(goal.pose[3:])
        self.update_target(pose_message(goal.pose))
        # A stationary board supplies new evidence for the exact same pose.
        self.last_target_change = received
        self.deadlines[self.revision] = self.observation_deadline(stamp, received)
        self.exposure_stamps[self.revision] = stamp
        if following:
            self.feedforward_samples[self.revision] = self.center_velocity.update(stamp, raw.center)
        else:
            self.reset_feedforward()
        for rev in list(self.deadlines):
            if rev < self.revision-64:
                del self.deadlines[rev]
                self.exposure_stamps.pop(rev, None)
                self.feedforward_samples.pop(rev, None)
        if self.ik_wait_reason:
            # A new exposure may have exactly the same numeric TCP target.
            self.valid_revision = -1
        if self.pending is None and self.valid_revision != self.revision:
            self.validation_tick()
        self.count_retreat_frame()

    def allows_pending_retreat_target(self):
        return self.boundary in VISION_HOLDS or self.boundary.startswith(('J', NATIVE_BOUND))

    def count_retreat_frame(self):
        if (not self.boundary or self.reopening or not (self.confirmed() or self.preparing_closed())
                or not self.approval_fresh()):
            return
        stamp = self.stamp
        if self.revision != self.approved_revision:
            if not self.allows_pending_retreat_target():
                return
            # IK can finish one exposure behind a moving board. Count only the
            # exposure that actually passed, even when a newer one is pending.
            stamp = self.exposure_stamps.get(self.approved_revision, 0.)
        if stamp <= self.retreat_stamp:
            return
        if self.allows_pending_retreat_target() and stamp - self.retreat_stamp > self.visual['max_age_s']:
            self.retreat = 0
        self.retreat_stamp = stamp
        moved = ((self.boundary in VISION_HOLDS and
                  (self.boundary != IK_HOLD or self.approved_revision != self.nearby_revision)) or
                 np.linalg.norm(self.goal.center-self.hold_origin[0]) > .0005 or
                 (self.goal.normal is not None and self.hold_origin[1] is not None and
                  np.linalg.norm(self.goal.normal-self.hold_origin[1]) > .002))
        try:
            q, actual = self.actual()
            feasible = moved and (self.boundary in VISION_HOLDS or
                                  self.retreat_step_ok(q, actual, self.start_step(q, actual)))
        except ValueError:
            feasible = False
        self.retreat = self.retreat+1 if feasible else 0

    def align(self):
        # The first camera observation owns the target. FK alone cannot create one.
        pass

    def disabled(self, request, response):
        response.success, response.message = False, '当前由棋盘提供目标，请使用视觉跟踪开始或停止'
        return response

    home_service = plan_service = execute_service = mode_service = disabled

    def start_issue(self):
        if self.guard_info and self.guard_state == 'fault':
            return f'Guard 故障：{self.guard_reason}'
        if not self.source_ok:
            return self.input_reason or '等待视觉位姿来源'
        if not self.guard_info:
            return '等待 Guard 状态'
        if not self.vision_gate.service_is_ready():
            return 'Guard 运动许可服务尚未就绪'
        return ''

    def start(self, request, response):
        self.check_source()
        issue = '已在跟踪' if self.active else self.start_issue()
        if issue:
            response.success, response.message = False, issue
            self.event('start_rejected', issue)
            return response
        if not self.fresh():
            # An empty-screen start must never execute the preceding preview.
            self.revoke_visual_command()
            self.goal = self.raw_goal = self.target = self.preview_q = None
            self.geometry.reset()
            self.inputs.pending = None
            self.start_after = self.clock_s()
            self.ik_wait_reason = ''
            self.reachability = '等待棋盘'
        elif self.ik_wait_reason:
            self.valid_revision = -1
        response = super().start(request, response)
        if not response.success:
            self.event('start_rejected', response.message)
        return response

    def begin_tracking(self):
        if not self.active or self.boundary or self.preparing or self.reopening:
            return
        self.start_pending = True
        if not self.fresh() or not self.approval_fresh():
            self.reason = '等待新鲜棋盘目标通过验证'
            return
        q, actual = self.actual()
        velocity = self.start_step(q, actual)
        reason = self.blocked(q, velocity)
        if reason:
            self.enter_boundary(reason, joint_step=(q, velocity))
            return
        self.generation = self.guard_info['stop_generation']
        self.preparing = True
        self.reason = '正在准备视觉跟踪'
        self.request_motion(True)

    def stop_tracking(self, reason):
        self.frozen_revision = None
        self.frozen_progress.reset()
        self.brake_sample = None
        self.queued_joint_motion = None
        self.joint_brake_reason = ''
        self.filtered_center = None
        self.reset_feedforward()
        self.waiting_frame = False
        self.ik_wait_reason = ''
        self.approach_goal = None
        self.boundary, self.preparing, self.reopening, self.retreat = '', False, False, 0
        self.first_step = self.motion_future = None
        self.geometry.reset()
        self.inputs.pending = None
        self.raw_goal = self.goal = self.target = None
        self.linear_until = 0.
        self.last_command = np.zeros(6)
        self.deadlines.clear()
        self.exposure_stamps.clear()
        self.unverified_since = None
        super().stop_tracking(reason)
        self.event('stop', reason)

    def event(self, event, reason, joint_step=None):
        now = self.clock_s()
        step = {} if joint_step is None else dict(measured_joints_rad=joint_step[0].tolist(),
            requested_joint_speed_rad_s=joint_step[1].tolist())
        if reason.startswith(NATIVE_BOUND):
            step['servo_reference_rad'] = self.guard_info.get('joint_bound_reference_rad')
        elif event in ('frozen_target_stalled', 'target_ik_hold') and joint_step is not None:
            dp, dr = pose_error(self.goal.pose, self.model.pose(joint_step[0]))
            step.update(position_error_mm=float(np.linalg.norm(dp)*1000.),
                        orientation_error_deg=float(np.rad2deg(np.linalg.norm(dr))),
                        joint_constraint_scale=self.joint_constraint_scale)
        self.log.write(json.dumps(dict(event=event, reason=reason, stamp=now,
            source_stamp=self.stamp, age_s=now-self.stamp if self.stamp else None,
            receive_age_s=time.monotonic()-self.received if self.received else None,
            input_detail=self.input_reason, source_ok=self.source_ok, bracket_s=self.gap,
            guard=self.guard_state, guard_generation=self.guard_info.get('stop_generation'),
            generation=self.generation, epoch=self.joint_epoch, **step), ensure_ascii=False)+'\n')
        self.get_logger().info(reason)

    def request_motion(self, enable, *, prepared_epoch=None, valid_until=0., checked_joints=None):
        if not self.vision_gate.service_is_ready():
            self.stop_tracking('视觉 Guard 服务中断')
            return
        generation = self.generation if enable else self.guard_info['stop_generation']
        request = VisionMotion.Request(enable=enable, stop_generation=generation)
        if prepared_epoch is not None:
            request.commit = True
            request.preparation_epoch = prepared_epoch
            request.valid_until = valid_until
            request.checked_joints = checked_joints.tolist()
        self.motion_future = self.vision_gate.call_async(request)
        self.motion_requested = time.monotonic()
        self.motion_future.add_done_callback(lambda f: self.motion_done(f, enable))

    def motion_done(self, future, enable):
        if future is not self.motion_future:
            return
        self.motion_future = None
        try:
            result = future.result()
            if not result.success:
                if enable and result.retry:
                    self.reopening = False
                    if not self.preparing_closed():
                        self.preparing, self.retreat = False, 0
                    return
                self.stop_tracking('运动许可请求失败 ' + result.message)
        except Exception as exc:
            self.stop_tracking(f'运动许可服务异常 {exc}')

    def on_guard(self, msg):
        data = json.loads(msg.data)
        previous = self.guard_state
        if data['state'] != previous or data.get('stop_generation') != self.guard_info.get('stop_generation'):
            self.queued_joint_motion = None
        self.guard_info = data
        self.guard_state, self.guard_reason = data['state'], data['reason']
        if not self.active:
            return
        if self.guard_state == 'fault' or (self.guard_state == 'idle' and previous != 'idle'):
            self.stop_tracking(self.guard_reason)
        elif self.guard_state == 'paused' and (previous != 'paused' or self.generation != data['stop_generation']):
            if data['pause_cause'] != 'joint_bound' or self.goal is None:
                self.stop_tracking(self.guard_reason)
                return
            try:
                q, actual = self.actual()
            except ValueError as exc:
                self.stop_tracking(str(exc))
                return
            # MoveIt checks its next joint state. Its status message carries
            # no joint index; the measured arm may still be well inside limits.
            ref = data.get('joint_bound_reference_rad')
            lower, upper = self.model.lower, self.model.upper
            margin = self.config['joint_margin']
            faces = joint_boundary(q, lower, upper, margin, self.last_joint_speed).split('、')
            if ref is not None:
                ref = np.asarray(ref, dtype=float)
                if ref.shape == (6,) and np.isfinite(ref).all():
                    one_step = joint_speed_limits(self.config)/self.config['rate']
                    direction = np.where(abs(ref-q) > .001, ref-q, self.last_joint_speed)
                    faces += joint_boundary(ref, lower, upper, margin + one_step, direction).split('、')
            detail = '、'.join(dict.fromkeys(f for f in faces if f))
            reason = (NATIVE_BOUND + ' ' + detail) if detail else NATIVE_BOUND
            if not detail:
                # Recover only on a board reversal AND a verified first step
                # opposing the approach. Moving the board farther cannot reopen.
                pose = self.goal.pose.copy()
                approach = self.last_command.copy()
                if np.linalg.norm(approach) <= 1e-8:
                    approach = np.concatenate(pose_error(pose, actual))
                self.bound_approach = (pose, approach)
            self.enter_boundary(reason, already_closed=True, joint_step=(q, self.last_joint_speed))
        elif self.guard_state == 'holding':
            if self.preparing and data['stop_generation'] != self.generation:
                self.enter_boundary(self.boundary or LOST_HOLD, already_closed=True)
            elif self.boundary and not self.reopening and self.motion_future is None:
                # A repeated close may have acknowledged a stop already underway.
                self.generation = data['stop_generation']
        elif self.guard_state == 'running' and previous != 'running':
            self.brake_sample = None
            self.reset_controller()

    def confirmed(self):
        g = self.guard_info
        return (self.motion_future is None and self.guard_state in ('holding', 'paused') and
                g.get('stop_generation') == self.generation and g.get('stop_settled') and
                g.get('gate_closed') and g.get('stop_confirmed'))

    def preparing_closed(self):
        return (self.preparing and self.guard_state in GUARD_PREPARING and
                self.guard_info.get('stop_generation') == self.generation and self.guard_info.get('gate_closed'))

    def enter_boundary(self, reason, already_closed=False, joint_step=None):
        self.frozen_revision = None
        self.frozen_progress.reset()
        self.approach_goal = None
        self.brake_sample = None
        self.reset_feedforward()
        self.retreat = 0
        self.first_step = None
        holding = self.boundary and not self.preparing and not self.reopening and not already_closed
        # A mechanical limit supersedes a camera hold. Its existing close
        # request and generation still own the stop acknowledgement.
        upgrading = holding and self.boundary in VISION_HOLDS and reason not in VISION_HOLDS
        if holding and not upgrading:
            return
        first = not self.boundary
        changed = reason != self.boundary
        self.boundary, self.preparing, self.reopening = reason, False, False
        self.retreat_stamp = self.stamp
        self.linear_until = 0.
        self.hold_origin = (self.goal.center.copy(), self.goal.normal)
        self.retreat_axes = []
        if already_closed or reason.startswith(('J', 'Servo 关节边界')):
            for i in range(len(JOINTS)):
                for name, inward in (('下限', 1), ('上限', -1)):
                    if f'J{i+1} {name}' in reason:
                        self.retreat_axes.append((i, inward))
        self.unverified_since = None
        if not upgrading:
            self.generation = self.guard_info['stop_generation'] + (0 if already_closed else 1)
            self.motion_future = None
        self.start_pending = False
        self.clear_ik_seeds(cancel_pending=True)
        self.approved_target = self.preview_q = None
        self.valid, self.valid_revision = False, -1
        self.reset_controller()
        self.last_command = np.zeros(6)
        self.publish_twist(self.last_command)
        if first or upgrading or (already_closed and changed):
            event = {DELAY_HOLD: 'delayed_input_hold', LOST_HOLD: 'target_lost',
                     ARRIVED_HOLD: 'frozen_target_reached',
                     NEARBY_HOLD: 'frozen_nearby_reached', STALLED_HOLD: 'frozen_target_stalled',
                     IK_HOLD: 'target_ik_hold'}.get(reason, 'boundary_entered')
            self.event(event, reason, joint_step)
        if not already_closed and not upgrading:
            self.request_motion(False)

    def reject(self, reason):
        retry_reason = ''
        waiting_to_start = False
        if reason == '目标不可达或碰撞' and self.ik_context is not None:
            code = self.ik_context.get('ik_failure_code')
            if code in (MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.TIMED_OUT):
                retry_reason = f'逆解未找到可用解（MoveIt {code}）'
                waiting_to_start = (code == MoveItErrorCodes.NO_IK_SOLUTION and self.active
                                    and self.start_pending and self.guard_state == 'idle')
        elif reason == '逆解检查超时':
            retry_reason = reason
        elif reason.startswith('目标处于关节限位保护区'):
            retry_reason = reason
        if retry_reason:
            self.wait_for_ik(retry_reason, waiting_to_start=waiting_to_start)
            return
        self.ik_wait_reason = ''
        self.valid, self.valid_revision = False, self.revision
        self.approved_target = self.preview_q = None
        self.reachability = reason
        self.clear_ik_seeds(cancel_pending=True)
        if self.active:
            self.stop_tracking(reason)

    def wait_for_ik(self, reason, *, waiting_to_start=False, keep_checkpoint=False):
        if keep_checkpoint:
            # A new rejection never extends the previous approval's exposure deadline.
            self.clear_ik_seeds(cancel_pending=True)
            self.brake_sample = None
            self.linear_until = 0.
            self.reset_feedforward()
        else:
            self.revoke_visual_command()
        self.valid, self.valid_revision = False, self.revision
        detail = (reason + '，请将棋盘移回可达位置' if waiting_to_start else reason + '，等待新帧验证')
        self.reachability = ('等待棋盘移回可达位置' if waiting_to_start else '正在验证可行接近点')
        if self.reopening:
            self.enter_boundary(self.boundary or IK_HOLD)
        if self.active and not self.ik_wait_reason:
            self.event('ik_wait', detail)
        self.ik_wait_reason = detail
        if waiting_to_start or (self.active and self.unverified_since is None):
            self.unverified_since = time.monotonic()

    def handle_ik_failure(self, reason, revision):
        context = self.ik_context
        if context is None or not self.active or self.frozen_revision is not None:
            return False
        if reason == '目标不可达或碰撞' and context.get('ik_failure_code') in (
                MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.TIMED_OUT):
            reason = f'逆解未找到可用解（MoveIt {context["ik_failure_code"]}）'
        elif not reason.startswith(('目标处于关节限位保护区', '关节角超出模型限位', '分段逆解需要切换关节分支')):
            return False

        if self.ik_best_q is not None:
            return False  # Preview already found a complete admissible target.
        if revision != self.revision:
            # A rejection of an older exposure may not close the current session.
            self.clear_ik_seeds(cancel_pending=True)
            return True
        if self.try_nearby_pose(context, reason):
            return True
        if 'nearby_poses' not in context and self.ik_seed_index < len(self.ik_seeds):
            return False  # The standard seed search may still find a full solution.
        self.wait_for_ik(context.get('full_failure', reason),
                         waiting_to_start=self.start_pending and self.guard_state == 'idle',
                         keep_checkpoint=(not self.reopening and self.approved_target is not None and
                                          self.approved_revision != revision and self.approval_fresh()))
        return True

    def can_approach(self):
        if not self.active or self.reopening or not self.fresh() or self.frozen_revision is not None:
            return False
        if self.boundary:
            return self.boundary in VISION_HOLDS and (self.confirmed() or self.preparing_closed())
        return self.guard_state == 'running' or self.start_pending

    def try_nearby_pose(self, context, reason):
        if (not self.can_approach() or context.get('target') is None or
                context['vision_deadline']-time.monotonic() < .020):
            return False
        try:
            q, actual = self.actual()
            seed = self.model.planning_joints(q, zero_tolerance=self.config.get('zero_feedback_tolerance', 0.)
                                              if self.mode == 'real' else 0.)
        except ValueError:
            return False
        candidates = context.get('nearby_poses')
        if candidates is None:
            cfg = self.config['control']
            candidates = nearby_poses(
                context['target'], actual, self.visual['distance_m'], cfg['position_tolerance'],
                cfg['orientation_tolerance'], boundary=self.candidate_boundary)
        alternative = next(candidates, None)
        if alternative is None:
            return False
        motion, target = alternative
        context['nearby_poses'] = candidates
        context.setdefault('full_failure', reason)
        context.update(target=target, nearby_motion=motion, max_joint_step=np.deg2rad(25.))
        self.ik_context = context
        self.ik_seed_revision = context['revision']
        self.ik_seed_index, self.ik_seeds = 0, [seed]
        self.ik_reference, self.ik_collect_all = seed, False
        try:
            Backend.submit_ik_seed(self)
        except (ValueError, RuntimeError) as exc:
            self.stop_tracking(f'分段逆解请求失败：{exc}')
            return True
        return self.pending is not None

    def candidate_boundary(self, pose):
        return workspace_reason(pose[:3], self.visual, self.model.radius) or self.reach.reason(pose)

    def validation_tick(self):
        if self.target is None or not (self.fresh() or self.frozen_ready()):
            return
        reason = self.candidate_boundary(self.target)
        if reason and self.pending is None:
            if not self.active:
                self.reachability = '开启跟踪后检查可行接近点'
            elif self.valid_revision != self.revision and self.ik.service_is_ready() and self.can_approach():
                context = dict(target=self.target.copy(), target_time=self.last_target_change,
                               revision=self.revision, epoch=self.joint_epoch,
                               vision_deadline=self.deadlines.get(self.revision, 0.))
                if not self.try_nearby_pose(context, reason):
                    self.wait_for_ik('等待可行接近点',
                                     keep_checkpoint=self.approved_revision != self.revision and self.approval_fresh())
            return
        super().validation_tick()
        if self.ik_context is not None:
            self.ik_context.setdefault('vision_deadline', self.deadlines.get(self.ik_context['revision'], 0.))

    def ik_done(self, future, revision):
        if future is not self.pending or future.cancelled():
            return
        fixed = self.frozen_ready()
        if ((fixed and revision != self.frozen_revision) or
                (not fixed and (not self.fresh() or
                                time.monotonic() > (self.ik_context or {}).get('vision_deadline', 0.)))):
            self.clear_ik_seeds(cancel_pending=True)
            return
        try:
            code = future.result().error_code.val
        except Exception as exc:
            self.reject(f'MoveIt 逆解服务异常：{exc}')
            return
        if code not in (MoveItErrorCodes.SUCCESS, MoveItErrorCodes.NO_IK_SOLUTION, MoveItErrorCodes.TIMED_OUT):
            reason = ('碰撞' if code in (MoveItErrorCodes.START_STATE_IN_COLLISION,
                                       MoveItErrorCodes.GOAL_IN_COLLISION) else '逆解检查失败')
            self.reject(f'MoveIt {reason}（状态码 {code}）')
            return
        self.ik_context['ik_failure_code'] = code
        checked = self.ik_context
        previous_approval = self.approved_target
        super().ik_done(future, revision)
        if self.approved_target is not previous_approval and self.approval_fresh():
            motion = checked.get('nearby_motion', '')
            self.nearby_revision, self.nearby_motion = (revision if motion else -1), motion
            if motion:
                self.reachability = '接近点已验证'
            else:
                self.approach_goal = None
            if motion and revision == self.revision:
                # A repeated full goal must not renew approval of this shorter pose.
                self.target = self.approved_target.copy()
            self.ik_wait_reason = ''
            self.unverified_since = None
            if not self.active and self.approved_revision == self.revision:
                try:
                    q, actual = self.actual()
                    reason = self.blocked(q, self.start_step(q, actual))
                    if reason:
                        self.reachability = f'目标可达 · 当前起步方向受限 {reason}'
                except ValueError as exc:
                    self.reachability = str(exc)
        self.count_retreat_frame()

    def approval_fresh(self):
        if self.frozen_revision is not None:
            return self.frozen_ready() and self.approved_target is not None and self.approved_revision == self.frozen_revision
        return self.approved_target is not None and time.monotonic() <= self.deadlines.get(self.approved_revision, 0.)

    def start_step(self, q, actual, target=None):
        # Guard resets the live controller on entering running. Preview precisely
        # its first output without advancing its integral, timing or observer.
        controller = CartesianController(self.config['control'])
        twist = controller.update(self.approved_target if target is None else target, actual, time.monotonic(),
                                  self.approved_target_time, self.scale, feedforward_active=False)
        return self.joint_velocity(q, self.shape_view(actual, twist))

    def reset_feedforward(self):
        self.center_velocity.reset()
        self.feedforward_samples.clear()
        self.linear_feedforward[:] = 0.

    def control_twist(self, actual):
        self.linear_feedforward[:] = 0.
        if (not self.boundary and self.frozen_revision is None and self.fresh() and
                self.approved_revision != self.nearby_revision):
            estimate = self.feedforward_samples.get(self.approved_revision)
            stamp = self.exposure_stamps.get(self.approved_revision, 0.)
            if estimate is not None and 0. <= self.clock_s()-stamp <= self.visual['max_age_s']:
                error = self.approved_target[:3]-actual[:3]
                self.linear_feedforward[:] = position_feedforward(
                    estimate, error, self.visual['feedforward_position_band_m'])
        twist = super().control_twist(actual, linear_feedforward=self.linear_feedforward)
        return self.shape_view(actual, twist)

    def current_view(self):
        if (self.view_source_ok and self.view is not None and self.raw_goal is not None
                and self.raw_goal.correcting and self.raw_goal.corners is not None and self.fresh()):
            return self.view
        return None

    def shape_view(self, actual, twist):
        view = self.current_view()
        self.view_margin, self.view_limited = None, False
        if view is not None:
            shaped, margin, self.view_limited = view.shape(
                self.raw_goal.corners, actual, twist, self.config['control']['linear_speed']*self.scale)
            self.view_margin = margin if np.isfinite(margin) else None
            if not np.array_equal(shaped[:3], twist[:3]):
                previous = self.last_command[:3]
                increment = self.visual['linear_acceleration_mps2']*self.scale/self.config['rate']
                shaped[:3] = capped(previous+capped(shaped[:3]-previous, increment),
                                   self.config['control']['linear_speed']*self.scale)
            return shaped
        return twist

    def joint_velocity(self, q, twist):
        view = self.current_view()
        reference = self.braking_reference()
        velocity, self.joint_constraint_scale = tracking_step(
            q, twist, self.model, self.config, self.scale,
            self.preview_q if self.approved_target is not None else None,
            reference=reference, position_first=view is not None and bool(np.any(twist[3:])),
            envelope=self.joint_stopping_envelope(q))
        if view is not None and np.any(velocity):
            scale = view.joint_scale(self.raw_goal.corners, self.model.pose(q), self.model.jacobian(q) @ velocity)
            self.view_limited = self.view_limited or scale < 1.
            velocity *= scale
        self.workspace_constraint_scale = self.workspace_speed_scale(q, velocity)
        return velocity*self.workspace_constraint_scale

    def try_retreat(self, q, actual):
        if (self.reopening or self.motion_future or self.first_step or
                not (self.confirmed() or self.preparing_closed()) or
                self.retreat < self.visual['retreat_frames'] or
                (not self.allows_pending_retreat_target() and self.approved_revision != self.revision) or
                not self.approval_fresh()):
            return
        velocity = self.start_step(q, actual)
        if not self.retreat_step_ok(q, actual, velocity):
            self.retreat = 0
            return
        if not self.preparing:
            self.preparing = True
            self.request_motion(True)
        elif self.guard_state == 'prepared':
            self.check_first_step(q, velocity)

    def check_first_step(self, q, velocity):
        if (self.reopening or self.motion_future or self.first_step or self.guard_state != 'prepared'
                or not self.preparing_closed() or not self.validity.service_is_ready()):
            return
        velocity = velocity*self.bound_speed_scale(q, velocity)
        req = GetStateValidity.Request(group_name='arm')
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = (q+velocity/self.config['rate']).tolist()
        self.first_step = self.validity.call_async(req)
        self.step_requested = time.monotonic()
        rev, generation, epoch = self.approved_revision, self.generation, self.guard_info['epoch']
        checked_joints = q.copy()
        self.first_step.add_done_callback(lambda f: self.first_step_checked(f, rev, generation, epoch, checked_joints))

    def first_step_checked(self, future, rev, generation, epoch, checked_joints):
        if future is not self.first_step:
            return
        self.first_step = None
        if not (self.active and self.guard_state == 'prepared' and self.preparing_closed() and
                generation == self.generation and epoch == self.guard_info.get('epoch') and
                (self.retreat >= self.visual['retreat_frames'] if self.boundary else self.start_pending) and
                rev == self.approved_revision and (rev == self.revision or not self.boundary or
                                                    self.allows_pending_retreat_target()) and
                self.fresh() and self.approval_fresh()):
            return
        try:
            if not future.result().valid:
                self.stop_tracking('视觉首段碰撞检查未通过')
                return
            q, actual = self.actual()
            if np.max(np.abs(q-checked_joints)) > .002:
                return
            velocity = self.start_step(q, actual)
            if self.boundary:
                if not self.retreat_step_ok(q, actual, velocity):
                    self.retreat = 0
                    return
            elif reason := self.blocked(q, velocity):
                self.enter_boundary(reason, joint_step=(q, velocity))
                return
        except (RuntimeError, ValueError) as exc:
            self.stop_tracking(str(exc))
            return
        now = time.monotonic()
        deadline = min(self.deadlines[rev], self.observation_deadline(self.stamp, self.received),
                       now+self.config['command_timeout'])
        if deadline > now:
            self.reopening = True
            self.request_motion(True, prepared_epoch=epoch, valid_until=deadline, checked_joints=checked_joints)

    def retreat_step_ok(self, q, actual, velocity):
        if not self.inward(q, velocity):
            return False
        if (self.boundary in VISION_HOLDS and self.approved_revision == self.nearby_revision
                and not np.any(abs(velocity) > 1e-7)):
            return False
        if self.boundary in (STALLED_HOLD, IK_HOLD) and not np.any(abs(velocity) > 1e-7):
            dp, dr = pose_error(self.approved_target, actual)
            return (np.linalg.norm(dp) <= self.config['hold_position_tolerance'] and
                    np.linalg.norm(dr) <= self.config['hold_orientation_tolerance'])
        if self.approved_revision == self.revision or self.boundary in VISION_HOLDS:
            return True
        # A newer, still unapproved exposure can veto a retreat immediately.
        # It never supplies the command sent to Servo.
        return self.inward(q, self.start_step(q, actual, self.goal.pose), self.goal.pose)

    def inward(self, q, velocity, target=None):
        if self.blocked(q, velocity):
            return False
        if self.boundary == NATIVE_BOUND:
            pose, approach = self.bound_approach
            if not np.any(approach):
                # No approach direction was observable. A new checked step may
                # only move joints toward the interior of their own ranges.
                center = (self.model.lower + self.model.upper)*.5
                return bool(np.any(velocity) and
                            np.all(abs(q + velocity/self.config['rate'] - center) <= abs(q-center) + 1e-12))
            weight = self.config['dls_rotation_weight']**2
            direction = approach * np.array([1., 1., 1., weight, weight, weight])
            direction /= np.linalg.norm(direction)
            retreat = np.concatenate(pose_error(self.approved_target if target is None else target, pose))
            return bool(direction @ retreat < -1e-10 and
                        direction @ (self.model.jacobian(q) @ velocity) < -1e-10)
        return all(velocity[i]*direction > 1e-7 for i, direction in self.retreat_axes)

    def bound_speed_scale(self, q, velocity):
        envelope = self.joint_stopping_envelope(q)
        position = np.where(velocity > 0, envelope.upper, envelope.lower)
        _, scale = limit_joint_velocity(position, velocity, self.model.lower, self.model.upper,
            self.config['joint_acceleration'], self.config['joint_margin'], self.config['command_timeout'])
        return scale

    def workspace_speed_scale(self, q, velocity):
        if not np.any(velocity):
            return 1.
        p = self.model.pose(q)[:3]
        linear = self.model.jacobian(q)[:3] @ velocity
        ref = self.braking_reference()
        ref_p = p if ref is None else self.model.pose(ref)[:3]
        acceleration = self.visual['linear_acceleration_mps2']*self.scale
        latency = self.config['command_timeout']
        scale = 1.
        for x, queued, speed, low, high in zip(p, ref_p, linear,
                self.visual['workspace_min_m'], self.visual['workspace_max_m']):
            if abs(speed) > 1e-10:
                room = high-max(x, queued) if speed > 0. else min(x, queued)-low
                scale = min(scale, braking_speed(room, acceleration, latency)/abs(speed))
        return scale

    def braking_reference(self):
        # Both nodes run in the same local launch. Use Guard's validated knot
        # only to reduce outward motion, never as a replacement for actual FK.
        queued = self.fresh_queued_motion()
        ref = self.guard_info.get('servo_reference_rad')
        received = self.guard_info.get('reference_received', 0.)
        if (self.guard_state == 'running' and ref is not None and (queued is None or received > queued[3]) and
                0. <= time.monotonic()-received <= self.config['command_timeout']):
            ref = np.asarray(ref, dtype=float)
            if ref.shape == (6,) and np.isfinite(ref).all():
                return ref
        return queued[0] if queued is not None else None

    def fresh_queued_motion(self):
        queued = self.queued_joint_motion
        timeout = self.config['command_timeout']
        if (self.guard_state == 'running' and queued is not None and
                0. <= time.monotonic()-queued[3] <= timeout and
                -timeout <= self.clock_s()-queued[2] <= timeout):
            return queued
        return None

    def fresh_measured_motion(self):
        measured = self.measured_joint_motion
        if (measured is not None and
                0. <= time.monotonic()-measured[2] <= self.config['command_timeout'] and
                -.1 <= self.clock_s()-measured[1] <= self.config['command_timeout']):
            return measured
        return None

    def joint_stopping_envelope(self, q):
        motion = []
        timeout = self.config['command_timeout']
        if (measured := self.fresh_measured_motion()) is not None:
            velocity, stamp, _ = measured
            motion.append((q, velocity, timeout+max(0., self.clock_s()-stamp)))
        if (queued := self.fresh_queued_motion()) is not None:
            position, velocity, _, received = queued
            # Servo can have one unpublished knot ahead of Guard's last
            # accepted one. Reserve it and the next command tick.
            motion.append((position, velocity, 2./self.config['rate']+time.monotonic()-received))
        return stopping_envelope(q, self.model, self.config, reference=self.braking_reference(), motion=motion)

    def brake_near_joint_limit(self, q):
        envelope = self.joint_stopping_envelope(q)
        if envelope.hold:
            self.enter_boundary(envelope.hold, joint_step=(q, self.last_joint_speed))
            return True
        if not envelope.braking:
            return False
        self.joint_brake_reason = envelope.braking
        self.controller.pause_updates()
        self.brake_sample = None
        self.linear_until = 0.
        self.recovery_weight = self.bound_scale = 0.
        self.linear_feedforward[:] = 0.
        self.last_command = np.zeros(6)
        self.publish_twist(self.last_command)
        return True

    def braking_position(self, q, velocity):
        ref = self.braking_reference()
        if ref is None:
            return q
        return np.where(velocity > 0, np.maximum(q, ref), np.minimum(q, ref))

    def blocked(self, q, velocity):
        if not np.any(velocity):
            return ''
        horizon = self.config['command_timeout']
        position = self.braking_position(q, velocity)
        # The shared braking model bounds outward travel over the command
        # horizon. At the face, inspect the original sign before clipping to zero.
        joints = joint_boundary(position, self.model.lower, self.model.upper, self.config['joint_margin'],
                                velocity)
        if joints:
            return joints
        p = self.model.pose(q)[:3]
        dp = self.model.jacobian(q)[:3] @ velocity
        for axis, x, v, low, high in zip('XYZ', p, dp, self.visual['workspace_min_m'],
                                         self.visual['workspace_max_m']):
            if x + v*horizon <= low and v < -1e-10:
                return f'工作空间 {axis}下限'
            if x + v*horizon >= high and v > 1e-10:
                return f'工作空间 {axis}上限'
        return ''

    def approach_limit_reason(self, q, actual):
        target = self.goal.pose
        position_tol = self.config['hold_position_tolerance']
        for axis, x, goal, low, high in zip('XYZ', actual[:3], target[:3],
                self.visual['workspace_min_m'], self.visual['workspace_max_m']):
            if goal < low and x <= low+position_tol:
                return f'工作空间 {axis}下限'
            if goal > high and x >= high-position_tol:
                return f'工作空间 {axis}上限'
        desired = np.concatenate(pose_error(target, actual))
        direction = damped_joint_velocity(q, desired, self.model, self.config, self.scale)
        faces = joint_boundary(q, self.model.lower, self.model.upper, self.config['joint_margin']*2., direction)
        return faces or IK_HOLD

    def approach_stalled(self, actual):
        goal = self.goal.pose
        tolerances = (self.config['hold_position_tolerance'], self.config['hold_orientation_tolerance'])
        if self.approach_goal is not None:
            change = pose_error(goal, self.approach_goal)
            if any(np.linalg.norm(error) > 2.*tol for error, tol in zip(change, tolerances)):
                self.approach_goal = None
        if self.approach_goal is None:
            self.approach_goal = goal.copy()
            self.approach_progress.reset()
        errors = tuple(np.linalg.norm(error) for error in pose_error(self.approach_goal, actual))
        return self.approach_progress.update(self.feedback.stamp, errors, tolerances, limited=True,
                                             position_first=True)

    def brake_weight(self):
        if (self.brake_sample is None or not self.active or self.guard_state != 'running' or
                self.boundary or self.start_pending or self.reopening or not self.source_ok or
                self.ik_wait_reason or
                (self.input_reason and not self.input_reason.startswith('观测已过期'))):
            return 0.
        _, deadline, _ = self.brake_sample
        if self.frozen_revision is not None:
            if (not self.miss_source_ok or self.approved_revision != self.frozen_revision
                    or self.clock_s() < self.miss_stamp):
                return 0.
            observation = self.observation_deadline(self.miss_stamp, self.miss_received)
        else:
            observation = self.observation_deadline(self.stamp, self.received)
        overdue = max(0., time.monotonic()-min(deadline, observation))
        return max(0., 1.-overdue/self.visual['brake_window_s'])

    def publish_twist(self, values, *, joint_step=None):
        super().publish_twist(values, joint_step=joint_step)
        self.last_joint_speed = joint_step[1].copy() if joint_step is not None else np.zeros(6)

    def send_tracking_step(self, q, velocity, command, deadline):
        # Smooth the return from a camera gap at the existing servo tick rate.
        self.recovery_weight = min(1., self.recovery_weight +
                                   1./(self.config['rate']*self.visual['brake_window_s']))
        self.bound_scale = self.bound_speed_scale(q, velocity*self.recovery_weight)
        weight = self.recovery_weight*self.bound_scale
        velocity = velocity*weight
        self.last_command = command*(weight*self.workspace_constraint_scale)
        self.publish_twist(self.last_command, joint_step=(q, velocity))
        if self.frozen_revision is not None:
            deadline = self.observation_deadline(self.miss_stamp, self.miss_received)
        else:
            deadline = min(deadline, self.observation_deadline(self.stamp, self.received))
        if np.any(velocity) and deadline > time.monotonic():
            self.brake_sample = (velocity.copy(), deadline, self.scale)
        else:
            self.brake_sample = None
        return velocity

    def brake_on_gap(self, q, actual, weight):
        # Only slow an already issued command. Limit travel to its original
        # reference and check the current measured joints on each braking tick.
        velocity, _, scale = self.brake_sample
        self.linear_feedforward[:] = 0.
        weight = min(weight, self.scale/max(scale, 1e-12), self.recovery_weight)
        twist = self.model.jacobian(q) @ velocity
        limits = self.config['control']
        for speed, limit in ((twist[:3], limits['linear_speed']), (twist[3:], limits['angular_speed'])):
            weight = min(weight, self.scale*limit/max(np.linalg.norm(speed), 1e-12))
        dt = 1./self.config['rate']
        if self.approved_target is not None:
            references = [self.approved_target]
            if self.fresh():
                references.append(self.goal.pose)
            for target in references:
                for error, speed in zip(pose_error(target, actual), (twist[:3], twist[3:])):
                    norm2 = float(speed @ speed)
                    if norm2 > 1e-12:
                        weight = min(weight, float(error @ speed)/(dt*norm2))
        else:
            weight = 0.
        self.recovery_weight = max(0., weight)
        velocity = velocity*self.recovery_weight
        workspace_scale = self.workspace_speed_scale(q, velocity)
        velocity *= workspace_scale
        reason = self.blocked(q, velocity)
        if reason:
            self.enter_boundary(reason, joint_step=(q, velocity))
            self.last_command = np.zeros(6)
            return
        scale = self.bound_speed_scale(q, velocity)
        velocity = velocity*scale
        self.last_command = twist*(self.recovery_weight*scale*workspace_scale)
        if np.any(velocity):
            self.publish_twist(self.last_command, joint_step=(q, velocity))
        elif self.fresh() or self.frozen_revision is not None:
            # A reversal may require zero while the new IK is in flight.
            # Leave that validation alive so the next tick can use its result.
            self.brake_sample = None
            self.publish_twist(self.last_command)
        else:
            self.revoke_visual_command()

    def tick(self):
        self.joint_brake_reason = ''
        self.heartbeat.publish(Float64(data=time.time()))
        if not self.ensure_switched():
            return
        command = np.zeros(6)
        if self.active:
            try:
                if time.monotonic()-self.panel_seen > .6:
                    raise ValueError('RViz 面板连接中断')
                if self.motion_future and time.monotonic()-self.motion_requested > 1.:
                    raise ValueError('Guard 服务超时')
                if self.first_step and time.monotonic()-self.step_requested > 1.:
                    raise ValueError('视觉首段碰撞检查超时')
                q, actual = self.actual()
                ready = self.observation_ready()
                if (self.guard_state == 'running' and not self.boundary and self.goal is not None and
                        self.brake_near_joint_limit(q)):
                    return
                awaiting_ik = (ready and self.brake_sample is not None and
                               self.approved_target is not None and not self.approval_fresh() and
                               time.monotonic() >= self.linear_until)
                if (not ready or awaiting_ik) and (weight := self.brake_weight()) > 0.:
                    if awaiting_ik:
                        self.controller.pause_updates()
                    self.brake_on_gap(q, actual, weight)
                    return
                if (ready and not self.boundary and not self.approval_fresh() and
                        self.unverified_since is not None and
                        time.monotonic()-self.unverified_since > self.visual['tracking_loss_timeout_s']):
                    self.enter_boundary(LOST_HOLD if self.frozen_revision is not None else
                                        self.approach_limit_reason(q, actual),
                                        joint_step=(q, np.zeros(6)))
                    return
                if not ready:
                    self.recovery_weight = 0.
                    self.last_command = command.copy()
                    self.publish_twist(command)
                    return
                if self.boundary:
                    if (self.reopening and self.guard_state == 'running' and
                            self.guard_info.get('stop_generation') == self.generation and
                            self.approval_fresh() and (self.allows_pending_retreat_target() or
                                                       self.approved_revision == self.revision)):
                        candidate = self.control_twist(actual)
                        velocity = self.joint_velocity(q, candidate)
                        if self.retreat_step_ok(q, actual, velocity):
                            if self.boundary in (LOST_HOLD, ARRIVED_HOLD, NEARBY_HOLD, STALLED_HOLD):
                                self.event('target_reacquired', '棋盘重新可见，新目标已验证，恢复跟踪')
                            elif self.boundary == DELAY_HOLD:
                                self.event('delayed_input_recovered', '新鲜棋盘位姿已验证，Guard 已恢复')
                            elif self.boundary == IK_HOLD:
                                self.event('ik_recovered', '已验证可达目标，Guard 已恢复')
                            else:
                                self.event('boundary_recovered', '已退回，Guard 已恢复')
                            self.boundary, self.preparing, self.reopening, self.retreat = '', False, False, 0
                            self.reason = '视觉跟踪中'
                            command = candidate
                        elif np.any(velocity) or not self.retreat_step_ok(q, actual, self.start_step(q, actual)):
                            self.enter_boundary(self.boundary)
                    else:
                        self.try_retreat(q, actual)
                elif self.start_pending and self.approval_fresh():
                    if (self.reopening and self.guard_state == 'running' and
                            self.guard_info.get('stop_generation') == self.generation):
                        self.start_pending = self.preparing = self.reopening = False
                        self.reason = '视觉跟踪中'
                    elif self.guard_state == 'prepared' and self.preparing_closed():
                        velocity = self.start_step(q, actual)
                        if reason := self.blocked(q, velocity):
                            self.enter_boundary(reason, joint_step=(q, velocity))
                        else:
                            self.check_first_step(q, velocity)
                    else:
                        self.begin_tracking()
                elif self.guard_state == 'running' and (
                        self.approval_fresh() or time.monotonic() < self.linear_until):
                    self.unverified_since = None
                    if self.approval_fresh():
                        self.linear_until = 0.
                        command = self.control_twist(actual)
                        if self.frozen_revision is not None and self.frozen_settled(q, actual):
                            partial = self.approved_revision == self.nearby_revision
                            self.enter_boundary(NEARBY_HOLD if partial else ARRIVED_HOLD)
                            return
                    else:
                        limit = self.config['linear_speed']*self.scale
                        command[:3] = self.previous_linear*min(1., limit/max(np.linalg.norm(self.previous_linear), 1e-12))
                    velocity = self.joint_velocity(q, command)
                    reason = self.blocked(q, velocity)
                    if reason:
                        self.enter_boundary(reason, joint_step=(q, velocity))
                        command[:] = 0.
                    elif self.frozen_revision is not None and self.frozen_stalled(q, actual, velocity):
                        self.enter_boundary(STALLED_HOLD, joint_step=(q, velocity))
                        return
                    elif self.fresh() and self.approval_fresh() and self.nearby_motion and (
                            self.approved_revision == self.nearby_revision) and self.approach_stalled(actual):
                        self.enter_boundary(self.approach_limit_reason(q, actual), joint_step=(q, velocity))
                        return
                elif self.guard_state == 'running' or self.start_pending:
                    if self.unverified_since is None:
                        self.unverified_since = time.monotonic()
            except ValueError as exc:
                command[:] = 0.
                self.stop_tracking(str(exc))
        if np.any(command) and (not (self.fresh() or self.frozen_ready()) or
                (not self.approval_fresh() and time.monotonic() >= self.linear_until)):
            command[:] = 0.
        self.last_command = command.copy()
        if np.any(command):
            self.send_tracking_step(q, velocity, command, self.deadlines.get(self.approved_revision, self.linear_until))
        else:
            self.linear_feedforward[:] = 0.
            self.brake_sample = None
            self.bound_scale = 1.
            self.joint_constraint_scale = 1.
            self.workspace_constraint_scale = 1.
            self.publish_twist(command)

    def frozen_settled(self, q, actual):
        dp, dr = pose_error(self.approved_target, actual)
        if (np.linalg.norm(dp) > self.config['hold_position_tolerance'] or
                np.linalg.norm(dr) > self.config['hold_orientation_tolerance'] or
                not self.controller.motion_settled):
            self.settle_q = None
        elif self.settle_q is None or np.max(abs(q-self.settle_q)) > self.config['hold_joint_stability']:
            self.settle_q, self.settle_since = q.copy(), time.monotonic()
        elif time.monotonic()-self.settle_since >= self.config['hold_settle_time']:
            return True
        return False

    def frozen_stalled(self, q, actual, velocity):
        dp, dr = pose_error(self.approved_target, actual)
        return self.frozen_progress.update(
            self.feedback.stamp, (np.linalg.norm(dp), np.linalg.norm(dr)),
            (self.config['hold_position_tolerance'], self.config['hold_orientation_tolerance']),
            limited=min(self.joint_constraint_scale, self.workspace_constraint_scale,
                        self.bound_speed_scale(q, velocity)) < .999)

    def visual_status(self):
        g = self.goal
        visible = g is not None and self.frozen_revision is None and self.fresh()
        detail = self.reason
        input_detail = self.input_reason
        gap_detail = '短时迟帧，减速' if self.waiting_frame and self.brake_sample is not None else '等待新鲜棋盘帧 · 零速'
        nearby = self.nearby_motion if (
            self.approved_revision == self.nearby_revision and self.approval_fresh()) else ''
        if self.boundary in VISION_HOLDS:
            if self.preparing:
                phase = self.guard_reason
            elif self.retreat:
                phase = '新帧已到，正在验证'
            elif self.confirmed():
                phase = '已停稳，等待可行目标' if visible or self.boundary == IK_HOLD else '已停稳，等待棋盘'
            else:
                phase = '已请求保持，等待停稳'
            if visible:
                limit = self.candidate_boundary(g.pose) if self.raw_goal is not None else ''
                target = f' · 完整目标触及{limit}，验证接近点' if limit else ''
                detail = f'棋盘可见{target} · {phase}'
            else:
                detail = f'{self.boundary} · {phase}'
        elif self.boundary:
            phase = ('已退回，正在验证' if self.reopening or self.retreat else
                     '机械臂已停稳，等待棋盘退回' if self.confirmed() else '已请求暂停，等待停稳')
            label = '关节受限' if self.boundary.startswith(('J', NATIVE_BOUND)) else '到达边界'
            detail = f'{label} {self.boundary} · {phase}'
        elif self.active and self.goal is None:
            detail = '跟踪已开启，等待棋盘出现'
        elif self.active and self.preparing:
            detail = self.guard_reason
        elif self.active and self.joint_brake_reason:
            detail = f'{self.joint_brake_reason} · 正在减速'
        elif self.active and self.waiting_frame:
            detail = gap_detail
        elif self.frozen_revision is not None:
            detail = '棋盘暂不可见，继续前往最后可见目标' if self.approval_fresh() else '棋盘暂不可见，正在验证最后目标'
        elif self.active and self.ik_wait_reason:
            detail = '正在验证可行接近点' + (' · 沿已验证目标接近' if self.approval_fresh() else ' · 原位等待')
        elif self.active and self.guard_state == 'running' and g is not None:
            detail = ('接近关节边界，跟踪减速 · ' if min(self.bound_scale, self.joint_constraint_scale) < .999
                      else '') + LABEL[g.correcting]
            if self.view_limited and self.fresh():
                detail += ' · 视野约束'
            if self.workspace_constraint_scale < .999:
                detail += ' · 接近工作空间边界，减速'
            if nearby:
                detail += ' · '+nearby
        elif not self.active and self.reason == '等待反馈' and (issue := self.start_issue()):
            detail = issue
        elif not self.active and self.fresh():
            if self.reason.startswith(('棋盘观测', '棋盘来源异常', '关节反馈来源异常')):
                detail = '棋盘识别已恢复 · 等待开启跟踪'
                input_detail = '上次停止：'+self.reason
            else:
                prefix = '' if self.reason == '等待反馈' else self.reason+' · '
                detail = prefix+'预览 · '+LABEL[g.correcting]
        tcp_center = (self.geometry.ec[:3, :3] @ g.camera_center + self.geometry.ec[:3, 3]
                      if visible else None)
        if visible:
            tolerance = self.visual['normal_tolerance_deg']
            if g.alpha is not None:
                detail += f' | 垂直偏差 {g.alpha:.1f}°（容差 {tolerance:g}°）'
            detail += f' | TCP 距离 {np.linalg.norm(tcp_center)*100:.1f} cm，横偏 {np.linalg.norm(tcp_center[:2])*1000:.1f} mm'
        return dict(state=detail, servo_ready=self.switched and self.ik.service_is_ready() and self.gate.service_is_ready(),
            visual_branch=LABEL[g.correcting] if g is not None else None,
            angle_deg=g.alpha if visible else None,
            boundary='' if self.boundary in VISION_HOLDS else self.boundary,
            waiting_for_target=self.active and (g is None or self.boundary in (LOST_HOLD, ARRIVED_HOLD,
                                                                              NEARBY_HOLD, STALLED_HOLD)),
            frozen_target=self.frozen_revision is not None, frozen_revision=self.frozen_revision,
            frozen_progress_hold=self.boundary == STALLED_HOLD,
            target_ik_hold=self.boundary == IK_HOLD,
            nearby_motion=nearby or None,
            workspace_constraint_scale=self.workspace_constraint_scale,
            last_missing_exposure=self.miss_stamp or None,
            delayed_input_hold=self.boundary == DELAY_HOLD,
            retreat_frames=self.retreat, source_stamp=self.stamp, age_s=self.clock_s()-self.stamp if g else None,
            receive_age_s=time.monotonic()-self.received if self.received else None,
            waiting_frame=self.waiting_frame, braking_on_gap=self.waiting_frame and self.brake_sample is not None,
            view_geometry_available=self.view_source_ok and self.view is not None,
            view_margin_px=self.view_margin if visible else None,
            view_limited=self.view_limited and visible,
            tcp_center_m=tcp_center.tolist() if tcp_center is not None else None, source_ok=self.source_ok,
            camera_center_m=g.camera_center.tolist() if g else None, input_detail=input_detail)

    def diagnostics(self):
        status = self.visual_status()
        status.update(active=self.active, guard=self.guard_state, generation=self.generation,
                      joint_bound_speed_scale=self.bound_scale,
                      joint_constraint_scale=self.joint_constraint_scale,
                      joint_brake_reason=self.joint_brake_reason,
                      linear_feedforward_mps=self.linear_feedforward.tolist(),
                      approved=self.approved_target.tolist() if self.approved_target is not None else None,
                      guard_stop={k: self.guard_info.get(k) for k in (
                          'stop_generation', 'gate_closed', 'stop_settled', 'stop_confirmed')},
                      received=self.received, bracket_s=self.gap, command=self.last_command.tolist())
        if self.feedback.q is not None:
            status.update(measured_joints_rad=self.feedback.q.tolist(),
                          requested_joint_speed_rad_s=self.last_joint_speed.tolist())
        if (measured := self.fresh_measured_motion()) is not None:
            status['measured_joint_speed_rad_s'] = measured[0].tolist()
        if (queued := self.fresh_queued_motion()) is not None:
            status.update(queued_joints_rad=queued[0].tolist(), queued_joint_speed_rad_s=queued[1].tolist(),
                          queued_joint_stamp=queued[2])
        if self.goal is not None:
            status.update(center_m=self.goal.center.tolist(), normal=self.goal.normal.tolist() if self.goal.normal is not None else None,
                          candidate=self.goal.pose.tolist(), measured_tcp=pose7(self.measured).tolist(),
                          normal_sign=self.geometry.sign, viewing_cos=self.goal.viewing_cos)
            status.update(raw_center_m=self.raw_goal.center.tolist(),
                          raw_normal=self.raw_goal.normal.tolist() if self.raw_goal.normal is not None else None,
                          raw_candidate=self.raw_goal.pose.tolist())
            markers = []
            for i, (origin, direction, color) in enumerate((
                (self.measured[:3, 3], self.measured[:3, 2], (0.2, .5, 1.)),
                (self.goal.center, self.goal.normal, (.2, 1., .3)))):
                if direction is None:
                    continue
                m = Marker(ns='board_axes', id=i, type=Marker.ARROW)
                m.header.frame_id = 'base_link'
                m.pose.orientation.w = 1.
                m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                            for p in (origin, origin+.10*direction)]
                m.scale.x, m.scale.y, m.scale.z = .006, .012, .015
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 1.
                m.lifetime.nanosec = 300000000
                markers.append(m)
            if self.fresh():
                self.markers.publish(MarkerArray(markers=markers))
        self.log.write(json.dumps(status, ensure_ascii=False)+'\n')

    def destroy_node(self):
        self.log.close()
        super().destroy_node()


if __name__ == '__main__':
    main(VisualBackend)
