"""Chessboard target source for the project's Servo, MoveIt IK and independent Guard."""
import json
from pathlib import Path
import time

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetStateValidity
from piper_servo_tracking.srv import VisionMotion
from std_msgs.msg import Float64, Header
from visualization_msgs.msg import Marker, MarkerArray

from servo_tracking.cartesian_control import CartesianController
from servo_tracking.core import JOINTS, joint_speed_limits, pose_error
from servo_tracking.node import Backend, main, pose_message
from scripts.tools.gamepad_control.motion import limit_joint_velocity
from .geometry import (TargetGeometry, WristReach, calibration, joint_boundary, matrix, pose7, settings,
                       workspace_reason, smooth_center, stabilize_goal, POSE_TOPIC, SIM_TOPIC)
from .observation import Observations
from .velocity import tracking_velocity

LABEL = ('位置跟踪', '位置跟踪 · 姿态纠偏')
DELAY_HOLD = '棋盘位姿持续迟到'
LOST_HOLD = '棋盘暂不可见'
ARRIVED_HOLD = '已到达最后可见目标'
VISION_HOLDS = (DELAY_HOLD, LOST_HOLD, ARRIVED_HOLD)
NATIVE_BOUND = 'Servo 关节边界'


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
        self.topic = SIM_TOPIC if self.mode == 'sim' else POSE_TOPIC
        self.source_ok = False
        self.board_gid = None
        self.miss_source_ok = False
        self.miss_stamp = self.miss_received = self.start_after = 0.
        self.frozen_revision = None
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
        self.reopening = False
        self.hold_origin = None
        self.retreat_axes = []
        self.bound_approach = None
        self.bound_scale = 1.
        self.unverified_since = None
        self.ik_wait_reason = ''
        self.motion_future = self.first_step = None
        self.marker_color = 'checking'
        self.measured = None
        self.last_command = np.zeros(6)
        self.last_joint_speed = np.zeros(6)
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
        if self.source_ok:
            missing = self.get_publishers_info_by_topic(self.topic + '/not_visible')
            self.miss_source_ok = (len(missing) == 1 and missing[0].node_name == board[0].node_name
                                   and missing[0].node_namespace == board[0].node_namespace)
        if not self.source_ok:
            self.input_reason = reason
            if self.active:
                self.stop_tracking(reason)

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

    def on_miss(self, header):
        stamp = header.stamp.sec + header.stamp.nanosec*1e-9
        now = self.clock_s()
        if (not self.miss_source_ok or header.frame_id != self.inputs.frame
                or stamp <= max(self.miss_stamp, self.inputs.last_input, self.stamp)
                or not 0 <= now-stamp <= self.visual['max_age_s']):
            return
        if self.inputs.pending is not None:
            # Finish matching the preceding visible exposure to joint feedback.
            # Subsequent blank frames can then freeze that exact pose.
            return
        if self.frozen_revision is not None and not self.frozen_ready():
            self.enter_boundary(LOST_HOLD)
        self.miss_stamp, self.miss_received = stamp, time.monotonic()
        if (self.frozen_revision is None and self.active and self.goal is not None
                and not self.boundary and not self.ik_wait_reason and not self.input_reason
                and self.inputs.last_input <= self.stamp
                and stamp-self.stamp <= self.visual['tracking_loss_timeout_s']):
            self.frozen_revision = self.revision
            self.waiting_frame = False
            self.brake_sample = None
            self.linear_until = 0.
            self.settle_q = None
            self.event('target_frozen', '棋盘暂不可见，继续前往最后可见目标')

    def revoke_visual_command(self):
        self.brake_sample = None
        self.filtered_center = None
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
        if self.frozen_revision is not None:
            if self.frozen_ready():
                return True
            self.enter_boundary(LOST_HOLD)
        if self.fresh():
            return True
        if not self.source_ok:
            raise ValueError(self.input_reason or '棋盘来源异常')
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
            if not self.waiting_frame:
                self.controller.pause_updates()
            self.waiting_frame = True
            return False
        if not self.waiting_frame or self.brake_sample is not None:
            self.waiting_frame = True
            self.recovery_weight = 0.
            self.revoke_visual_command()
            self.unverified_since = None
        if self.reopening:
            # Cancel an in-flight open on the first stale frame; keep any
            # mechanical retreat direction and obtain a new stop generation.
            self.enter_boundary(self.boundary)
        elif expired and not self.boundary:
            reason = DELAY_HOLD if delayed else LOST_HOLD
            self.enter_boundary(reason)
        elif expired and self.boundary == DELAY_HOLD and not delayed:
            self.boundary = LOST_HOLD
            self.event('target_lost', LOST_HOLD)
        return False

    def on_feedback(self, msg):
        previous = self.feedback.stamp
        super().on_feedback(msg)
        if self.feedback.stamp > previous and not self.feedback.error:
            self.inputs.feedback.append((self.feedback.stamp, matrix(self.model.pose(self.feedback.q))))
            if self.inputs.pending:
                self.consume()

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
        if following and self.filtered_center is not None and 0 < gap_since_exposure <= self.visual['max_age_s']:
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
                self.deadlines[self.approved_revision] = min(received+self.visual['max_age_s'],
                    time.monotonic() + self.visual['max_age_s']-(self.clock_s()-stamp))
            self.frozen_revision = None
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
        self.deadlines[self.revision] = min(received+self.visual['max_age_s'],
            time.monotonic()+self.visual['max_age_s']-(self.clock_s()-stamp))
        self.exposure_stamps[self.revision] = stamp
        for rev in list(self.deadlines):
            if rev < self.revision-64:
                del self.deadlines[rev]
                self.exposure_stamps.pop(rev, None)
        if reason:
            self.reject(reason)
            return
        if self.ik_wait_reason:
            # A new exposure may have exactly the same numeric TCP target.
            self.valid_revision = -1
        if self.pending is None and self.valid_revision != self.revision:
            self.validation_tick()
        self.count_retreat_frame()

    def allows_pending_retreat_target(self):
        return self.boundary in VISION_HOLDS or self.boundary.startswith(('J', NATIVE_BOUND))

    def count_retreat_frame(self):
        if not self.boundary or self.reopening or not self.confirmed() or not self.approval_fresh():
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
        moved = (self.boundary in VISION_HOLDS or
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
        if not self.active or self.boundary:
            return
        if not (self.fresh() or self.frozen_ready()) or not self.approval_fresh():
            self.start_pending = True
            self.reason = '等待新鲜棋盘目标通过验证'
            return
        self.start_pending = False
        q, actual = self.actual()
        velocity = self.start_step(q, actual)
        reason = self.blocked(q, velocity)
        if reason:
            self.enter_boundary(reason, joint_step=(q, velocity))
            return
        self.reason = '正在准备视觉跟踪'
        self.request_motion(True)

    def stop_tracking(self, reason):
        self.frozen_revision = None
        self.brake_sample = None
        self.filtered_center = None
        self.waiting_frame = False
        self.ik_wait_reason = ''
        self.boundary, self.reopening, self.retreat = '', False, 0
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
        self.log.write(json.dumps(dict(event=event, reason=reason, stamp=now,
            source_stamp=self.stamp, age_s=now-self.stamp if self.stamp else None,
            receive_age_s=time.monotonic()-self.received if self.received else None,
            input_detail=self.input_reason, source_ok=self.source_ok, bracket_s=self.gap,
            guard=self.guard_state, guard_generation=self.guard_info.get('stop_generation'),
            generation=self.generation, epoch=self.joint_epoch, **step), ensure_ascii=False)+'\n')
        self.get_logger().info(reason)

    def request_motion(self, enable):
        if not self.vision_gate.service_is_ready():
            self.stop_tracking('视觉 Guard 服务中断')
            return
        request = VisionMotion.Request(enable=enable, stop_generation=self.guard_info['stop_generation'])
        self.motion_future = self.vision_gate.call_async(request)
        self.motion_requested = time.monotonic()
        self.motion_future.add_done_callback(lambda f: self.motion_done(f))

    def motion_done(self, future):
        if future is not self.motion_future:
            return
        self.motion_future = None
        try:
            result = future.result()
            if not result.success:
                self.stop_tracking('运动许可请求失败 ' + result.message)
        except Exception as exc:
            self.stop_tracking(f'运动许可服务异常 {exc}')

    def on_guard(self, msg):
        data = json.loads(msg.data)
        previous = self.guard_state
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
        elif self.guard_state == 'running' and previous != 'running':
            self.brake_sample = None
            self.reset_controller()

    def confirmed(self):
        g = self.guard_info
        return (self.guard_state in ('holding', 'paused') and
                g.get('stop_generation') == self.generation and g.get('stop_settled') and
                g.get('gate_closed') and g.get('stop_confirmed'))

    def enter_boundary(self, reason, already_closed=False, joint_step=None):
        self.frozen_revision = None
        self.brake_sample = None
        self.retreat = 0
        self.first_step = None
        holding = self.boundary and not self.reopening and not already_closed
        # A mechanical limit supersedes a camera hold. Its existing close
        # request and generation still own the stop acknowledgement.
        upgrading = holding and self.boundary in VISION_HOLDS and reason not in VISION_HOLDS
        if holding and not upgrading:
            return
        first = not self.boundary
        self.boundary, self.reopening = reason, False
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
        if first or upgrading:
            event = {DELAY_HOLD: 'delayed_input_hold', LOST_HOLD: 'target_lost',
                     ARRIVED_HOLD: 'frozen_target_reached'}.get(reason, 'boundary_entered')
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
        if retry_reason:
            self.revoke_visual_command()
            self.valid_revision = self.revision
            self.reachability = retry_reason + '，等待新帧验证'
            if waiting_to_start:
                self.reachability = '运动许可关闭，当前目标无可用逆解，请将棋盘移回可达位置'
            if self.reopening:
                self.enter_boundary(self.boundary)
                self.ik_wait_reason = self.reachability
                return
            if self.active and not self.ik_wait_reason:
                self.event('ik_wait', self.reachability)
            self.ik_wait_reason = self.reachability
            if waiting_to_start or (self.active and self.unverified_since is None):
                self.unverified_since = time.monotonic()
            return
        self.ik_wait_reason = ''
        self.valid, self.valid_revision = False, self.revision
        self.approved_target = self.preview_q = None
        self.reachability = reason
        self.clear_ik_seeds(cancel_pending=True)
        if not self.active:
            return
        if ('工作空间' in reason or '可达半径' in reason or '限位保护区' in reason or
                reason == '目标超出可达范围'):
            self.enter_boundary(reason)
        else:
            self.stop_tracking(reason)

    def candidate_boundary(self, pose):
        return workspace_reason(pose[:3], self.visual, self.model.radius) or self.reach.reason(pose)

    def validation_tick(self):
        if self.target is None or not (self.fresh() or self.frozen_ready()):
            return
        if self.candidate_boundary(self.target):
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
        super().ik_done(future, revision)
        if self.approval_fresh():
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
        return self.joint_velocity(q, twist)

    def joint_velocity(self, q, twist):
        return tracking_velocity(q, twist, self.model, self.config, self.scale,
                                 self.preview_q if self.approved_target is not None else None)

    def try_retreat(self, q, actual):
        if (self.reopening or self.motion_future or self.first_step or not self.confirmed() or
                self.retreat < self.visual['retreat_frames'] or
                (not self.allows_pending_retreat_target() and self.approved_revision != self.revision) or
                not self.approval_fresh() or not self.validity.service_is_ready()):
            return
        velocity = self.start_step(q, actual)
        if not self.retreat_step_ok(q, actual, velocity):
            self.retreat = 0
            return
        velocity = velocity*self.bound_speed_scale(q, velocity)
        req = GetStateValidity.Request(group_name='arm')
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = (q+velocity/self.config['rate']).tolist()
        self.first_step = self.validity.call_async(req)
        self.step_requested = time.monotonic()
        rev, generation = self.approved_revision, self.generation
        self.first_step.add_done_callback(lambda f: self.retreat_checked(f, rev, generation))

    def retreat_checked(self, future, rev, generation):
        if future is not self.first_step:
            return
        self.first_step = None
        if not (self.active and self.boundary and self.confirmed() and generation == self.generation and
                self.retreat >= self.visual['retreat_frames'] and rev == self.approved_revision and
                (rev == self.revision or self.allows_pending_retreat_target()) and
                self.fresh() and self.approval_fresh()):
            return
        try:
            if not future.result().valid:
                self.stop_tracking('退回首段碰撞检查未通过')
                return
            q, actual = self.actual()
            if not self.retreat_step_ok(q, actual, self.start_step(q, actual)):
                self.retreat = 0
                return
        except (RuntimeError, ValueError) as exc:
            self.stop_tracking(str(exc))
            return
        self.reopening = True
        self.request_motion(True)

    def retreat_step_ok(self, q, actual, velocity):
        if not self.inward(q, velocity):
            return False
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
        position = self.braking_position(q, velocity)
        _, scale = limit_joint_velocity(position, velocity, self.model.lower, self.model.upper,
            self.config['joint_acceleration'], self.config['joint_margin'], self.config['command_timeout'])
        return scale

    def braking_position(self, q, velocity):
        # Both nodes run in the same local launch. Use Guard's validated knot
        # only to reduce outward motion, never as a replacement for actual FK.
        ref = self.guard_info.get('servo_reference_rad')
        if (self.guard_state != 'running' or ref is None or not
                0 <= time.monotonic()-self.guard_info['reference_received'] <= self.config['command_timeout']):
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
            if x + v*horizon <= low and v <= 0:
                return f'工作空间 {axis}下限'
            if x + v*horizon >= high and v >= 0:
                return f'工作空间 {axis}上限'
        return ''

    def brake_weight(self):
        if (self.brake_sample is None or not self.active or self.guard_state != 'running' or
                self.boundary or self.start_pending or self.reopening or not self.source_ok or
                self.ik_wait_reason or
                (self.input_reason and not self.input_reason.startswith('观测已过期'))):
            return 0.
        _, deadline, _ = self.brake_sample
        now = time.monotonic()
        overdue = max(0., now-deadline, self.clock_s()-self.stamp-self.visual['max_age_s'],
                      now-self.received-self.visual['max_age_s'])
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
        self.last_command = command*weight
        self.publish_twist(self.last_command, joint_step=(q, velocity))
        now = time.monotonic()
        deadline = min(deadline, self.received+self.visual['max_age_s'],
                       now+self.visual['max_age_s']-(self.clock_s()-self.stamp))
        if np.any(velocity) and deadline > now:
            self.brake_sample = (velocity.copy(), deadline, self.scale)
        else:
            self.brake_sample = None
        return velocity

    def brake_on_gap(self, q, actual, weight):
        # Only slow an already issued command. Limit travel to its original
        # reference and check the current measured joints on each braking tick.
        velocity, _, scale = self.brake_sample
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
        reason = self.blocked(q, velocity)
        if reason:
            self.enter_boundary(reason, joint_step=(q, velocity))
            self.last_command = np.zeros(6)
            return
        scale = self.bound_speed_scale(q, velocity)
        velocity = velocity*scale
        self.last_command = twist*(self.recovery_weight*scale)
        if np.any(velocity):
            self.publish_twist(self.last_command, joint_step=(q, velocity))
        elif self.fresh():
            # A reversal may require zero while the new IK is in flight.
            # Leave that validation alive so the next tick can use its result.
            self.brake_sample = None
            self.publish_twist(self.last_command)
        else:
            self.revoke_visual_command()

    def tick(self):
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
                    raise ValueError('退回首段碰撞检查超时')
                q, actual = self.actual()
                ready = self.observation_ready()
                awaiting_ik = (ready and self.brake_sample is not None and
                               self.approved_target is not None and not self.approval_fresh() and
                               time.monotonic() >= self.linear_until)
                if (not ready or awaiting_ik) and (weight := self.brake_weight()) > 0.:
                    if awaiting_ik:
                        self.controller.pause_updates()
                    self.brake_on_gap(q, actual, weight)
                    return
                if (ready and not self.start_pending and not self.boundary and self.unverified_since is not None and
                        time.monotonic()-self.unverified_since > self.visual['tracking_loss_timeout_s']):
                    reason = self.ik_wait_reason or '未取得新鲜的逆解'
                    if self.frozen_revision is not None:
                        self.enter_boundary(LOST_HOLD)
                        return
                    raise ValueError(f'视觉目标验证超时：{reason}，请重新开始')
                if not ready:
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
                            if self.boundary in (LOST_HOLD, ARRIVED_HOLD):
                                self.event('target_reacquired', '棋盘重新可见，新目标已验证，恢复跟踪')
                            elif self.boundary == DELAY_HOLD:
                                self.event('delayed_input_recovered', '新鲜棋盘位姿已验证，Guard 已恢复')
                            else:
                                self.event('boundary_recovered', '已退回，Guard 已恢复')
                            self.boundary, self.reopening, self.retreat = '', False, 0
                            self.reason = '视觉跟踪中'
                            command = candidate
                        elif np.any(velocity) or not self.retreat_step_ok(q, actual, self.start_step(q, actual)):
                            self.enter_boundary(self.boundary)
                    else:
                        self.try_retreat(q, actual)
                elif self.start_pending and self.approval_fresh():
                    self.begin_tracking()
                elif self.guard_state == 'running' and (
                        self.approval_fresh() or time.monotonic() < self.linear_until):
                    self.unverified_since = None
                    if self.approval_fresh():
                        self.linear_until = 0.
                        command = self.control_twist(actual)
                        if self.frozen_revision is not None and self.frozen_settled(q, actual):
                            self.enter_boundary(ARRIVED_HOLD)
                            return
                    else:
                        limit = self.config['linear_speed']*self.scale
                        command[:3] = self.previous_linear*min(1., limit/max(np.linalg.norm(self.previous_linear), 1e-12))
                    velocity = self.joint_velocity(q, command)
                    reason = self.blocked(q, velocity)
                    if reason:
                        self.enter_boundary(reason, joint_step=(q, velocity))
                        command[:] = 0.
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
            self.brake_sample = None
            self.bound_scale = 1.
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

    def visual_status(self):
        g = self.goal
        detail = self.reason
        input_detail = self.input_reason
        gap_detail = '短时迟帧，减速' if self.waiting_frame and self.brake_sample is not None else '等待新鲜棋盘帧 · 零速'
        if self.boundary in VISION_HOLDS:
            phase = ('新帧已到，正在验证' if self.retreat or self.reopening else
                     '已停稳，等待棋盘' if self.confirmed() else '已请求保持，等待停稳')
            detail = f'{self.boundary} · {phase}'
        elif self.boundary:
            phase = ('已退回，正在验证' if self.reopening or self.retreat else
                     '机械臂已停稳，等待棋盘退回' if self.confirmed() else '已请求暂停，等待停稳')
            detail = f'边界超限 {self.boundary} · {phase}'
        elif self.active and self.goal is None:
            detail = '跟踪已开启，等待棋盘出现'
        elif self.frozen_revision is not None:
            detail = '棋盘暂不可见，继续前往最后可见目标' if self.approval_fresh() else '棋盘暂不可见，正在验证最后目标'
        elif self.active and self.waiting_frame:
            detail = gap_detail
        elif self.active and self.ik_wait_reason:
            detail = self.ik_wait_reason + ' · 零速'
        elif self.active and self.guard_state == 'running' and g is not None:
            detail = ('接近关节边界，跟踪减速 · ' if self.bound_scale < .999 else '') + LABEL[g.correcting]
        elif not self.active and self.reason == '等待反馈' and (issue := self.start_issue()):
            detail = issue
        elif not self.active and self.fresh():
            if self.reason.startswith(('棋盘观测', '棋盘来源异常', '关节反馈来源异常')):
                detail = '棋盘识别已恢复 · 等待开启跟踪'
                input_detail = '上次停止：'+self.reason
            else:
                prefix = '' if self.reason == '等待反馈' else self.reason+' · '
                detail = prefix+'预览 · '+LABEL[g.correcting]
        has_measurement = g is not None and self.frozen_revision is None and self.fresh()
        tcp_center = (self.geometry.ec[:3, :3] @ g.camera_center + self.geometry.ec[:3, 3]
                      if has_measurement else None)
        if has_measurement:
            tolerance = self.visual['normal_tolerance_deg']
            if g.alpha is not None:
                detail += f' | 垂直偏差 {g.alpha:.1f}°（容差 {tolerance:g}°）'
            detail += f' | TCP 距离 {np.linalg.norm(tcp_center)*100:.1f} cm，横偏 {np.linalg.norm(tcp_center[:2])*1000:.1f} mm'
        return dict(state=detail, servo_ready=self.switched and self.ik.service_is_ready() and self.gate.service_is_ready(),
            visual_branch=LABEL[g.correcting] if g is not None else None,
            angle_deg=g.alpha if has_measurement else None,
            boundary='' if self.boundary in VISION_HOLDS else self.boundary,
            waiting_for_target=self.active and (g is None or self.boundary in (LOST_HOLD, ARRIVED_HOLD)),
            frozen_target=self.frozen_revision is not None, frozen_revision=self.frozen_revision,
            last_missing_exposure=self.miss_stamp or None,
            delayed_input_hold=self.boundary == DELAY_HOLD,
            retreat_frames=self.retreat, source_stamp=self.stamp, age_s=self.clock_s()-self.stamp if g else None,
            receive_age_s=time.monotonic()-self.received if self.received else None,
            waiting_frame=self.waiting_frame, braking_on_gap=self.waiting_frame and self.brake_sample is not None,
            tcp_center_m=tcp_center.tolist() if tcp_center is not None else None, source_ok=self.source_ok,
            camera_center_m=g.camera_center.tolist() if g else None, input_detail=input_detail)

    def diagnostics(self):
        status = self.visual_status()
        status.update(active=self.active, guard=self.guard_state, generation=self.generation,
                      joint_bound_speed_scale=self.bound_scale,
                      approved=self.approved_target.tolist() if self.approved_target is not None else None,
                      guard_stop={k: self.guard_info.get(k) for k in (
                          'stop_generation', 'gate_closed', 'stop_settled', 'stop_confirmed')},
                      received=self.received, bracket_s=self.gap, command=self.last_command.tolist())
        if self.feedback.q is not None:
            status.update(measured_joints_rad=self.feedback.q.tolist(),
                          requested_joint_speed_rad_s=self.last_joint_speed.tolist())
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
