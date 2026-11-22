from collections import Counter
import json
import time
from types import SimpleNamespace as NS
from threading import Lock
import unittest
from unittest.mock import Mock, patch

import cv2
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation
from servo_tracking.node import Backend, pose_message
from servo_tracking.guard import Guard

from scripts.task.visual_servo.backend import DELAY_HOLD, IK_HOLD, LOST_HOLD, NATIVE_BOUND, STALLED_HOLD, VisualBackend
from scripts.task.visual_servo.geometry import (CameraTarget, Goal, stabilize_goal, TargetGeometry, WristReach, calibration,
    chessboard_target, joint_boundary, matrix, nearby_poses, pose7, settings, smooth_center, workspace_reason)
from scripts.task.visual_servo.observation import CenterVelocity, Observations
from scripts.task.visual_servo.progress import FrozenProgress
from scripts.task.visual_servo.scene import BoardScene
from scripts.tools.handeye_calibration.chessboard import (
    BoardSearch, CaptureClock, ChessboardDetector, object_grid, ordered_corners)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = settings()
        self.ec, _, _ = calibration(self.cfg, None, sim=True)
        self.be = matrix([.2, -.1, .4, *Rotation.from_euler('xyz', [14, 55, 7], degrees=True).as_quat()])
        self.scene = BoardScene(self.cfg, self.ec, self.be)
        self.tracker = TargetGeometry(self.cfg, self.ec)

    def observe(self, angle, roll=0, tilt_axis='x'):
        offset = np.eye(4)
        offset[:3, :3] = (Rotation.from_euler(tilt_axis, angle, degrees=True) *
                          Rotation.from_euler('z', roll, degrees=True)).as_matrix()
        cg = self.scene.camera_board(self.be, offset)
        return self.tracker.measure(self.be, chessboard_target(cg, self.cfg))

    def test_endpoints_both_directions_composite_and_roll(self):
        for axis in 'xy':
            for angle in (-7, -5, -3, 0, 3, 5, 7):
                for roll in (-30, 0, 30):
                    goal = self.observe(angle, roll, axis)
                    self.assertAlmostEqual(goal.alpha, abs(angle), places=8)
                    self.assertEqual(goal.correcting, abs(angle) > 5)
                    # Reconstruct the measured board in the commanded tool frame.
                    tcp = matrix(goal.pose)
                    np.testing.assert_allclose(tcp[:3, :3].T @ (goal.center-tcp[:3, 3]),
                                               [0, 0, self.cfg['distance_m']], atol=1e-12)
                    residual = np.rad2deg(np.arccos(np.clip(matrix(goal.pose)[:3, 2] @ goal.normal, -1., 1.)))
                    self.assertAlmostEqual(residual, 4 if abs(angle) > 5 else abs(angle), places=5)
        self.assertTrue(self.observe([4., 4.], tilt_axis='xy').correcting)

    def test_stationary_noise_slow_accumulated_motion_and_tilt_boundary(self):
        base = self.observe(4.85)
        for offset in (.0002, .0005, .0009):
            raw = self.observe(4.97)
            raw.center = base.center + [offset, 0., 0.]
            raw.pose[:3] += [offset, 0., 0.]
            stable = stabilize_goal(raw, base, self.cfg)
            np.testing.assert_array_equal(stable.pose, base.pose)
            self.assertAlmostEqual(stable.alpha, 4.97)
        raw.center = base.center + [.0012, 0., 0.]
        self.assertIs(stabilize_goal(raw, base, self.cfg), raw)
        crossed = self.observe(5.01)
        self.assertIs(stabilize_goal(crossed, base, self.cfg), crossed)
        self.assertTrue(crossed.correcting)
        returned = self.observe(5.)
        self.assertIs(stabilize_goal(returned, crossed, self.cfg), returned)
        self.assertFalse(returned.correcting)

    def test_flip_rejected(self):
        self.observe(1)
        with self.assertRaisesRegex(ValueError, '翻转'):
            self.observe(179)

    def test_unreachable_workspace_target_yields_a_shortened_interior_step(self):
        actual = np.array([.689, .1, .3, 0., 0., 0., 1.])
        outside = actual.copy()
        outside[0] = .80
        reason = lambda pose: workspace_reason(pose[:3], self.cfg, 2.)
        steps = list(nearby_poses(outside, actual, .5, .001, .01, boundary=reason))
        self.assertTrue(steps)
        self.assertLessEqual(steps[0][1][0], .70)
        self.assertGreater(steps[0][1][0], .699)
        self.assertTrue(all(not reason(pose) and pose[0] > actual[0] for _, pose in steps))
        np.testing.assert_allclose([pose[0]-actual[0] for _, pose in steps[1:]], [.010, .004, .0016])
        actual[0] = .70
        self.assertEqual(list(nearby_poses(outside, actual, .5, .001, .01, boundary=reason)), [])

    def test_center_filter_uses_exposure_interval_and_preserves_standoff_and_raw_angle(self):
        goal = self.observe(6.)
        previous = goal.center + [.006, -.001, .002]
        part = smooth_center(goal, previous, .02, .05)
        whole = smooth_center(goal, previous, .05, .05)
        np.testing.assert_allclose(smooth_center(goal, part.center, .03, .05).center, whole.center)
        np.testing.assert_allclose(whole.center - whole.pose[:3], goal.center-goal.pose[:3])
        np.testing.assert_array_equal(whole.pose[3:], goal.pose[3:])
        self.assertEqual((whole.alpha, whole.correcting), (goal.alpha, True))

    def test_same_center_from_three_sources_shares_one_tcp_target(self):
        board_pose = self.scene.camera_board(self.be, np.eye(4))
        board = chessboard_target(board_pose, self.cfg)
        point = CameraTarget(board.center)
        flower = CameraTarget(board.center, 3*board.direction)
        # Each observation enters with its original exposure, including a
        # centre-only detector which has no quaternion to publish.
        poses = []
        for observation in (board, point, flower):
            stream = Observations('camera', self.cfg)
            stream.receive_target('camera', 8., observation, 8.02, 12.)
            stream.feedback.append((8., self.be))
            stamp, receipt, measured, target, _ = stream.match(8.022, 12.002)
            self.assertEqual((stamp, receipt), (8., 12.))
            tracker = TargetGeometry(self.cfg, self.ec)
            goal = tracker.measure(measured, target)
            poses.append(goal.pose)
            if observation is point:
                self.assertIsNone(goal.normal)
                self.assertIsNone(goal.alpha)
                shifted = tracker.measure(measured, CameraTarget(point.center + [.025, 0., 0.]))
                np.testing.assert_allclose(shifted.pose[3:], goal.pose[3:])
                np.testing.assert_allclose(shifted.pose[:3]-goal.pose[:3],
                                           (self.be @ self.ec)[:3, 0]*.025, atol=1e-12)
        np.testing.assert_allclose(poses, [poses[0]]*3, atol=1e-12)
        tilted_pose = np.eye(4)
        tilted_pose[:3, :3] = Rotation.from_euler('y', 12, degrees=True).as_matrix()
        tilted = chessboard_target(self.scene.camera_board(self.be, tilted_pose), self.cfg)
        self.assertTrue(tracker.measure(self.be, CameraTarget(tilted.center, tilted.direction)).correcting)
        with self.assertRaises(ValueError):
            CameraTarget([0., 0., .35], [0., 0., 0.])
        with self.assertRaises(ValueError):
            CameraTarget([0., 0., np.nan])

    def test_source_time_matching(self):
        stream = Observations('camera', self.cfg)
        msg = PoseStamped(pose=pose_message([0., 0., .35, 0., 0., 0., 1.]))
        msg.header.frame_id, msg.header.stamp.sec = 'camera', 100
        stream.receive(msg, 100.015, 55)
        stream.feedback.append((99.99, self.be))
        self.assertIsNone(stream.match(100.02, 55.01))
        stream.feedback.append((100.01, self.be))
        match = stream.match(100.02, 55.01)
        np.testing.assert_allclose(match[2], self.be)
        self.assertAlmostEqual(match[4], .02)
        with self.assertRaisesRegex(ValueError, '乱序'):
            stream.receive(msg, 100.03, 55.02)
        msg.header.frame_id = 'base_link'
        with self.assertRaisesRegex(ValueError, '坐标系'):
            stream.receive(msg, 100.04, 55.03)
        msg.header.frame_id, msg.header.stamp.sec = 'camera', 101
        stream.receive(msg, 101.01, 56)
        self.assertRaises(ValueError, stream.match, 101.16, 56.02)
        stream.feedback.append((102.03, self.be))
        msg.header.stamp.sec = 102
        stream.receive(msg, 102.03, 57)
        self.assertRaisesRegex(ValueError, '间隔', stream.match, 102.04, 57.01)

    def test_wrist_envelope_contains_real_fk(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel
        model = RobotModel(URDF.read_text(), tip='link6')
        reach = WristReach(model)
        rng = np.random.default_rng(711)
        for q in rng.uniform(model.lower, model.upper, size=(200, 6)):
            self.assertEqual(reach.reason(model.pose(q)), '')
        # Inside the coarse base-centred radius but beyond the wrist shell.
        pose = [0.72, 0., .12, 0., 0., 0., 1.]
        self.assertLess(np.linalg.norm(pose[:3]), model.radius)
        self.assertIn('腕心', reach.reason(pose))

    def test_directional_limits(self):
        q = np.array([.99, -.99, .4])
        lo, hi = -np.ones(3), np.ones(3)
        self.assertEqual(joint_boundary(q, lo, hi, .015), 'J1 上限、J2 下限')
        self.assertFalse(joint_boundary(q, lo, hi, .015, [-.1, .1, 0], .2))
        self.assertEqual(joint_boundary(q, lo, hi, .015, [-.1, -.1, 0], .2), 'J2 下限')
        self.assertFalse(joint_boundary(q, lo, hi, .015, [0., 0., .1], .2))
        self.assertIn('X上限', workspace_reason([3., .0, .0], self.cfg, 1))


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        self.s = s = object.__new__(VisualBackend)
        s.mode = 'sim'
        s.retreat_stamp = 0.
        s.filtered_center = None
        s.brake_sample, s.recovery_weight, s.input_reason = None, 1., ''
        s.visual = settings()
        s.frozen_progress = FrozenProgress(s.visual)
        s.approach_progress = FrozenProgress(s.visual)
        s.approach_goal = None
        s.center_velocity = CenterVelocity(s.visual)
        s.feedforward_samples = {}
        s.linear_feedforward = np.zeros(3)
        s.view = s.view_margin = None
        s.view_source_ok = s.view_limited = False
        s.miss_stamp = s.start_after = s.miss_received = 0.
        s.miss_source_ok = True
        s.frozen_revision = None
        s.raw_goal = None
        s.goal = NS(center=np.zeros(3), normal=np.array([0., 0., 1.]))
        s.guard_info = dict(stop_generation=4, epoch=2, gate_closed=False, stop_settled=False,
                            stop_confirmed=False)
        s.active = True
        s.guard_state = 'running'
        s.joint_velocity_anchor = s.measured_joint_motion = s.queued_joint_motion = None
        s.joint_brake_reason = ''
        s.last_joint_speed = np.zeros(6)
        s.bound_scale = 1.
        s.joint_constraint_scale = 1.
        s.workspace_constraint_scale = 1.
        s.boundary = ''
        s.preparing = s.reopening = False
        s.motion_future = s.first_step = None
        s.waiting_frame = False
        s.unverified_since = None
        s.ik_wait_reason = ''
        s.nearby_revision, s.nearby_motion = -1, ''
        s.deadlines = {}
        s.switched = True
        s.ik, s.gate = Mock(), Mock()
        s.exposure_stamps = {}
        s.preview_q = None
        s.ik_context = None
        s.ik_best_q = None
        s.valid_revision = -1
        s.inputs = Observations('camera', s.visual)
        s.controller = Mock()
        s.start_pending = False
        s.source_ok = True
        s.clock_s = Mock(return_value=100.16)
        s.stamp, s.received = 100., time.monotonic()-.16
        s.event = Mock()
        s.request_motion = Mock()
        s.clear_ik_seeds = Mock()
        s.publish_twist = Mock()
        s.reset_controller = Mock()
        s.approval_fresh = Mock(return_value=True)
        s.fresh = Mock(return_value=True)
        s.validity = Mock()
        s.stop_tracking = Mock()
        s.approved_target = np.array([0., 0., 0., 0., 0., 0., 1.])
        s.actual = Mock(return_value=(np.zeros(6), s.approved_target.copy()))
        s.start_step = Mock(return_value=np.ones(6))
        s.blocked = Mock(return_value='')
        s.config = dict(rate=100., joint_acceleration=1.5, joint_margin=.015, command_timeout=.2)
        s.model = NS(lower=np.full(6, -3.), upper=np.full(6, 3.))
        s.approved_revision = s.revision = 28

    def prepared_retreat(self):
        s = self.s
        self.assertTrue(s.preparing)
        self.assertIsNone(s.first_step)
        s.guard_state = 'prepared'
        s.clock_s.return_value = s.stamp + .02
        s.received = time.monotonic() - .01
        s.deadlines[s.approved_revision] = s.observation_deadline(s.stamp, s.received)
        s.try_retreat(*s.actual())
        self.assertIsNotNone(s.first_step)
        return s.first_step

    def complete_first_step(self, future):
        future.result.return_value.valid = True
        future.add_done_callback.call_args.args[0](future)

    def assert_open_requested(self):
        s = self.s
        self.assertEqual(s.request_motion.call_args.args, (True,))
        self.assertEqual(s.request_motion.call_args.kwargs['prepared_epoch'], s.guard_info['epoch'])

    def test_arming_with_an_expired_preview_never_opens_the_gate(self):
        s = self.s
        s.active, s.reason, s.retreat = False, '等待反馈', 0
        s.guard_state, s.check_source, s.vision_gate = 'idle', Mock(), Mock()
        s.panel_seen, s.geometry = time.monotonic(), Mock()
        s.ik, s.gate = Mock(), Mock()
        s.fresh = VisualBackend.fresh.__get__(s)
        s.target = s.approved_target.copy()
        reply = s.start(None, NS(success=False, message=''))
        self.assertTrue(reply.success)
        self.assertTrue(s.active and s.start_pending)
        self.assertIsNone(s.approved_target)
        self.assertIsNone(s.target)
        self.assertTrue(s.visual_status()['waiting_for_target'])
        self.assertFalse(s.observation_ready())
        s.request_motion.assert_not_called()

    def test_frozen_motion_requires_advancing_misses_and_exact_ik(self):
        s = self.s
        s.fresh = VisualBackend.fresh.__get__(s)
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        s.inputs.last_input = s.stamp
        header = NS(frame_id='camera', stamp=NS(sec=100, nanosec=140_000_000))
        s.on_miss(header)
        self.assertTrue(s.observation_ready())
        self.assertTrue(s.approval_fresh())
        s.approved_revision -= 1
        self.assertFalse(s.approval_fresh())
        f = s.pending = Future()
        s.ik_done(f, s.approved_revision)
        s.clear_ik_seeds.assert_called_with(cancel_pending=True)
        s.approved_revision = s.revision
        with patch.object(Backend, 'ik_done') as accept:
            f.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS)))
            s.ik_context = dict(vision_deadline=time.monotonic()-1)
            s.ik_done(f, s.revision)
            accept.assert_called_once_with(f, s.revision)
        s.clock_s.return_value += .16
        s.on_miss(header)  # A repeated blank image never renews permission.
        self.assertFalse(s.observation_ready())
        self.assertFalse(s.approval_fresh())
        self.assertTrue(s.waiting_frame)
        self.assertIsNone(s.brake_sample)
        s.request_motion.assert_not_called()
        s.clock_s.return_value += .10
        self.assertFalse(s.observation_ready())
        self.assertEqual(s.boundary, LOST_HOLD)
        s.request_motion.assert_called_once_with(False)
        header.stamp.nanosec = 400_000_000
        s.on_miss(header)
        self.assertIsNone(s.frozen_revision)

    def test_missing_frame_cannot_override_a_boundary_or_a_newer_pose(self):
        s = self.s
        header = NS(frame_id='unrelated', stamp=NS(sec=100, nanosec=140_000_000))
        s.on_miss(header)
        self.assertEqual(s.miss_stamp, 0.)
        header.frame_id = 'camera'
        s.inputs.last_input = 100.15
        s.on_miss(header)
        self.assertEqual(s.miss_stamp, 0.)
        s.inputs.last_input = s.stamp
        s.boundary = 'J4 上限'
        s.on_miss(header)
        self.assertIsNone(s.frozen_revision)
        s.event.assert_not_called()

    def test_guard_refusal_identifies_status_service_and_fault(self):
        s = self.s
        s.active, s.check_source, s.vision_gate = False, Mock(), Mock()
        s.guard_info = {}
        self.assertEqual(s.start(None, NS()).message, '等待 Guard 状态')
        s.guard_info, s.guard_state = {'state': 'idle'}, 'idle'
        s.vision_gate.service_is_ready.return_value = False
        self.assertIn('许可服务', s.start(None, NS()).message)
        s.guard_state, s.guard_reason = 'fault', '驱动反馈中断'
        self.assertIn('驱动反馈中断', s.start(None, NS()).message)
        s.request_motion.assert_not_called()

    def test_hold_and_resume_generation(self):
        s = self.s
        s.enter_boundary('J2 下限')
        s.enter_boundary('J2 下限')
        np.testing.assert_array_equal(s.publish_twist.call_args.args[0], np.zeros(6))
        s.request_motion.assert_called_once_with(False)
        self.assertEqual(s.event.call_count, 1)
        self.assertFalse(s.confirmed())
        s.guard_state = 'holding'
        s.guard_info.update(gate_closed=True, stop_settled=True, stop_confirmed=True)
        self.assertFalse(s.confirmed())  # Older acknowledgement.
        s.guard_info['stop_generation'] = 5
        q, pose = s.actual()
        s.approved_revision = s.revision
        s.retreat = 2
        s.try_retreat(q, pose)
        self.assertFalse(s.validity.call_async.called)
        s.retreat = 3
        s.start_step.return_value = -np.ones(6)
        s.try_retreat(q, pose)
        self.assertFalse(s.validity.call_async.called)
        s.start_step.return_value = np.ones(6)
        s.retreat = 3
        s.try_retreat(q, pose)
        s.request_motion.assert_called_with(True)
        s.validity.call_async.assert_not_called()
        self.complete_first_step(self.prepared_retreat())
        self.assert_open_requested()
        self.assertTrue(s.reopening)

    def test_running_zero_pose_stops_and_recovers_with_checked_joint_jog(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.cartesian_control import CartesianController
        from servo_tracking.core import RobotModel, load_config
        s = self.s
        s.config = load_config(s.visual['servo_config'])
        s.config['control'].update(linear_speed=.03, angular_speed=np.deg2rad(5.))
        s.model = RobotModel(URDF.read_text(), tip='link6')
        s.scale, s.dragging = .5, False
        s.controller = CartesianController(s.config['control'])
        s.start_step = VisualBackend.start_step.__get__(s)
        s.blocked = VisualBackend.blocked.__get__(s)
        q = np.zeros(6)
        actual = s.model.pose(q)
        s.actual.return_value = (q, actual)
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.reset_controller = s.controller.reset
        s.panel_seen = time.monotonic()
        s.approved_target_time = time.monotonic()
        # First approved pose in the 20:01 real recording after the J3 event.
        s.approved_target = np.array([.143083518, -.016165462, .185974854,
                                      -.030603759, .787946628, .028043190, .614342821])
        s.tick()
        self.assertTrue(s.active)
        self.assertEqual(s.boundary, 'J3 上限')
        s.request_motion.assert_called_once_with(False)
        self.assertIsNone(s.controller.last_command_time)
        s.guard_state = 'holding'
        s.guard_info.update(stop_generation=5, gate_closed=True, stop_settled=True, stop_confirmed=True)
        # Move the target to an FK pose inside both J2 and J3 limits.
        s.approved_target = s.model.pose([0., .06, -.06, 0., 0., 0.])
        s.goal.center = s.goal.center + s.approved_target[:3]-actual[:3]
        s.retreat_stamp = 0.
        for frame in range(1, 4):
            s.stamp = 100. + frame/30.
            s.count_retreat_frame()
            self.assertEqual(s.retreat, frame)
            s.try_retreat(q, actual)
            s.validity.call_async.assert_not_called()
        future = self.prepared_retreat()
        checked = s.validity.call_async.call_args.args[0].robot_state.joint_state.position
        self.complete_first_step(future)
        self.assert_open_requested()
        s.guard_state = 'running'
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.panel_seen = time.monotonic()
        s.tick()
        self.assertFalse(s.boundary)
        q_used, velocity = s.publish_twist.call_args.kwargs['joint_step']
        np.testing.assert_allclose(q_used + velocity/s.config['rate'], checked)
        self.assertGreater(velocity[1], 0.)
        self.assertLess(velocity[2], 0.)
        s.stop_tracking.assert_not_called()
        # The actual publisher must emit precisely the inspected joint step.
        s.twist, s.jog, s.get_clock = Mock(), Mock(), Mock()
        s.get_clock.return_value.now.return_value = Time(seconds=100)
        s.actual.reset_mock()
        Backend.publish_twist(s, s.last_command, joint_step=(q_used, velocity))
        s.actual.assert_not_called()
        np.testing.assert_array_equal(s.jog.publish.call_args.args[0].velocities, velocity)

    def test_native_j4_hold_with_lagging_feedback_and_unknown_axis_reversal(self):
        from servo_tracking.core import RobotModel, load_config
        from tests.visual_servo_kinematics import URDF
        s = self.s
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        # Guard feedback at the two native joint4 warnings in the 11:08 session.
        for q, speed, with_reference in (
                ([-.788836, 1.164553, -.985605, 1.719202, .509060, -1.799906],
                 [-.126874, .442730, -.004709, .228531, .267253, -.404514], False),
                ([.187850, 1.353258, -.384042, 2.186601, 1.387781, -2.278789],
                 [.314325, -.011076, .170402, .037008, .282353, -.197007], True)):
            q = np.array(q)
            actual = s.model.pose(q)
            s.actual.return_value = q, actual
            s.guard_state, s.boundary, s.reopening = 'running', '', False
            s.goal.pose = actual.copy()
            s.last_joint_speed = np.array(speed)
            s.last_command = s.model.jacobian(q) @ s.last_joint_speed
            # Construct a retreat frame from exactly the opposite wrist motion.
            retreat_pose = s.model.pose(q - .05*s.last_joint_speed)
            g = dict(state='paused', reason='joint bound', pause_cause='joint_bound',
                     stop_generation=7, epoch=5, gate_closed=True, stop_settled=False, stop_confirmed=False)
            if with_reference:
                ref = q.copy()
                ref[3] = s.model.upper[3] - s.config['joint_margin'] - .002
                g['joint_bound_reference_rad'] = ref.tolist()
            s.on_guard(NS(data=json.dumps(g)))
            s.stop_tracking.assert_not_called()
            self.assertEqual(s.boundary, NATIVE_BOUND + (' J4 上限' if with_reference else ''))
            self.assertTrue(s.active)
            s.approved_target = retreat_pose
            self.assertFalse(s.inward(q, s.last_joint_speed))
            self.assertTrue(s.inward(q, -s.last_joint_speed))
            if not with_reference:
                s.approved_target = actual
                self.assertFalse(s.inward(q, -s.last_joint_speed))
                s.approved_target = retreat_pose
            s.goal.center = s.hold_origin[0] + np.array([.01, 0., 0.])
            s.start_step.return_value = -s.last_joint_speed
            s.approved_revision = s.revision
            s.count_retreat_frame()
            self.assertEqual(s.retreat, 0)
            g.update(stop_settled=True, stop_confirmed=True)
            s.on_guard(NS(data=json.dumps(g)))
            for _ in range(3):
                s.stamp += 1/30.
                s.count_retreat_frame()
            s.try_retreat(q, actual)
            self.complete_first_step(self.prepared_retreat())
            self.assert_open_requested()
            s.motion_future = None

    def test_directional_braking_near_j4_preserves_reverse_and_otherwise_full_speed(self):
        s = self.s
        q = np.zeros(6)
        q[3] = s.model.upper[3] - .05
        v = np.array([.25, 0., 0., .8, 0., 0.])
        factor = s.bound_speed_scale(q, v)
        self.assertGreater(factor, 0.)
        self.assertLess(factor, 1.)
        self.assertEqual(s.bound_speed_scale(q, -v), 1.)
        self.assertEqual(s.bound_speed_scale(np.zeros(6), v), 1.)
        # The first 11:08 warning occurred well before physical feedback had
        # caught up with Servo. A fresh validated endpoint can only reduce speed.
        s.guard_info.update(servo_reference_rad=q.tolist(), reference_received=time.monotonic())
        self.assertEqual(s.bound_speed_scale(np.zeros(6), v), factor)
        self.assertEqual(s.bound_speed_scale(np.zeros(6), -v), 1.)
        s.guard_info['reference_received'] -= 1.
        self.assertEqual(s.bound_speed_scale(np.zeros(6), v), 1.)

    def test_uses_latest_admitted_trajectory_velocity_without_renewing_a_duplicate(self):
        s = self.s
        s.model.check_joints = np.asarray
        s.clock_s.return_value = 100.04
        msg = JointState(name=[f'joint{i}' for i in range(1, 7)], position=[0., 0., 0., 2.104, 0., 0.],
                         velocity=[0., 0., 0., .4, 0., 0.])
        msg.header.stamp.sec = 100
        msg.header.stamp.nanosec = 40_000_000
        with patch('scripts.task.visual_servo.backend.time.monotonic', return_value=200.) as now:
            s.on_accepted_trajectory(msg)
            self.assertEqual(s.braking_reference()[3], 2.104)
            self.assertAlmostEqual(s.fresh_queued_motion()[1][3], .4)
            now.return_value = 200.15
            s.on_accepted_trajectory(msg)
            self.assertEqual(s.fresh_queued_motion()[3], 200.)
            s.guard_info.update(servo_reference_rad=[0., 0., 0., 2.11, 0., 0.], reference_received=200.1)
            self.assertEqual(s.braking_reference()[3], 2.11)
            s.guard_info.pop('servo_reference_rad')
            now.return_value = 200.201
            self.assertIsNone(s.braking_reference())

    def test_measured_velocity_uses_source_intervals_and_resets_on_feedback_gap(self):
        from servo_tracking.core import FreshJoints, JOINTS
        s = self.s
        s.feedback = FreshJoints()
        s.model.pose = lambda q: np.array([0., 0., 0., 0., 0., 0., 1.])
        msg = JointState(name=JOINTS, position=[0.]*6)
        msg.header.stamp.sec = 100

        def sample(ns, j4):
            msg.header.stamp.nanosec = ns
            msg.position[3] = j4
            s.on_feedback(msg)

        sample(0, 0.)
        sample(3_000_000, .0015)
        self.assertIsNone(s.measured_joint_motion)
        sample(10_000_000, .005)
        self.assertAlmostEqual(s.measured_joint_motion[0][3], .5)
        sample(10_000_000, .02)
        self.assertAlmostEqual(s.measured_joint_motion[0][3], .5)
        sample(300_000_000, .015)
        self.assertIsNone(s.measured_joint_motion)

    def test_real_zero_edge_feedback_jitter_does_not_veto_inward_start(self):
        from servo_tracking.core import FreshJoints, JOINTS, RobotModel
        from tests.visual_servo_kinematics import URDF
        s = self.s
        s.mode = 'real'
        s.model = RobotModel(URDF.read_text(), tip='link6')
        s.config['zero_feedback_tolerance'] = np.deg2rad(.1)
        s.feedback = FreshJoints()
        msg = JointState(name=JOINTS, position=[0.]*6)
        msg.header.stamp.sec = 100
        s.on_feedback(msg)
        msg.header.stamp.nanosec = 10_000_000
        msg.position[1] = np.deg2rad(-.044)  # First J2 boundary event at 17:19:39.
        msg.position[2] = np.deg2rad(.01)
        s.on_feedback(msg)
        np.testing.assert_array_equal(s.feedback.q, msg.position)
        np.testing.assert_array_equal(s.measured_joint_motion[0], np.zeros(6))
        self.assertFalse(s.brake_near_joint_limit(s.feedback.q))
        s.request_motion.assert_not_called()

    def test_tick_brakes_logged_j4_approach_and_discards_old_jog(self):
        s = self.s
        s.model.upper[3] = np.deg2rad(127.)
        q, _ = s.actual.return_value
        q[3] = np.deg2rad(120.16)
        s.last_joint_speed[3] = np.deg2rad(11.89)
        measured = np.zeros(6)
        measured[3] = np.deg2rad(31.03)
        s.heartbeat, s.ensure_switched, s.observation_ready = Mock(), lambda: True, lambda: True
        s.panel_seen = 200.
        s.measured_joint_motion = measured, s.clock_s(), 200.
        s.brake_sample = (s.last_joint_speed.copy(), 200.1, 1.)
        s.linear_until = 200.1
        with patch('scripts.task.visual_servo.backend.time.monotonic', return_value=200.):
            s.tick()
        self.assertEqual(s.joint_brake_reason, 'J4 上限')
        np.testing.assert_array_equal(s.publish_twist.call_args.args[0], np.zeros(6))
        self.assertIsNone(s.brake_sample)
        self.assertEqual(s.linear_until, 0.)
        s.request_motion.assert_not_called()

    def test_queued_outward_motion_triggers_hold_even_after_reverse_jog(self):
        s = self.s
        s.model.upper[3] = np.deg2rad(127.)
        q, _ = s.actual.return_value
        q[3] = np.deg2rad(123.02)
        tail = q.copy()
        tail[3] = np.deg2rad(126.9)
        queued = np.zeros(6)
        queued[3] = np.deg2rad(30.)
        s.last_joint_speed[3] = np.deg2rad(-32.47)
        s.queued_joint_motion = tail, queued, s.clock_s()+.07, 200.
        with patch('scripts.task.visual_servo.backend.time.monotonic', return_value=200.):
            self.assertTrue(s.brake_near_joint_limit(q))
        self.assertEqual(s.boundary, 'J4 上限')
        self.assertTrue(s.active)
        self.assertEqual(s.retreat_axes, [(3, -1)])
        s.request_motion.assert_called_once_with(False)

    def test_new_outward_exposure_vetoes_pending_joint_retreat(self):
        s = self.s
        s.enter_boundary('J4 上限')
        s.approved_revision, s.revision = 30, 31
        s.goal.pose = np.array([.1, 0., 0., 0., 0., 0., 1.])
        q, actual = s.actual()
        v = np.array([0., 0., 0., -.1, 0., 0.])
        s.start_step.return_value = -v
        self.assertFalse(s.retreat_step_ok(q, actual, v))
        s.start_step.return_value = v
        self.assertTrue(s.retreat_step_ok(q, actual, v))

    def test_stationary_target_at_zero_pose_does_not_trigger_directional_alarm(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        s = self.s
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        q = np.zeros(6)
        actual = s.model.pose(q)
        s.approved_target = actual.copy()
        s.approved_target[0] += .25*s.config['control']['position_tolerance']
        s.approved_target_time, s.scale = time.monotonic(), .5
        velocity = VisualBackend.start_step(s, q, actual)
        np.testing.assert_array_equal(velocity, np.zeros(6))
        self.assertEqual(VisualBackend.blocked(s, q, velocity), '')

    def test_late_retreat_check_after_stop_and_new_board(self):
        s = self.s
        s.enter_boundary('工作空间 X上限')
        s.guard_state = 'holding'
        s.guard_info.update(stop_generation=5, gate_closed=True, stop_settled=True, stop_confirmed=True)
        s.approved_revision, s.retreat = s.revision, 4
        s.try_retreat(*s.actual())
        future = self.prepared_retreat()
        s.active = False
        self.complete_first_step(future)
        self.assertEqual(s.request_motion.call_count, 2)  # Hold and prepare.
        s.active, s.first_step = True, future
        s.revision += 1
        self.complete_first_step(future)
        self.assertEqual(s.request_motion.call_count, 2)

    def test_expired_exposure_during_preparation_waits_closed_for_fresh_first_step(self):
        s = self.s
        s.enter_boundary(LOST_HOLD)
        s.guard_state = 'holding'
        s.guard_info.update(stop_generation=5, gate_closed=True, stop_settled=True, stop_confirmed=True)
        s.approved_target = s.actual.return_value[1].copy()
        s.approved_revision, s.retreat = s.revision, 3
        s.try_retreat(*s.actual())
        expired = self.prepared_retreat()
        s.fresh = VisualBackend.fresh.__get__(s)
        s.clock_s.return_value = s.stamp+.16
        self.complete_first_step(expired)
        self.assertFalse(s.observation_ready())
        self.assertTrue(s.preparing and s.guard_state == 'prepared')
        self.assertFalse(s.reopening)
        self.assertEqual(s.request_motion.call_count, 2)
        s.stamp += .19
        s.received = time.monotonic()
        s.clock_s.return_value = s.stamp+.02
        s.approved_target = s.actual.return_value[1].copy()
        s.deadlines[s.revision] = s.observation_deadline(s.stamp, s.received)
        s.approved_revision, s.retreat = s.revision, 3
        s.try_retreat(*s.actual())
        self.complete_first_step(s.first_step)
        self.assert_open_requested()
        self.assertTrue(s.reopening)

    def test_visible_board_outside_workspace_can_enter_nearby_validation_after_loss(self):
        s = self.s
        s.target = np.r_[.75, 0., .25, 0., 0., 0., 1.]
        s.goal.pose = s.target
        s.model.radius = 1.
        s.try_nearby_pose = Mock(return_value=True)
        s.pending, s.last_target_change, s.joint_epoch = None, time.monotonic(), 0
        s.deadlines[s.revision] = time.monotonic()+.10
        s.enter_boundary(LOST_HOLD)
        s.guard_state = 'holding'
        s.guard_info.update(stop_generation=s.generation, gate_closed=True, stop_settled=True, stop_confirmed=True)
        s.validation_tick()
        self.assertEqual(s.try_nearby_pose.call_args.args[1], '工作空间 X上限')
        self.assertEqual(s.request_motion.call_count, 1)

    def test_retreat_counts_distinct_verified_exposures(self):
        s = self.s
        s.enter_boundary('工作空间 Y上限')
        s.guard_state = 'holding'
        s.guard_info.update(stop_generation=5, gate_closed=True, stop_settled=True, stop_confirmed=True)
        s.goal.center[0] = .02
        s.retreat_stamp = 0.
        s.approved_revision = s.revision
        for stamp, expected in [(1., 1), (1., 1), (2., 2), (3., 3)]:
            s.stamp = stamp
            s.count_retreat_frame()
            self.assertEqual(s.retreat, expected)
        s.retreat, s.stamp = 0, 4.
        s.approved_revision -= 1
        s.count_retreat_frame()
        self.assertEqual(s.retreat, 0)

    def test_tilt_handover_keeps_only_unexpired_translation(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        s = self.s
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        q = np.array(s.config['initial_joints'])
        s.actual.return_value = (q, s.model.pose(q))
        s.ensure_switched = lambda: True
        s.heartbeat = Mock()
        s.approval_fresh.return_value = False
        s.scale = .5
        s.panel_seen = time.monotonic()
        s.motion_future = s.first_step = None
        s.start_pending = False
        s.previous_linear = np.array([.005, .001, -.002])
        s.linear_until = time.monotonic()+.12
        s.tick()
        np.testing.assert_allclose(s.last_command, np.r_[s.previous_linear, [0., 0., 0.]])
        s.linear_until = time.monotonic()-.01
        s.tick()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))

    def test_missing_board_holds_but_missing_panel_cancels(self):
        s = self.s
        s.ensure_switched = lambda: True
        s.heartbeat = Mock()
        s.panel_seen = time.monotonic()
        s.fresh.return_value = False
        s.clock_s.return_value = s.stamp+s.visual['tracking_loss_timeout_s']+.01
        s.tick()
        self.assertEqual(s.boundary, LOST_HOLD)
        s.stop_tracking.assert_not_called()
        s.request_motion.assert_called_once_with(False)
        s.fresh.return_value, s.panel_seen = True, 0.
        s.tick()
        self.assertIn('面板', s.stop_tracking.call_args.args[0])

    def test_continuous_recognition_crosses_exposure_deadline(self):
        s = self.s
        # Prior exposure 100.000 arrived at 100.090. At 100.160 it no
        # longer authorizes motion, even though arrival silence is only 70 ms.
        s.received = time.monotonic()-.070
        s.fresh = VisualBackend.fresh.__get__(s)
        s.ensure_switched = lambda: True
        s.heartbeat = Mock()
        s.panel_seen = time.monotonic()
        s.deadlines = {28: time.monotonic()+1}
        s.approved_target = np.ones(7)
        s.start_pending = False
        s.tick()
        s.tick()
        self.assertTrue(s.waiting_frame)
        self.assertIsNone(s.approved_target)
        self.assertEqual(s.deadlines, {})
        s.clear_ik_seeds.assert_called_once_with(cancel_pending=True)
        s.stop_tracking.assert_not_called()
        s.request_motion.assert_not_called()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        # The next recognized exposure is only 100 ms after the preceding
        # one. It must pass a new IK check before leaving zero output.
        s.geometry = TargetGeometry(s.visual, np.eye(4))
        cg = matrix([0., 0., .35, 0., 0., 0., 1.])
        s.inputs = NS(match=Mock(return_value=(100.1, time.monotonic(), np.eye(4), chessboard_target(cg, s.visual), .005)))
        s.candidate_boundary = lambda pose: ''
        s.update_target = Mock()
        s.pending = None
        s.validation_tick = Mock()
        s.consume()
        self.assertFalse(s.waiting_frame)
        self.assertTrue(s.fresh())
        s.validation_tick.assert_called_once()
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        s.tick()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        s.stop_tracking.assert_not_called()
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        q = np.array(s.config['initial_joints'])
        s.actual.return_value = (q, s.model.pose(q))
        s.approved_target, s.approved_revision = s.goal.pose, s.revision
        new_command = np.array([.003, 0., 0., 0., 0., 0.])
        s.control_twist = Mock(return_value=new_command)
        s.scale = .5
        s.tick()
        np.testing.assert_allclose(s.last_command, new_command*.1)
        for _ in range(10):
            s.tick()
        np.testing.assert_allclose(s.last_command, new_command)

    def test_braking_uses_last_authorized_step_then_expires_without_extending_ik(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        from servo_tracking.cartesian_control import CartesianController
        s = self.s
        s.model = RobotModel(URDF.read_text(), tip='link6')
        s.config = load_config(s.visual['servo_config'])
        s.controller = CartesianController(s.config['control'])
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.scale, s.linear_until, s.stamp = .5, 0., 100.
        q = np.array(s.config['initial_joints'])
        actual = s.model.pose(q)
        s.actual.return_value = q, actual
        velocity = np.array([.01, 0., 0., 0., 0., 0.])
        twist = s.model.jacobian(q) @ velocity
        s.approved_target = s.model.pose(q + velocity)
        start = time.monotonic()
        s.received = s.panel_seen = start
        s.deadlines[s.revision] = start + .15
        s.fresh = VisualBackend.fresh.__get__(s)
        with patch('scripts.task.visual_servo.backend.time.monotonic', return_value=start) as clock:
            s.clock_s.return_value = 100.
            s.send_tracking_step(q, velocity, twist, start+.15)
            for age, factor in ((.16, .9), (.20, .5), (.24, .1)):
                clock.return_value = start+age
                s.clock_s.return_value = 100.+age
                s.tick()
                np.testing.assert_allclose(s.publish_twist.call_args.kwargs['joint_step'][1], velocity*factor)
                self.assertTrue(s.waiting_frame)
                self.assertEqual(s.deadlines[s.revision], start+.15)
            s.blocked.return_value = 'J2 上限'
            s.enter_boundary = Mock(wraps=s.enter_boundary)
            clock.return_value = start+.245
            s.clock_s.return_value = 100.245
            s.tick()
            self.assertEqual(s.boundary, 'J2 上限')
            self.assertIsNone(s.brake_sample)
            np.testing.assert_array_equal(s.last_command, np.zeros(6))

    def test_brake_deadline_and_faults_override_motion(self):
        s = self.s
        t = time.monotonic()
        s.clock_s.return_value, s.received = s.stamp+.155, t-.03
        s.brake_sample = (np.ones(6), t-.101, .5)
        self.assertEqual(s.brake_weight(), 0.)
        s.brake_sample = (np.ones(6), t-.02, .5)
        self.assertGreater(s.brake_weight(), 0.)
        for attr, invalid in [('source_ok', False), ('guard_state', 'holding'),
                              ('start_pending', True), ('input_reason', '观测曝光时间无效')]:
            previous = getattr(s, attr)
            setattr(s, attr, invalid)
            self.assertEqual(s.brake_weight(), 0.)
            setattr(s, attr, previous)

    def test_frozen_short_result_gap_brakes_then_resumes_on_an_advancing_miss(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        from servo_tracking.cartesian_control import CartesianController
        s = self.s
        s.model = RobotModel(URDF.read_text(), tip='link6')
        s.config = load_config(s.visual['servo_config'])
        s.controller = CartesianController(s.config['control'])
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.scale, s.stamp, s.received, s.panel_seen = .5, 100., 200., 200.
        s.fresh = VisualBackend.fresh.__get__(s)
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        q = np.array(s.config['initial_joints'])
        s.actual.return_value = q, s.model.pose(q)
        velocity = np.array([.01, 0., 0., 0., 0., 0.])
        twist = s.model.jacobian(q) @ velocity
        s.approved_target = s.goal.pose = s.model.pose(q + velocity)
        header = NS(frame_id='camera', stamp=NS(sec=100, nanosec=40_000_000))
        with patch('scripts.task.visual_servo.backend.time.monotonic', return_value=200.08) as clock:
            s.clock_s.return_value = 100.08
            s.on_miss(header)
            self.assertTrue(s.approval_fresh())
            s.send_tracking_step(q, velocity, twist, 200.15)
            clock.return_value, s.clock_s.return_value = 200.21, 100.21
            s.tick()
            np.testing.assert_allclose(s.publish_twist.call_args.kwargs['joint_step'][1], velocity*.8)
            self.assertTrue(s.waiting_frame)
            self.assertFalse(s.approval_fresh())
            s.request_motion.assert_not_called()
            clock.return_value, s.clock_s.return_value = 200.22, 100.22
            header.stamp.nanosec = 170_000_000
            s.on_miss(header)
            self.assertTrue(s.observation_ready() and s.approval_fresh())
            self.assertFalse(s.waiting_frame)
            s.request_motion.assert_not_called()
            clock.return_value, s.clock_s.return_value = 200.43, 100.43
            self.assertFalse(s.observation_ready())
            s.request_motion.assert_called_once_with(False)

    def test_new_frame_cannot_hide_long_gap_before_control_tick(self):
        for stamp_gap, receipt_gap in ((1.55, 1.46), (1.40, 1.55)):
            with self.subTest(stamp_gap=stamp_gap, receipt_gap=receipt_gap):
                self.setUp()
                s = self.s
                now = time.monotonic()
                s.stamp, s.received = 100., now-receipt_gap
                s.clock_s.return_value = 100.+stamp_gap+.03
                s.fresh = VisualBackend.fresh.__get__(s)
                s.active = True
                s.stop_tracking.side_effect = lambda _: setattr(s, 'active', False)
                s.geometry = TargetGeometry(s.visual, np.eye(4))
                cg = matrix([0., 0., .35, 0., 0., 0., 1.])
                s.inputs.match = Mock(return_value=(100.+stamp_gap, now, np.eye(4), chessboard_target(cg, s.visual), 0.))
                s.pending = None
                s.deadlines = {}
                s.valid_revision = -1
                s.update_target = s.validation_tick = Mock()
                s.candidate_boundary = lambda p: ''
                s.consume()
                self.assertTrue(s.active)
                self.assertEqual(s.boundary, LOST_HOLD)
                self.assertIsNone(s.approved_target)
                s.stop_tracking.assert_not_called()
                s.request_motion.assert_called_once_with(False)

    def test_wait_does_not_mask_source_loss_or_boundary_reopening(self):
        s = self.s
        s.fresh.return_value = False
        s.received = time.monotonic()-.04
        s.source_ok = False
        s.input_reason = '棋盘来源异常，位姿发布者 2 个，期望 1 个'
        with self.assertRaisesRegex(ValueError, '来源异常'):
            s.observation_ready()
        s.source_ok = True
        s.boundary = 'Servo 关节边界 J2 下限'
        s.reopening = True
        s.motion_future = Mock()
        self.assertFalse(s.observation_ready())
        self.assertFalse(s.reopening)
        self.assertEqual(s.retreat_axes, [(1, 1)])
        s.request_motion.assert_called_once_with(False)

    def test_late_but_advancing_camera_enters_confirmed_hold_and_revalidates(self):
        s = self.s
        # The 12:14 trace stopped at 1501 ms with a recent 682 ms late pose.
        s.fresh = VisualBackend.fresh.__get__(s)
        s.clock_s.return_value = 101.501
        now = time.monotonic()
        s.received = now-1.425
        late = PoseStamped(pose=pose_message([0., 0., .35, 0., 0., 0., 1.]))
        late.header.frame_id = 'camera'
        late.header.stamp.sec, late.header.stamp.nanosec = 100, 780_000_000
        with self.assertRaisesRegex(ValueError, '已过期'):
            s.inputs.receive(late, 101.462, now-.039)
        self.assertIsNone(s.inputs.pending)
        self.assertFalse(s.observation_ready())
        self.assertFalse(s.observation_ready())
        s.request_motion.assert_called_once_with(False)
        self.assertEqual(s.boundary, DELAY_HOLD)
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        self.assertIsNone(s.approved_target)
        s.approved_target = s.actual.return_value[1].copy()
        s.guard_state = 'holding'
        s.guard_info.update(gate_closed=True, stop_settled=True, stop_confirmed=True)
        s.approved_revision = s.revision
        s.retreat_stamp = s.stamp
        s.stamp = 102.
        s.count_retreat_frame()
        self.assertEqual(s.retreat, 0)  # Stop acknowledgement belongs to the old generation.
        s.guard_info['stop_generation'] = s.generation
        for stamp in (102., 102., 102.05):
            s.stamp = stamp
            s.count_retreat_frame()
        s.try_retreat(*s.actual())
        self.assertEqual(s.retreat, 2)
        self.assertIsNone(s.first_step)
        s.stamp = s.clock_s.return_value = 102.10
        s.received = time.monotonic()
        s.count_retreat_frame()
        s.try_retreat(*s.actual())
        self.complete_first_step(self.prepared_retreat())
        self.assert_open_requested()
        # A successful service reply alone cannot authorize a command.
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, load_config
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        q = np.asarray(s.config['initial_joints'])
        s.actual.return_value = (q, s.model.pose(q))
        s.ensure_switched, s.heartbeat, s.scale = lambda: True, Mock(), .5
        s.approved_target = s.model.pose(q)
        s.panel_seen = time.monotonic()
        s.control_twist = Mock(return_value=np.array([.002, 0., 0., 0., 0., 0.]))
        s.tick()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        s.guard_state = 'running'
        s.tick()
        self.assertFalse(s.boundary)
        self.assertGreater(np.linalg.norm(s.last_command), 0.)
        s.event.assert_called_with('delayed_input_recovered', '新鲜棋盘位姿已验证，Guard 已恢复')

    def test_far_reacquisition_requires_three_new_frames_then_opens_for_a_moving_board(self):
        s = self.s
        q, actual = s.actual()
        s.enter_boundary(LOST_HOLD)
        s.guard_state = 'holding'
        s.guard_info.update(gate_closed=True, stop_settled=True, stop_confirmed=True,
                            stop_generation=s.generation)
        s.approved_target = actual.copy()
        s.approved_target[0] += .08
        for dt, count in ((.03, 1), (0., 1), (.03, 2), (.03, 3)):
            s.stamp += dt
            if dt:
                s.revision += 1
            s.approved_revision = s.revision - 1
            s.exposure_stamps[s.approved_revision] = s.stamp - .01
            s.count_retreat_frame()
            self.assertEqual(s.retreat, count)
            s.try_retreat(q, actual)
            if count < 3:
                s.validity.call_async.assert_not_called()
        s.validity.call_async.assert_not_called()
        self.complete_first_step(self.prepared_retreat())
        self.assertTrue(s.active and s.reopening)
        self.assert_open_requested()

    def test_mechanical_limit_supersedes_delayed_input_hold(self):
        s = self.s
        s.enter_boundary(DELAY_HOLD)
        generation = s.generation
        pending = s.motion_future = Future()
        s.enter_boundary('J2 上限')
        self.assertEqual(s.boundary, 'J2 上限')
        self.assertEqual(s.retreat_axes, [(1, -1)])
        self.assertEqual(s.generation, generation)
        self.assertIs(s.motion_future, pending)
        s.request_motion.assert_called_once_with(False)

    def test_repeated_old_pose_cannot_release_input_hold(self):
        s = self.s
        now = time.monotonic()
        msg = PoseStamped(pose=pose_message([0., 0., .35, 0., 0., 0., 1.]))
        msg.header.frame_id, msg.header.stamp.sec = 'camera', 101
        with self.assertRaisesRegex(ValueError, '已过期'):
            s.inputs.receive(msg, 101.68, now-1.1)
        with self.assertRaisesRegex(ValueError, '重复'):
            s.inputs.receive(msg, 102.6, now)
        s.fresh.return_value = False
        s.clock_s.return_value = 102.6
        for age in (2.6, 60., 300.):
            s.clock_s.return_value = 100.+age
            self.assertFalse(s.observation_ready())
        self.assertTrue(s.active)
        self.assertEqual(s.boundary, LOST_HOLD)
        self.assertEqual(s.retreat, 0)
        s.request_motion.assert_called_once_with(False)
        s.stop_tracking.assert_not_called()

    def test_restarted_detector_with_same_name_requires_new_start(self):
        s = self.s
        s.topic, s.mode, s.board_gid = '/handeye/chessboard/pose', 'real', bytes([1]*16)
        s.get_publishers_info_by_topic = Mock(side_effect=[
            [NS(node_name='handeye_chessboard', endpoint_gid=[2]*16)],
            [NS(node_name='agx_arm_ctrl_single_node')]])
        s.check_source()
        self.assertFalse(s.source_ok)
        self.assertIn('发布者已更换', s.stop_tracking.call_args.args[0])
        s.request_motion.assert_not_called()

    def test_start_can_wait_with_closed_gate_while_fresh_ik_reports_no_solution(self):
        s = self.s
        s.guard_state, s.start_pending = 'idle', True
        s.unverified_since = time.monotonic()-2.
        s.ik_context = {'ik_failure_code': MoveItErrorCodes.NO_IK_SOLUTION}
        s.reject('目标不可达或碰撞')
        self.assertIn('移回', s.ik_wait_reason)
        self.assertLess(time.monotonic()-s.unverified_since, .1)
        s.request_motion.assert_not_called()
        s.stop_tracking.assert_not_called()

    def test_preview_soft_limit_can_arm_and_wait_for_a_verified_approach(self):
        s = self.s
        s.active, s.switched, s.source_ok = False, True, True
        s.guard_state, s.check_source, s.vision_gate = 'idle', Mock(), Mock()
        s.aligned_revision, s.panel_seen = -1, time.monotonic()
        s.target = s.approved_target.copy()
        s.reject('目标处于关节限位保护区 J5 上限')
        self.assertEqual(s.reachability, '正在验证可行接近点')
        response = s.start(None, NS(success=False, message=''))
        self.assertTrue(response.success)
        self.assertTrue(s.active and s.start_pending)
        self.assertEqual(s.valid_revision, -1)
        self.assertIsNone(s.approved_target)
        s.request_motion.assert_not_called()

    def test_real_startup_and_running_timing_revoke_only_motion(self):
        # Ages and arrival silences of the six timer stops in the 11:58 real session.
        cases = [(209, 152, 'running'), (156, 98, 'pausing'), (206, 155, 'running'),
                 (205, 154, 'running'), (216, 160, 'running'), (218, 156, 'running'),
                 (1364, 1313, 'running')]
        for age_ms, silence_ms, guard in cases:
            with self.subTest(age_ms=age_ms):
                s = self.s
                s.waiting_frame = False
                s.clock_s.return_value = s.stamp+age_ms/1000
                s.received = time.monotonic()-silence_ms/1000
                s.guard_state = guard
                s.motion_future = Future() if guard == 'pausing' else None
                s.motion_requested = s.panel_seen = time.monotonic()
                s.heartbeat, s.ensure_switched = Mock(), lambda: True
                s.fresh = VisualBackend.fresh.__get__(s)
                s.approved_target, s.linear_until = np.ones(7), time.monotonic()+2
                s.tick()
                s.stop_tracking.assert_not_called()
                s.request_motion.assert_not_called()
                self.assertIsNone(s.approved_target)
                self.assertEqual(s.linear_until, 0.)
                self.assertTrue(s.waiting_frame)
                np.testing.assert_array_equal(s.last_command, np.zeros(6))

    def test_new_frame_first_after_167_ms_gap_needs_fresh_ik_for_same_pose(self):
        s = self.s
        now = time.monotonic()
        s.clock_s.return_value = 100.231
        s.received = now-.141
        s.fresh = VisualBackend.fresh.__get__(s)
        s.approved_target, s.valid_revision = np.ones(7), s.revision
        s.geometry = TargetGeometry(s.visual, np.eye(4))
        cg = matrix([0., 0., .35, 0., 0., 0., 1.])
        s.inputs = NS(match=Mock(return_value=(100.167, now, np.eye(4), chessboard_target(cg, s.visual), .005)))
        s.pending = None
        s.update_target = Mock()  # Same numeric pose, unchanged revision.
        s.validation_tick = Mock()
        s.candidate_boundary = lambda p: ''
        s.consume()
        s.validation_tick.assert_called_once()
        s.stop_tracking.assert_not_called()
        self.assertTrue(s.fresh())
        self.assertIsNone(s.approved_target)
        np.testing.assert_array_equal(s.last_command, np.zeros(6))

    def test_start_handshake_recovers_with_new_observation_and_collision_checked_ik(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import JOINTS, RobotModel, load_config
        s = self.s
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.panel_seen, s.received = time.monotonic(), time.monotonic()-.098
        s.clock_s.return_value = 100.156
        s.fresh = VisualBackend.fresh.__get__(s)
        s.guard_state, s.start_pending = 'syncing', False
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        s.approved_revision, s.approved_target = s.revision, np.ones(7)
        s.tick()
        s.guard_state = 'running'
        s.tick()
        self.assertIsNone(s.approved_target)
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        s.config = load_config(s.visual['servo_config'])
        s.model = RobotModel(URDF.read_text(), tip='link6')
        q = np.asarray(s.config['initial_joints'])
        target_q = q.copy()
        target_q[0] += .01
        target = s.model.pose(target_q)
        s.actual.return_value = (q, s.model.pose(q))
        s.scale, s.mode, s.control_mode = .5, 'real', 'servo'
        s.preview_q, s.pause_revision, s.joint_epoch = None, -1, 1
        s.ik, s.publish_ghost = Mock(), Mock()
        s.get_clock = Mock()
        s.get_clock.return_value.now.return_value = Time(seconds=100)
        s.ik.call_async.side_effect = lambda req: Future()
        s.geometry = NS(measure=Mock(return_value=Goal(target, target[:3], np.array([0., 0., .35]),
                                                      None, None, False, False, None)))
        s.inputs = NS(match=Mock(return_value=(100.233, time.monotonic(), np.eye(4), np.eye(4), .005)))
        s.candidate_boundary = lambda p: ''
        s.target, s.pending = target, None
        s.clear_ik_seeds = Backend.clear_ik_seeds.__get__(s)
        s.update_target = Mock()  # The board can return at exactly the same target.
        s.clock_s.return_value = 100.29
        s.consume()
        s.tick()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        self.assertTrue(s.ik.call_async.call_args.args[0].ik_request.avoid_collisions)
        # Reject all requests for this exposure. The next exposure has the
        # same numeric target and must start a new collision-aware request.
        for _ in range(8):
            if s.pending is None:
                break
            f = s.pending
            f.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.NO_IK_SOLUTION)))
            if s.pending is f:
                s.ik_done(f, s.revision)
        self.assertIn('-31', s.ik_wait_reason)
        self.assertIsNone(s.pending)
        s.tick()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        s.clock_s.return_value = 100.35
        s.inputs.match.return_value = (100.30, time.monotonic(), np.eye(4), np.eye(4), .005)
        s.consume()
        self.assertTrue(s.ik.call_async.call_args.args[0].ik_request.avoid_collisions)
        f = s.pending
        f.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
                        solution=NS(joint_state=NS(name=JOINTS, position=target_q))))
        if s.pending is f:  # rclpy can dispatch the done callback synchronously.
            s.ik_done(f, s.revision)
        self.assertTrue(s.approval_fresh())
        self.assertFalse(s.ik_wait_reason)
        s.control_twist = Mock(return_value=np.array([.002, 0., 0., 0., 0., 0.]))
        s.tick()
        self.assertGreater(np.linalg.norm(s.last_command), 0.)
        s.stop_tracking.assert_not_called()
        s.request_motion.assert_not_called()

    def test_ik_no_solution_waits_for_new_exposure_then_holds_the_session(self):
        s = self.s
        s.ik_collect_all, s.ik_seeds, s.ik_seed_index = False, [np.zeros(6)], 1
        s.ik_seed_revision, s.ik_best_q = s.revision, None
        s.ik_context = dict(revision=s.revision, vision_deadline=time.monotonic()+.1)
        f = s.pending = Future()
        f.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.NO_IK_SOLUTION)))
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        s.ik_done(f, s.revision)
        s.event.assert_called_once()
        self.assertIn('-31', s.ik_wait_reason)
        s.stop_tracking.assert_not_called()
        np.testing.assert_array_equal(s.last_command, np.zeros(6))
        s.ensure_switched, s.heartbeat = lambda: True, Mock()
        s.start_pending = False
        s.panel_seen = time.monotonic()
        start = s.unverified_since
        s.tick()
        self.assertEqual(s.unverified_since, start)
        s.stop_tracking.assert_not_called()
        s.unverified_since -= s.visual['tracking_loss_timeout_s']+.01
        s.approach_limit_reason = Mock(return_value=IK_HOLD)
        s.tick()
        self.assertEqual(s.boundary, IK_HOLD)
        self.assertTrue(s.active)
        s.request_motion.assert_called_once_with(False)
        s.stop_tracking.assert_not_called()

    def test_workspace_braking_uses_available_room_and_leaves_retreat_available(self):
        s = self.s
        s.scale = .3
        s.model.pose = lambda q: np.r_[q[:3], 0., 0., 0., 1.]
        s.model.jacobian = lambda q: np.eye(6)
        v = np.r_[.09, np.zeros(5)]
        q = np.r_[.60, np.zeros(5)]
        self.assertEqual(s.workspace_speed_scale(q, v), 1.)
        s.guard_info.update(servo_reference_rad=np.r_[.698, np.zeros(5)],
                            reference_received=time.monotonic())
        slow = s.workspace_speed_scale(q, v)
        self.assertGreater(slow, 0.)
        self.assertLess(slow, .2)
        self.assertEqual(s.workspace_speed_scale(q, -v), 1.)
        q[0] = .70
        self.assertEqual(s.workspace_speed_scale(q, v), 0.)

    def test_approach_requires_feedback_to_reach_a_boundary_before_naming_it(self):
        from servo_tracking.core import load_config
        s = self.s
        s.scale = .3
        s.config = load_config(s.visual['servo_config'])
        s.model.jacobian = lambda q: np.eye(6)
        s.goal.pose = np.r_[.8, 0., .3, 0., 0., 0., 1.]
        actual = s.goal.pose.copy()
        actual[0] = .5
        self.assertEqual(s.approach_limit_reason(np.zeros(6), actual), IK_HOLD)
        actual[0] = .699
        self.assertEqual(s.approach_limit_reason(np.zeros(6), actual), '工作空间 X上限')
        s.goal.pose = actual.copy()
        s.goal.pose[3:] = Rotation.from_euler('y', 12., degrees=True).as_quat()
        q = np.zeros(6)
        self.assertEqual(s.approach_limit_reason(q, actual), IK_HOLD)
        q[4] = s.model.upper[4]-.017
        self.assertEqual(s.approach_limit_reason(q, actual), 'J5 上限')

    def test_approach_can_advance_while_orientation_temporarily_yields(self):
        s = self.s
        s.config.update(hold_position_tolerance=.003, hold_orientation_tolerance=.02)
        s.goal.pose = np.r_[.4, 0., .3, 0., 0., 0., 1.]
        s.feedback = NS(stamp=1.)
        actual = np.r_[.07, 0., .3, Rotation.from_euler('y', 15., degrees=True).as_quat()]
        self.assertFalse(s.approach_stalled(actual))
        s.feedback.stamp = 3.
        actual[0] += .002
        actual[3:] = Rotation.from_euler('y', 16., degrees=True).as_quat()
        self.assertFalse(s.approach_stalled(actual))
        s.feedback.stamp = 5.
        self.assertTrue(s.approach_stalled(actual))

    def test_soft_limited_full_ik_retries_only_a_checked_local_target(self):
        from servo_tracking.core import JOINTS, load_config
        s = self.s
        s.config = load_config(s.visual['servo_config'])
        s.mode, s.control_mode, s.pause_revision, s.joint_epoch = 'sim', 'servo', -1, 0
        s.nearby_revision, s.approved_revision = -1, -1
        s.approved_target, s.pending = None, None
        s.approval_fresh = VisualBackend.approval_fresh.__get__(s)
        s.clear_ik_seeds = Backend.clear_ik_seeds.__get__(s)
        s.candidate_boundary = Mock(return_value='')
        s.get_clock = Mock()
        s.get_clock.return_value.now.return_value = Time(seconds=100)
        s.publish_ghost = Mock()
        s.ik.call_async.side_effect = lambda request: Future()

        def bounds(q, margin=0.):
            if np.any(abs(np.asarray(q)) > 3.-margin):
                raise ValueError('fixture bound')

        s.model.planning_joints = lambda q, zero_tolerance=0.: q.copy()
        s.model.check_joints = bounds
        s.model.pose = lambda q: np.r_[q[0], 0., 0., 0., 0., 0., 1.]
        s.model.radius = 1.
        s.target = s.model.pose([.1])
        now = time.monotonic()
        s.deadlines[s.revision] = now+.1
        s.validation_tick()

        def complete(q):
            future = s.pending
            future.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS),
                solution=NS(joint_state=NS(name=JOINTS, position=q))))
            if s.pending is future:
                s.ik_done(future, s.revision)

        complete([.1, 0., 0., 0., 2.99, 0.])  # Inside the URDF limit but beyond the soft limit.
        self.assertIsNone(s.approved_target)
        self.assertEqual(s.ik.call_async.call_count, 2)
        req = s.ik.call_async.call_args.args[0].ik_request
        self.assertTrue(req.avoid_collisions)
        self.assertAlmostEqual(req.pose_stamped.pose.position.x, .025)
        complete([.025, 0., 0., 0., 0., 0.])
        self.assertEqual(s.nearby_motion, '分段接近')
        self.assertEqual(s.reachability, '接近点已验证')
        self.assertEqual(s.nearby_revision, s.revision)
        np.testing.assert_array_equal(s.target, s.approved_target)
        self.assertEqual(s.deadlines[s.approved_revision], now+.1)
        s.request_motion.assert_not_called()

        # An unchanged full goal in the next exposure is still a new request.
        revision = s.revision
        s.update_target(pose_message(s.model.pose([.1])))
        self.assertEqual(s.revision, revision+1)

    def test_rejection_of_old_goal_cannot_stop_the_current_session(self):
        s = self.s
        older = s.revision-1
        s.pause_revision, s.joint_epoch = -1, 0
        s.config['ik_timeout'] = .12
        s.ik_context = dict(revision=older, vision_deadline=time.monotonic()+.1,
                            target_time=time.monotonic(), epoch=0)
        f = s.pending = Future()
        f.set_result(NS(error_code=MoveItErrorCodes(val=MoveItErrorCodes.NO_IK_SOLUTION)))
        approved = s.approved_target
        s.ik_done(f, older)
        self.assertIs(s.approved_target, approved)
        self.assertEqual(s.pause_revision, -1)
        s.request_motion.assert_not_called()
        s.stop_tracking.assert_not_called()

    def test_explicit_collision_and_service_failure_stop_immediately(self):
        s = self.s
        for code in (MoveItErrorCodes.GOAL_IN_COLLISION, MoveItErrorCodes.COMMUNICATION_FAILURE):
            s.ik_context = dict(vision_deadline=time.monotonic()+1.)
            f = s.pending = Future()
            f.set_result(NS(error_code=MoveItErrorCodes(val=code)))
            s.ik_done(f, s.revision)
            self.assertIn(str(code), s.stop_tracking.call_args.args[0])
            self.assertFalse(s.ik_wait_reason)
            s.stop_tracking.reset_mock()

    def test_recorded_zero_departure_and_predictive_soft_limit(self):
        from tests.visual_servo_kinematics import URDF
        from servo_tracking.core import RobotModel, joint_speed_limits, load_config, pose_error
        from scripts.task.visual_servo.velocity import tracking_velocity
        cfg = load_config(self.s.visual['servo_config'])
        model = RobotModel(URDF.read_text(), tip='link6')
        goal_q = np.array([-1.102641, 1.895403, -.720784, 1.747986, 1.278640, -2.650087])
        goal = model.pose(goal_q)
        for j3 in (0., -.015, -.029):
            q = np.array([0., -j3, j3, 0., 0., 0.])
            error = np.concatenate(pose_error(goal, model.pose(q)))
            velocity = tracking_velocity(q, error, model, cfg, .3, goal_q)
            self.assertGreater(velocity[1], 0.)
            self.assertLess(velocity[2], 0.)
            self.assertFalse(joint_boundary(q, model.lower, model.upper, cfg['joint_margin'],
                                            velocity, cfg['command_timeout']))
            self.assertTrue(np.all(abs(velocity) <= joint_speed_limits(cfg, .3)))
            linear, angular = np.split(model.jacobian(q) @ velocity, 2)
            self.assertLessEqual(np.linalg.norm(linear), cfg['control']['linear_speed'] * .3 + 1e-12)
            self.assertLessEqual(np.linalg.norm(angular), cfg['control']['angular_speed'] * .3 + 1e-12)
            self.assertGreater(linear @ error[:3] + cfg['dls_rotation_weight']**2 * (angular @ error[3:]), 0.)
        np.testing.assert_array_equal(tracking_velocity(q, np.zeros(6), model, cfg, .3, goal_q), np.zeros(6))

    def test_start_waits_for_fresh_approval_then_requests_gate_once(self):
        s = self.s
        s.approval_fresh.return_value = False
        s.begin_tracking()
        self.assertTrue(s.start_pending)
        s.request_motion.assert_not_called()
        s.approval_fresh.return_value = True
        s.begin_tracking()
        s.begin_tracking()
        self.assertTrue(s.start_pending and s.preparing)
        s.request_motion.assert_called_once_with(True)

    def test_guard_refuses_late_open(self):
        s = NS(stop_generation=9, state='holding', enable=Mock(), stop_has_been_requested=True, stop_confirmed=False)
        for gen in (8, 9):
            resp = NS(success=False, message='')
            Guard.vision_motion(s, NS(enable=True, commit=False, stop_generation=gen), resp)
            self.assertFalse(resp.success)
        s.enable.assert_not_called()

    def test_concurrent_hold_acknowledges_existing_native_stop(self):
        g = NS(stop_generation=34, state='paused', pause_cause='joint_bound', reason='J4 受限',
               stop=Mock(), enable=Mock())
        for generation in (33, 34):
            request = NS(enable=False, stop_generation=generation)
            reply = Guard.vision_motion(g, request, NS(success=False, message=''))
            self.assertTrue(reply.success)
            self.assertEqual(reply.message, 'J4 受限')
        g.stop.assert_not_called()
        g.state = 'idle'  # A stale close cannot re-arm a stopped session.
        reply = Guard.vision_motion(g, NS(enable=False, stop_generation=33), NS(success=False, message=''))
        self.assertFalse(reply.success)
        g.stop.assert_not_called()

    def test_repeated_hold_uses_latest_confirmed_stop_without_ending_session(self):
        s = self.s
        s.guard_state = 'holding'
        s.enter_boundary(LOST_HOLD)
        future = s.motion_future = Future()
        state = NS(data=json.dumps(dict(s.guard_info, state='holding', reason='保持',
                                        gate_closed=True, stop_settled=True, stop_confirmed=True)))
        s.on_guard(state)
        self.assertFalse(s.confirmed())
        future.set_result(NS(success=True, message='保持'))
        s.motion_done(future, False)
        s.on_guard(state)
        self.assertTrue(s.confirmed())
        self.assertEqual(s.generation, 4)
        s.stop_tracking.assert_not_called()

    def test_progress_hold_preserves_session_and_clears_motion(self):
        s = self.s
        s.reason, s.goal.correcting, s.goal.camera_center = '', False, np.zeros(3)
        s.feedback = NS(stamp=0.)
        s.config.update(hold_position_tolerance=.003, hold_orientation_tolerance=.02)
        s.approved_target[0] = .084
        s.joint_constraint_scale = .0006
        q, pose = s.actual()
        pose = np.array([0., 0., 0., 0., 0., 0., 1.])
        s.actual.return_value = q, pose
        s.frozen_revision = s.revision
        s.frozen_ready = Mock(return_value=True)
        s.heartbeat, s.ensure_switched = Mock(), lambda: True
        s.control_twist = Mock(return_value=np.r_[.01, np.zeros(5)])
        s.joint_velocity = Mock(return_value=np.zeros(6))
        s.send_tracking_step = Mock()
        s.linear_until = 0.
        for i in range(16):
            s.feedback.stamp = 1+i*.1
            pose[0] = i*.000013
            s.panel_seen = time.monotonic()
            s.tick()
            self.assertEqual(s.boundary == STALLED_HOLD, i == 15)
        s.fresh.return_value = False
        self.assertTrue(s.visual_status()['waiting_for_target'])
        s.request_motion.assert_called_once_with(False)
        s.stop_tracking.assert_not_called()

    def test_stall_hold_requires_feasible_first_motion_when_new_target_is_far(self):
        s = self.s
        s.boundary, s.retreat_axes = STALLED_HOLD, []
        s.config.update(hold_position_tolerance=.003, hold_orientation_tolerance=.02)
        q, actual = s.actual()
        s.approved_target[0] = .05
        self.assertFalse(s.retreat_step_ok(q, actual, np.zeros(6)))
        self.assertTrue(s.retreat_step_ok(q, actual, np.ones(6)*.01))

    def test_old_ik_result_does_not_cross_deadline_or_branch(self):
        s = self.s
        s.pending = Future()
        s.ik_context = {'vision_deadline': time.monotonic()-1}
        s.ik_done(s.pending, s.revision)
        s.clear_ik_seeds.assert_called_once_with(cancel_pending=True)
        s.clear_ik_seeds.reset_mock()
        s.ik_done(Mock(), s.revision-1)
        s.clear_ik_seeds.assert_not_called()


class DetectorTests(unittest.TestCase):
    def test_slow_grid_result_uses_its_own_exposure_and_cannot_replace_newer_output(self):
        s = NS(reported_stamp=0, search=Mock(), publish_detection=Mock(return_value=True),
               recheck_outcomes=Counter(), recheck_pose_ages=[], get_clock=Mock(), get_logger=Mock(),
               pose_age_ms=61.)
        node_time = s.get_clock.return_value.now.return_value
        gray, corners, pose = np.zeros((3, 4), dtype=np.uint8), np.ones((2, 1, 2)), object()
        image, header = object(), object()
        stamp = 1_000_000_000
        node_time.nanoseconds = stamp+60_000_000
        ChessboardDetector.accept_recheck(s, stamp, image, header, gray, corners, pose, 'sb75', 50.)
        s.publish_detection.assert_called_once_with(image, header, corners, pose)
        self.assertEqual(s.reported_stamp, stamp)
        s.publish_detection.reset_mock()
        ChessboardDetector.accept_recheck(s, stamp-33_000_000, image, header, gray, None, None, 'sb100', 80.)
        s.publish_detection.assert_not_called()
        newer = stamp+33_000_000
        node_time.nanoseconds = newer+170_000_000
        ChessboardDetector.accept_recheck(s, newer, image, header, gray, corners, pose, 'sb100', 105.)
        s.publish_detection.assert_not_called()
        self.assertEqual(s.search.seed.call_args.args[-1], newer)

    def test_tracking_exposure_uses_kernel_epoch_and_preserves_a_delayed_frame_age(self):
        epoch = 1_795_095_811_000_000_000
        for unit in [1_000_000, 1000]:
            clock = CaptureClock()
            for i in range(12):
                readout = epoch+i*33_000_000
                # Last USB delivery is 80 ms late, then the detector runs 200 ms late.
                delivery = readout + (80_000_000 if i == 11 else 0)
                data = dict(hw_timestamp=1_000_000+i*33_000,
                            sensor_timestamp=996_000+i*33_000, backend_timestamp=delivery//unit)
                now = delivery + 200_000_000
                if i < 3:
                    with self.assertRaisesRegex(ValueError, '同步'):
                        clock.stamp(data, now)
                else:
                    self.assertEqual(clock.stamp(data, now), readout-4_000_000)
            with self.assertRaisesRegex(ValueError, '内核帧时间'):
                clock.stamp({**data, 'backend_timestamp': (now+100_000_000)//unit}, now)
            with self.assertRaisesRegex(ValueError, '同步'):
                clock.stamp({**data, 'hw_timestamp': 2000, 'sensor_timestamp': 1500}, now)

    def test_hardware_tracking_pose_and_preview_share_the_same_exposure_estimate(self):
        import json
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo
        from scripts.task.visual_servo.scene import K, render_board
        cfg = settings()
        cg = matrix([-.05, -.075, .35, 0., 0., 0., 1.])
        node = NS(frame='camera', pattern=cfg['pattern'], points=object_grid(cfg['pattern'], cfg['size']),
                  search=BoardSearch(tracking=True), bridge=CvBridge(), poses=Mock(), preview=Mock(),
                  projection=Mock(), projection_key=None, detection_outcomes=Counter(),
                  wake_detection=Mock(), latest_pair=None, overwritten=0, pair_lock=Lock(), capture_clock=CaptureClock(),
                  timing_since=time.monotonic(), timing_samples=[], latest_preview=None,
                  recheck_pose_ages=[], recheck_outcomes=Counter(),
                  get_clock=Mock(), get_logger=Mock())
        node.queue_pair = ChessboardDetector.queue_pair.__get__(node)
        node.detect = ChessboardDetector.detect.__get__(node)
        node.publish_detection = ChessboardDetector.publish_detection.__get__(node)
        node.preview.get_subscription_count.return_value = 1
        info = CameraInfo(width=640, height=480, k=K.ravel().tolist(), d=[0.]*5, distortion_model='plumb_bob')
        image = node.bridge.cv2_to_imgmsg(render_board(cg, cfg), encoding='bgr8')
        image.header.frame_id = info.header.frame_id = 'camera'
        epoch = 1_795_095_811_000_000_000
        for i in range(5):
            readout = epoch + i*33_000_000
            raw = readout + 140_000_000
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(raw, 10**9)
            info.header.stamp = image.header.stamp
            data = dict(hw_timestamp=1_000_000+i*33_000, actual_exposure=6000,
                        backend_timestamp=readout//1_000_000)
            meta = NS(header=image.header, json_data=json.dumps(data))
            node.get_clock.return_value.now.return_value.nanoseconds = readout + 12_000_000
            ChessboardDetector.queue_capture(node, image, info, meta)
        ChessboardDetector.detect_latest(node)
        node.preview.publish.assert_not_called()
        node.publish_preview = ChessboardDetector.publish_preview.__get__(node)
        ChessboardDetector.preview_latest(node)
        pose = node.poses.publish.call_args.args[0]
        self.assertEqual(pose.header, node.preview.publish.call_args.args[0].header)
        self.assertEqual(pose.header.stamp.sec*10**9+pose.header.stamp.nanosec, readout-3_000_000)
        self.assertNotEqual(pose.header.stamp, image.header.stamp)
        node.projection.publish.assert_called_once_with(info)

    def test_latest_complete_pair_survives_slow_detection(self):
        s = NS(latest_pair=None, pair_lock=Lock(), overwritten=0, search=NS(tracking=False),
               wake_detection=Mock(), detect=Mock(),
               get_clock=Mock())
        s.get_clock.return_value.now.return_value.nanoseconds = 10_010_000_000
        images = [NS(header=NS(stamp=NS(sec=10, nanosec=i))) for i in range(4)]
        for i in (0, 1, 3, 2):
            ChessboardDetector.queue_pair(s, images[i], f'info-{i}')
        ChessboardDetector.detect_latest(s)
        ChessboardDetector.detect_latest(s)
        s.detect.assert_called_once_with(images[3], 'info-3')

    def test_camera_image_reader_matches_publisher_reliability_without_reconnecting_each_poll(self):
        from rclpy.qos import ReliabilityPolicy
        reliable, best = ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT
        s = NS(raw_topic='/camera/color/image_raw', image=Mock(), image_reader=None, image_reliability=None,
               get_publishers_info_by_topic=Mock(), create_subscription=Mock(), destroy_subscription=Mock(),
               get_logger=Mock())
        for offered, expected in (([], best), ([reliable], reliable), ([reliable], reliable),
                                  ([best, reliable], best), ([best], best)):
            s.get_publishers_info_by_topic.return_value = [NS(qos_profile=NS(reliability=q)) for q in offered]
            ChessboardDetector.match_image_publisher(s)
            self.assertEqual(s.image_reliability, expected)
        self.assertEqual(s.create_subscription.call_count, 3)
        self.assertEqual(s.destroy_subscription.call_count, 2)

    def test_current_grid_tracking_reacquisition_and_loss(self):
        from scripts.task.visual_servo.scene import K, render_board
        cfg = settings()
        pattern = cfg['pattern']
        points = object_grid(pattern, cfg['size'])
        search = BoardSearch(tracking=True)
        def frame(x):
            cg = matrix([x, -.075, .35, 0., 0., 0., 1.])
            return cv2.cvtColor(render_board(cg, cfg), cv2.COLOR_BGR2GRAY)
        # Include an exact two-square jump: a periodic interior texture alone
        # must not authorize the previous origin when the outer corners moved.
        for i, (x, method) in enumerate(((-.17, 'sb75'), (-.1695, 'flow'),
                                         (-.1395, None), (.04, 'sb75'))):
            gray = frame(x)
            with patch('scripts.tools.handeye_calibration.chessboard.ordered_corners',
                       wraps=ordered_corners) as scans:
                corners, result = search.locate(gray, pattern, points, K, np.zeros(5), (i+1)*33_000_000)
            full_scans = [call for call in scans.call_args_list if not call.kwargs.get('fast')]
            self.assertLessEqual(len(full_scans), 1)
            if method:
                self.assertEqual(search.method, method)
            self.assertEqual(len(corners), len(points))
            self.assertLess(abs(result[1][0]-x), .001)
        center = tuple(np.rint(corners.mean(axis=0)[0]).astype(int))
        missing = cv2.circle(gray.copy(), center, 32, 128, -1)
        for im in (missing, np.zeros_like(gray)):
            self.assertIsNone(search.locate(im, pattern, points, K, np.zeros(5))[1])
            self.assertIsNone(search.previous_corners)
        # A failed half-visible board cannot keep an old pose alive.
        self.assertIsNotNone(search.locate(gray, pattern, points, K, np.zeros(5), 10**9)[1])
        with patch.object(search, 'track', side_effect=AssertionError('stale flow seed')):
            self.assertIsNotNone(search.locate(gray, pattern, points, K, np.zeros(5), 2*10**9)[1])

    def test_outer_squares_clipped_with_every_inner_corner_visible(self):
        from scripts.task.visual_servo.scene import K, render_board
        cfg = settings()
        points = object_grid(cfg['pattern'], cfg['size'])
        search = BoardSearch(tracking=True)
        # The bottom row is ~6 px inside the image, with ~27 px square pitch.
        for i in range(2):
            cg = matrix([-.05 + i*.0005, -.0185, .35, 0., 0., 0., 1.])
            gray = cv2.cvtColor(render_board(cg, cfg), cv2.COLOR_BGR2GRAY)
            _, pose = search.locate(gray, cfg['pattern'], points, K, np.zeros(5), (i+1)*33_000_000)
            self.assertIsNotNone(pose)
            self.assertLess(np.linalg.norm(pose[1]-cg[:3, 3]), .001)
        self.assertEqual(search.method, 'flow')

    def test_reacquisition_uses_roi_and_distinguishes_invalid_pose(self):
        from scripts.task.visual_servo.scene import K, render_board
        cfg = settings()
        pattern = cfg['pattern']
        points = object_grid(pattern, cfg['size'])
        search = BoardSearch(tracking=True)
        pose = matrix([-.05, -.075, .42, 0., 0., 0., 1.])
        gray = cv2.cvtColor(render_board(pose, cfg), cv2.COLOR_BGR2GRAY)
        args = (pattern, points, K, np.zeros(5))
        self.assertIsNotNone(search.locate(gray, *args, 33_000_000)[1])
        self.assertIsNone(search.locate(np.full_like(gray, 255), *args, 66_000_000)[1])
        self.assertEqual(search.failure, 'grid_missing')
        self.assertIsNotNone(search.locate(gray, *args, 99_000_000)[1])
        self.assertEqual(search.method, 'roi')
        with patch('scripts.tools.handeye_calibration.chessboard.solve_pose', return_value=None):
            corners, result = search.locate(gray, *args, 132_000_000)
        self.assertIsNotNone(corners)
        self.assertIsNone(result)
        self.assertEqual(search.failure, 'pose_invalid')


class TrackingUpgradeTests(unittest.TestCase):
    def test_normal_two_to_three_second_continuation_is_not_timed_out(self):
        progress = FrozenProgress(settings())
        for i in range(29):
            self.assertFalse(progress.update(i*.1, (.086-.003*i, .11-.003*i),
                                             (.003, .02), limited=True))

    def test_velocity_estimate_uses_exposures_and_clears_across_occlusion(self):
        cfg = settings()
        estimator = CenterVelocity(cfg)
        for i in range(10):
            speed = estimator.update(100+i*.033, np.array([.05*.033*i, 0., 0.]))
        np.testing.assert_allclose(speed, [.05*cfg['feedforward_gain'], 0., 0.], atol=.001)
        np.testing.assert_array_equal(estimator.update(100.297, np.ones(3)), np.zeros(3))
        estimator.reset()
        np.testing.assert_array_equal(estimator.update(101., np.ones(3)), np.zeros(3))

    def test_bounded_solver_keeps_unconstrained_axes_moving_and_allows_retreat(self):
        from servo_tracking.core import load_config
        from scripts.task.visual_servo.velocity import tracking_step
        config = load_config(settings()['servo_config'])
        model = NS(lower=np.full(6, -3.), upper=np.full(6, 3.), jacobian=lambda q: np.eye(6))
        reference = np.zeros(6)
        reference[3] = model.lower[3]+config['joint_margin']+.00001
        twist = np.array([.05, 0., 0., -.25, 0., 0.])
        step, ratio = tracking_step(np.zeros(6), twist, model, config, .3, reference=reference)
        self.assertLess(ratio, .001)
        self.assertGreater(step[0], .045)
        self.assertLess(abs(step[3]), .0001)
        step, ratio = tracking_step(np.zeros(6), -twist, model, config, .3, reference=reference)
        self.assertGreater(step[3], .2)
        self.assertEqual(ratio, 1.)

    def test_view_anticipates_corner_exit_and_uses_remaining_pixel_margin(self):
        from sensor_msgs.msg import CameraInfo
        from scripts.task.visual_servo.scene import K
        from scripts.task.visual_servo.visibility import BoardView
        view = BoardView(CameraInfo(width=640, height=480, k=K.ravel().tolist(), d=[0.]*5,
                                   distortion_model='plumb_bob'), np.eye(4), settings())
        actual = np.array([0., 0., 0., 0., 0., 0., 1.])
        corners = np.array([[.09, -.05, .35], [.17, -.05, .35], [.17, .05, .35], [.09, .05, .35]])
        twist = np.array([0., 0., 0., 0., -.4, 0.])
        self.assertLess(view.predicted_margin(corners, np.eye(4), twist), 0.)
        shaped, margin, limited = view.shape(corners, actual, twist, .09)
        self.assertGreaterEqual(margin, view.safe_margin)
        self.assertTrue(limited)
        self.assertTrue(np.any(shaped))
        self.assertLess(abs(shaped[4]), abs(twist[4]))
        self.assertGreaterEqual(view.predicted_margin(corners, np.eye(4), shaped), view.safe_margin)

    def test_angular_priority_yields_only_when_translation_advances(self):
        from servo_tracking.core import load_config
        from scripts.task.visual_servo.velocity import tracking_step
        cfg = load_config(settings()['servo_config'])
        jacobian = np.eye(6)
        jacobian[3, 0] = 2.
        model = NS(lower=np.full(6, -3.), upper=np.full(6, 3.), jacobian=lambda q: jacobian)
        q = np.zeros(6)
        q[3] = model.lower[3]+cfg['joint_margin']
        twist = np.array([.06, 0., 0., .02, 0., 0.])
        balanced, _ = tracking_step(q, twist, model, cfg, .3, reference=q)
        priority, _ = tracking_step(q, twist, model, cfg, .3, reference=q, position_first=True)
        self.assertGreater(priority[0], balanced[0])
        self.assertGreaterEqual((jacobian @ priority)[3], 0.)
        self.assertGreaterEqual(priority[3], 0.)


if __name__ == '__main__':
    unittest.main()
