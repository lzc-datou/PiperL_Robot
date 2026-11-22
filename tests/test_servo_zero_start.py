"""Bounded zero-edge planning references; fake services only, no ROS init."""
from concurrent.futures import Future
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest

import numpy as np

try:
    from scripts.task.servo_tracking.core import (
        JOINTS, FreshJoints, RobotModel, MAX_ZERO_FEEDBACK_TOLERANCE)
    from scripts.task.servo_tracking.guard import Guard
    from scripts.task.servo_tracking.joint_motion import JointMotion
    from tests import test_servo_guard_recovery as recovery
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS = True
except ImportError:
    ROS = False


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'
# Exact measured snapshots from the two real failures; these are not commands.
LOG_Q = (
    np.array([0., -.00047123889803846896, -.00034906585039886593, 0., 0., 0.]),
    np.array([0., -.0012042771838760874, .0002617993877991494, 0., 0., 0.]),
    np.array([0., -.0003665191429188092, 0., 0., 0., 0.]),
)


class FakeController:
    def __init__(self):
        self.calls = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, goal_uuid=None):
        future = Future()
        self.calls.append((goal, goal_uuid, future))
        return future


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; every transport is fake')
class ServoZeroStartTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.).start()
        self.wall = patch('scripts.task.servo_tracking.guard.time.time', return_value=1000.).start()
        self.addCleanup(patch.stopall)
        self.g = recovery.ServoGuardRecoveryTests.guard(self)
        self.g.model = RobotModel(URDF.read_text(), tip='link6')
        self.g.feedback, self.g.commanded = FreshJoints(), FreshJoints()
        self.g.health = MethodType(Guard.health, self.g)
        self.g.config.update(zero_feedback_tolerance=MAX_ZERO_FEEDBACK_TOLERANCE,
                             joint_margin=.015, lower_singularity_threshold=17.,
                             planning_time=5., plan_settle_timeout=5.)
        motor = [NS(name=name, level=0, OK=0, values=[NS(key='enabled', value='true')]) for name in JOINTS]
        arm = [NS(name='arm', level=0, OK=0, values=[NS(key=key, value=value) for key, value in
               dict(arm_status='0', teach_status='0', ctrl_mode='1').items()])]
        header = self.message(np.zeros(6)).header
        self.g.diagnostics = {kind: (100., NS(header=header, status=statuses))
                              for kind, statuses in (('motor', motor), ('arm', arm))}
        self.g.get_clock = Mock()
        self.g.get_clock.return_value.now.return_value.to_msg.return_value = header.stamp
        # A separate strictly legal command fixture, never substituted for raw feedback.
        self.anchor = np.array([.0002, .0001, -.0002, .0003, .0004, -.0001])

    def message(self, q, stamp=None):
        msg = JointState()
        msg.name, msg.position = JOINTS.copy(), np.asarray(q).tolist()
        stamp = self.wall.return_value if stamp is None else stamp
        nanoseconds = round(stamp * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
        return msg

    def measured(self, q, stamp=None):
        self.g.on_feedback(self.message(q, stamp))

    def attach_motion(self):
        self.planner, self.controller = recovery.FakeService(), FakeController()
        self.g.create_client, self.g.create_service = Mock(return_value=self.planner), Mock()
        with patch('scripts.task.servo_tracking.joint_motion.ActionClient', return_value=self.controller):
            self.motion = JointMotion(self.g)
        self.g.joint = self.motion
        self.addCleanup(self.motion.shutdown)

    def preparing_execution(self, accepted=False):
        self.now.return_value, self.wall.return_value = 100., 1000.
        self.g.feedback, self.g.commanded = FreshJoints(), FreshJoints()
        self.g.last_forwarded = None
        self.g.driver_output.publish.reset_mock()
        self.attach_motion()
        self.measured(LOG_Q[1])
        self.g.state, self.g.epoch, self.g.stage_time = 'joint_executing', 7, 100.
        self.motion.execution_epoch = 7
        self.motion.owned_uuid = bytes([7] * 16)
        self.motion.actions[self.motion.owned_uuid] = None
        if accepted:
            self.motion.goal_handle = NS(cancel_goal_async=Mock())
        else:
            self.motion.pending_goal = Future()

    def test_health_preserves_exact_real_log_measurements(self):
        for index, raw in enumerate(LOG_Q):
            with self.subTest(snapshot=index):
                self.measured(raw, 1000. + .001 * index)
                np.testing.assert_array_equal(self.g.health(), raw)
                np.testing.assert_array_equal(self.g.feedback.q, raw)
                with self.assertRaisesRegex(ValueError, '关节角超出模型限位'):
                    self.g.model.check_joints(raw)

    def test_visual_servo_and_controller_start_from_legal_zero_edge_references(self):
        g = self.g
        g.visual_mode = True
        g.servo_joint_states = Mock()
        raw = np.deg2rad([0., -.038, .015, 0., 0., 0.])
        msg = self.message(raw)
        msg.velocity = [0., -.02, .02, .01, 0., 0.]
        msg.effort = [0., .3, .2, 0., 0., 0.]
        g.on_feedback(msg)
        sample = g.servo_joint_states.publish.call_args.args[0]
        self.assertEqual(sample.header.stamp.sec, 1000)
        np.testing.assert_array_equal(sample.position, np.zeros(6))
        np.testing.assert_array_equal(sample.velocity, [0., 0., 0., .01, 0., 0.])
        np.testing.assert_array_equal(sample.effort, msg.effort)
        np.testing.assert_array_equal(g.health(), raw)

        g.controller_hold(g.health())
        hold = g.output.publish.call_args.args[0].points[0]
        np.testing.assert_array_equal(hold.positions, np.zeros(6))
        self.assertFalse(hold.velocities)
        g.state = 'running'
        g.on_commanded(self.message(hold.positions))
        self.assertEqual(g.state, 'running')
        np.testing.assert_array_equal(g.driver_output.publish.call_args.args[0].position, np.zeros(6))
        self.now.return_value += .01
        self.wall.return_value += .01
        inward = np.deg2rad([0., .04, -.01, 0., 0., 0.])
        g.on_commanded(self.message(inward))
        np.testing.assert_array_equal(g.driver_output.publish.call_args.args[0].position, inward)

    def test_visual_reference_does_not_publish_a_faulted_feedback_sample(self):
        g = self.g
        g.visual_mode = True
        g.servo_joint_states = Mock()
        for j2, j4 in ((-.101, 0.), (0., 127.001)):
            self.wall.return_value += .01
            raw = np.deg2rad([0., j2, 0., j4, 0., 0.])
            self.measured(raw)
            np.testing.assert_array_equal(g.feedback.q, raw)
            with self.assertRaisesRegex(ValueError, '模型限位'):
                g.health()
        g.servo_joint_states.publish.assert_not_called()

    def test_simulation_feedback_keeps_strict_model_bounds(self):
        self.g.real = False
        self.measured(LOG_Q[1])
        with self.assertRaisesRegex(ValueError, '关节角超出模型限位.*joint2'):
            self.g.health()
        with self.assertRaisesRegex(ValueError, '关节角超出模型限位.*joint2'):
            self.g.planning_start()

    def test_planning_reference_is_explicitly_normalized_without_altering_raw(self):
        for index, raw in enumerate(LOG_Q):
            with self.subTest(snapshot=index):
                self.measured(raw, 1000. + .001 * index)
                reference = self.g.planning_start()
                expected = raw.copy()
                expected[1] = 0.
                if expected[2] > 0.:
                    expected[2] = 0.
                np.testing.assert_array_equal(reference, expected)
                np.testing.assert_array_equal(self.g.feedback.q, raw)
                np.testing.assert_array_equal(self.g.model.check_joints(reference), expected)
                reference[0] = 1.  # Planning reference never aliases the measurement.
                np.testing.assert_array_equal(self.g.feedback.q, raw)
        self.g.get_logger.return_value.warning.assert_called()
        warning = self.g.get_logger.return_value.warning.call_args.args[0]
        self.assertIn('非实测关节角', warning)
        self.assertIn('planning_reference', warning)
        self.assertIn('feedback_source_age', warning)
        self.assertNotIn('measured_anchor', warning)

    def test_legal_current_measurement_returns_an_unmodified_independent_copy(self):
        self.measured(self.anchor)
        reference = self.g.planning_start()
        np.testing.assert_array_equal(reference, self.anchor)
        reference[1] = 1.
        np.testing.assert_array_equal(self.g.feedback.q, self.anchor)
        self.g.get_logger.return_value.warning.assert_not_called()

    def test_duplicate_timestamps_do_not_replace_or_retime_raw_measurement(self):
        self.measured(LOG_Q[2])
        stamp, received = self.g.feedback.stamp, self.g.feedback.received
        self.now.return_value = 100.1
        # A changed payload under the same acquisition timestamp is not a new sample.
        self.measured(np.zeros(6), 1000.)
        np.testing.assert_array_equal(self.g.feedback.q, LOG_Q[2])
        self.assertEqual((self.g.feedback.stamp, self.g.feedback.received), (stamp, received))

    def test_fixed_negative_zero_bias_without_any_legal_sample_accepts_official_planning(self):
        self.attach_motion()
        for index in range(10):
            self.now.return_value = 100. + index * .02
            self.wall.return_value = 1000. + index * .02
            self.measured(LOG_Q[2])
            np.testing.assert_array_equal(self.g.planning_start(), np.zeros(6))
            np.testing.assert_array_equal(self.g.feedback.q, LOG_Q[2])
        self.assertFalse(hasattr(self.g, 'bounded_feedback'))
        request = NS(joints=[.1, 1.28, -1.12, .3, .88, 0.], speed_scale=.5,
                     purpose='point', target_revision=9, auto_execute=False)
        response = self.motion.plan(request, NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.g.state, 'joint_pausing')
        self.g.pause_servo.complete()
        self.assertEqual(self.g.state, 'joint_planning')
        np.testing.assert_array_equal(self.planner.calls[-1][0].motion_plan_request.start_state.joint_state.position,
                                      np.zeros(6))
        self.assertEqual(self.controller.calls, [])
        self.assertEqual(self.g.gate.calls, [])

    def test_normalization_never_bypasses_raw_feedback_freshness(self):
        self.measured(LOG_Q[2])
        np.testing.assert_array_equal(self.g.planning_start(), np.zeros(6))
        self.now.return_value, self.wall.return_value = 100.3, 1000.3
        with self.assertRaisesRegex(ValueError, '关节反馈缺失或超时'):
            self.g.planning_start()
        self.assertEqual(self.g.feedback.stamp, 1000.)
        self.assertEqual(self.g.feedback.received, 100.)

    def test_feedback_tolerance_does_not_allow_other_axes_or_larger_zero_error(self):
        for index, value in ((0, self.g.model.upper[0] + 1e-5), (1, -.002), (2, .002)):
            raw = LOG_Q[1].copy()
            raw[index] = value
            self.measured(raw, 1000. + .001 * (index + 1))
            with self.subTest(joint=index), self.assertRaisesRegex(ValueError, '关节角超出模型限位'):
                self.g.health()

    def begin_validation(self):
        self.attach_motion()
        self.measured(LOG_Q[1])
        goal = [.1, 1.28, -1.12, .3, .88, 0.]
        reference = np.zeros(6)
        response = self.motion.plan(NS(joints=goal, speed_scale=.5, purpose='point',
                                      target_revision=9, auto_execute=False), NS())
        self.assertTrue(response.success, response.message)
        self.g.pause_servo.complete()
        request = self.planner.calls[-1][0].motion_plan_request
        np.testing.assert_array_equal(request.start_state.joint_state.position, reference)
        trajectory = JointTrajectory(joint_names=JOINTS)
        first = JointTrajectoryPoint(positions=reference.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6)
        last = JointTrajectoryPoint(positions=goal, velocities=[0.] * 6, accelerations=[0.] * 6)
        last.time_from_start.sec = 2
        trajectory.points = [first, last]
        with patch('scripts.task.servo_tracking.joint_motion.validate_plan_math', return_value=2.) as validate:
            self.planner.calls[-1][1].set_result(NS(motion_plan_response=NS(error_code=NS(val=1),
                trajectory=NS(joint_trajectory=trajectory, multi_dof_joint_trajectory=NS(points=[])))))
            self.motion.pending_validation.exception(timeout=1.)
            np.testing.assert_array_equal(validate.call_args.args[1], reference)
        np.testing.assert_array_equal(self.g.feedback.q, LOG_Q[1])

    def test_async_math_uses_captured_planning_reference_and_execution_checks_raw_feedback(self):
        self.begin_validation()
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_ready')
        self.assertTrue(self.motion.ready)
        np.testing.assert_array_equal(self.motion.check_start(), LOG_Q[1])
        changed = LOG_Q[1].copy()
        changed[0] = .006
        self.measured(changed, 1000.001)
        with self.assertRaisesRegex(ValueError, '真实关节起点已变化'):
            self.motion.check_start()
        self.assertEqual(self.g.gate.calls, [])
        self.assertEqual(self.controller.calls, [])

    def test_async_validation_completion_rechecks_fresh_feedback_and_start_drift(self):
        for failure in ('feedback', 'drift'):
            with self.subTest(failure=failure):
                self.now.return_value, self.wall.return_value = 100., 1000.
                self.g.feedback = FreshJoints()
                self.g.state, self.g.closed, self.g.hold_pending = 'idle', True, False
                self.g.stop_future = self.g.closing = None
                self.begin_validation()
                if failure == 'feedback':
                    self.now.return_value, self.wall.return_value = 100.3, 1000.3
                else:
                    changed = LOG_Q[1].copy()
                    changed[0] = .006
                    self.measured(changed, 1000.001)
                self.g.tick()
                self.assertEqual(self.g.state, 'fault')
                self.assertFalse(self.motion.ready)
                self.assertEqual(self.controller.calls, [])
                self.assertFalse(any(request.data for request, _ in self.g.gate.calls))

    def test_initial_old_raw_hold_is_dropped_for_pending_and_accepted_owned_goal(self):
        for accepted in (False, True):
            with self.subTest(accepted=accepted):
                self.preparing_execution(accepted)
                self.now.return_value = 100.04
                self.g.on_commanded(self.message(LOG_Q[1]))
                self.assertEqual(self.g.state, 'joint_executing')
                self.assertIsNone(self.g.last_forwarded)
                self.g.driver_output.publish.assert_not_called()
                self.assertEqual(self.g.gate.calls, [])

    def test_first_strict_plan_command_is_forwarded_without_altering_it(self):
        self.preparing_execution()
        self.g.on_commanded(self.message(LOG_Q[1]))
        self.now.return_value = 100.04
        self.g.on_commanded(self.message(self.anchor, 1000.001))
        self.assertEqual(self.g.state, 'joint_executing')
        self.g.driver_output.publish.assert_called_once()
        np.testing.assert_array_equal(self.g.driver_output.publish.call_args.args[0].position, self.anchor)
        np.testing.assert_array_equal(self.g.last_forwarded, self.anchor)
        # Even inside 100 ms, no later out-of-bounds command receives the exception.
        self.g.on_commanded(self.message(LOG_Q[1], 1000.002))
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.g.driver_output.publish.call_count, 1)

    def test_old_raw_hold_outside_start_window_faults_and_closes_gate(self):
        self.preparing_execution()
        self.now.return_value = 100.101
        self.g.on_commanded(self.message(LOG_Q[1]))
        self.assertEqual(self.g.state, 'fault')
        self.g.driver_output.publish.assert_not_called()
        self.assertFalse(self.g.gate.calls[-1][0].data)

    def test_old_epoch_or_non_joint_execution_never_gets_the_hold_exception(self):
        for variant in ('epoch', 'running', 'joint_settling', 'no_goal'):
            with self.subTest(variant=variant):
                self.preparing_execution()
                if variant == 'epoch':
                    self.motion.execution_epoch -= 1
                elif variant == 'no_goal':
                    self.motion.pending_goal = None
                else:
                    self.g.state = variant
                self.g.on_commanded(self.message(LOG_Q[1]))
                self.assertEqual(self.g.state, 'fault')
                self.g.driver_output.publish.assert_not_called()

    def test_startup_exception_requires_both_zero_tolerance_and_actual_agreement(self):
        for variant in ('zero_tolerance', 'actual_agreement'):
            with self.subTest(variant=variant):
                self.preparing_execution()
                command = LOG_Q[1].copy()
                if variant == 'zero_tolerance':
                    command[1] = -.002
                else:
                    actual = LOG_Q[1].copy()
                    actual[1] = .001
                    self.measured(actual, 1000.001)
                self.g.on_commanded(self.message(command))
                self.assertEqual(self.g.state, 'fault')
                self.g.driver_output.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
