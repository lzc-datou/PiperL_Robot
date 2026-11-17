"""ROS message adapters with fake transports; no node, executor, SDK or CAN.

Use generated message classes and the production pure observer/controller, but
invoke Backend methods on a SimpleNamespace.  No rclpy initialization or ROS
publisher/service is created.  Source header time and monotonic receipt time
are deliberately different to catch accidental clock-domain matching.
"""
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from builtin_interfaces.msg import Time
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from scripts.task.servo_tracking.cartesian_control import CartesianController
    from scripts.task.servo_tracking.core import JOINTS, FreshJoints, damped_joint_velocity
    from scripts.task.servo_tracking.execution_feedback import ExecutionObserver
    from scripts.task.servo_tracking.node import Backend
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use ROS runtime wrapper; all transports remain fake')
class ServoExecutionAdapterTests(unittest.TestCase):
    def setUp(self):
        self.wall = patch('scripts.task.servo_tracking.node.time.time', return_value=1000.).start()
        self.monotonic = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)
        self.q = np.array([.1, .2, -.3, .4, .5, .6])
        self.velocity = np.array([.02, -.03, .04, -.05, .06, -.07])
        self.matrix = np.diag(np.arange(1., 7.)) + .05*np.ones((6, 6))

    @staticmethod
    def stamp(seconds):
        nanoseconds = int(round(seconds*1e9))
        return Time(sec=nanoseconds//1000000000, nanosec=nanoseconds%1000000000)

    def state(self, real_observer=True):
        model = NS(jacobian=Mock(side_effect=lambda q: self.matrix + np.diag(np.asarray(q)*.1)))
        observer = ExecutionObserver() if real_observer else Mock()
        state = NS(feedback=FreshJoints(), execution_observer=observer, execution_source_stamps={},
                   model=model, config={'joint_speed': 2.4}, scale=.5,
                   active=True, guard_state='running', twist=Mock(), jog=Mock(),
                   controller=CartesianController({'algorithm': 'feedforward_p'}),
                   actual=Mock(return_value=(self.q.copy(), np.array([0., 0., 0., 0., 0., 0., 1.]))),
                   get_clock=Mock())
        state.get_clock.return_value.now.return_value.to_msg.return_value = self.stamp(1000.)
        for name in ('on_feedback', 'on_servo_trajectory', 'on_controller_state',
                     'fresh_execution_source', 'publish_twist', 'reset_controller'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        return state

    def feedback(self, seconds=1000., q=None, names=None, velocities=()):
        q = self.q if q is None else np.asarray(q)
        names = list(reversed(JOINTS)) if names is None else names
        message = JointState()
        message.header.stamp = self.stamp(seconds)
        message.name = names
        message.position = [float(q[JOINTS.index(name)]) for name in names]
        message.velocity = list(map(float, velocities))
        return message

    def trajectory(self, seconds=1000., q=None, velocity=None):
        q = self.q if q is None else np.asarray(q)
        velocity = self.velocity if velocity is None else np.asarray(velocity)
        message = JointTrajectory()
        message.header.stamp = self.stamp(seconds)
        message.joint_names = list(reversed(JOINTS))
        point = JointTrajectoryPoint()
        point.positions = [float(q[JOINTS.index(name)]) for name in message.joint_names]
        point.velocities = [float(velocity[JOINTS.index(name)]) for name in message.joint_names]
        message.points = [point]
        return message

    def controller_state(self, seconds=1000.):
        trajectory = self.trajectory(seconds)
        message = JointTrajectoryControllerState()
        message.header = trajectory.header
        message.joint_names = trajectory.joint_names
        message.reference = trajectory.points[0]
        return message

    def test_sdk_missing_or_placeholder_velocity_uses_raw_position_difference(self):
        for placeholder in ((), [0.]*6, [999.]*6):
            with self.subTest(placeholder=placeholder):
                state = self.state()
                state.on_feedback(self.feedback(velocities=placeholder))
                self.assertFalse(state.execution_observer.observations['measured'])
                self.monotonic.return_value = 100.01
                next_q = self.q + self.velocity*.01
                state.on_feedback(self.feedback(1000.01, next_q, velocities=placeholder))
                samples = state.execution_observer.observations['measured']
                self.assertEqual(len(samples), 1)
                stamp, actual = samples[-1]
                midpoint = (self.q + next_q)*.5
                expected = (self.matrix + np.diag(midpoint*.1)) @ self.velocity
                self.assertEqual(stamp, 100.01)  # Receipt time, not ROS source time.
                np.testing.assert_allclose(actual, expected, atol=1e-12)
                np.testing.assert_array_equal(state.feedback.q, next_q)
                self.monotonic.return_value = 100.

    def test_duplicate_feedback_cannot_refresh_estimator_or_invent_velocity(self):
        state = self.state()
        state.on_feedback(self.feedback())
        self.monotonic.return_value = 100.01
        state.on_feedback(self.feedback(q=self.q + 10.))  # Changed payload, same source stamp.
        self.assertEqual(state.execution_observer.joints['measured'][0], 100.)
        np.testing.assert_array_equal(state.feedback.q, self.q)
        self.assertFalse(state.execution_observer.observations['measured'])
        next_q = self.q + self.velocity*.02
        self.monotonic.return_value = 100.02
        state.on_feedback(self.feedback(1000.02, next_q))
        expected = (self.matrix + np.diag((self.q+next_q)*.05)) @ self.velocity
        np.testing.assert_allclose(state.execution_observer.observations['measured'][-1][1], expected)

    def test_invalid_or_partial_raw_source_never_refreshes_measured_observer(self):
        invalid = []
        partial = self.feedback(1000.01)
        partial.name.pop(); partial.position.pop()
        invalid.append(partial)
        duplicate_name = self.feedback(1000.01)
        duplicate_name.name[0] = duplicate_name.name[1]
        invalid.append(duplicate_name)
        nonfinite = self.feedback(1000.01)
        nonfinite.position[2] = float('nan')
        invalid.append(nonfinite)
        invalid.extend((self.feedback(999.99), self.feedback(0.)))
        bad_stamp = self.feedback(1000.01)
        invalid.append(NS(name=bad_stamp.name, position=bad_stamp.position,
                          header=NS(stamp=NS(sec=float('nan'), nanosec=0))))
        for message in invalid:
            with self.subTest(message=message):
                state = self.state(real_observer=False)
                state.on_feedback(self.feedback())
                state.execution_observer.observe_joints.reset_mock()
                self.monotonic.return_value = 100.01
                state.on_feedback(message)
                state.execution_observer.observe_joints.assert_not_called()
                self.assertEqual(state.feedback.stamp, 1000.)
                self.monotonic.return_value = 100.

    def test_servo_first_knot_maps_reordered_joints_with_its_own_jacobian(self):
        state = self.state()
        state.on_servo_trajectory(self.trajectory())
        sample = state.execution_observer.observations['servo'][-1]
        self.assertEqual(sample[0], 100.)
        np.testing.assert_allclose(sample[1], (self.matrix+np.diag(self.q*.1)) @ self.velocity)
        np.testing.assert_array_equal(state.execution_observer.joints['servo'][1], self.q)

    def test_controller_reference_not_actual_or_desired_alias_maps_to_its_stage(self):
        state = self.state()
        message = self.controller_state()
        message.feedback.positions = [999.]*6
        message.feedback.velocities = [999.]*6
        state.on_controller_state(message)
        self.assertFalse(state.execution_observer.observations['servo'])
        sample = state.execution_observer.observations['controller'][-1]
        self.assertEqual(sample[0], 100.)
        np.testing.assert_allclose(sample[1], (self.matrix+np.diag(self.q*.1)) @ self.velocity)

    def test_official_trajectory_future_header_within_budget_is_valid(self):
        # Official Servo includes expected latency in its trajectory header.
        # That clock budget must agree with core.validate_trajectory (+150ms).
        for method, factory, stage in (('on_servo_trajectory', self.trajectory, 'servo'),
                                        ('on_controller_state', self.controller_state, 'controller')):
            for seconds in (999.86, 1000., 1000.08, 1000.14):
                with self.subTest(method=method, source=seconds):
                    state = self.state()
                    getattr(state, method)(factory(seconds))
                    self.assertEqual(len(state.execution_observer.observations[stage]), 1)
                    self.assertEqual(state.execution_source_stamps[stage], seconds)

    def test_stale_or_excess_future_headers_do_not_refresh_execution_authority(self):
        for method, factory, stage in (('on_servo_trajectory', self.trajectory, 'servo'),
                                        ('on_controller_state', self.controller_state, 'controller')):
            for seconds in (999.84, 1000.16):
                with self.subTest(method=method, source=seconds):
                    state = self.state()
                    getattr(state, method)(factory(seconds))
                    self.assertFalse(state.execution_observer.observations[stage])
                    self.assertNotIn(stage, state.execution_source_stamps)

    def test_duplicate_or_backward_servo_and_controller_stamp_cannot_refresh(self):
        for method, factory, stage in (('on_servo_trajectory', self.trajectory, 'servo'),
                                        ('on_controller_state', self.controller_state, 'controller')):
            state = self.state()
            getattr(state, method)(factory())
            self.monotonic.return_value = 100.02
            getattr(state, method)(factory())
            getattr(state, method)(factory(999.99))
            samples = state.execution_observer.observations[stage]
            self.assertEqual(len(samples), 1)
            self.assertEqual(samples[-1][0], 100.)
            self.assertEqual(state.execution_source_stamps[stage], 1000.)
            self.monotonic.return_value = 100.

    def test_missing_or_malformed_trajectory_and_controller_data_invalidates_stage(self):
        for method, factory, stage in (('on_servo_trajectory', self.trajectory, 'servo'),
                                        ('on_controller_state', self.controller_state, 'controller')):
            for defect in ('velocity', 'positions', 'joint', 'nonfinite'):
                with self.subTest(method=method, defect=defect):
                    state = self.state()
                    getattr(state, method)(factory())
                    message = factory(1000.01)
                    point = message.points[0] if stage == 'servo' else message.reference
                    if defect == 'velocity':
                        point.velocities = []
                    elif defect == 'positions':
                        point.positions = [0.]
                    elif defect == 'joint':
                        message.joint_names[0] = 'unknown_joint'
                    else:
                        point.velocities[0] = float('nan')
                    self.monotonic.return_value = 100.01
                    getattr(state, method)(message)
                    self.assertFalse(state.execution_observer.observations[stage])
                    self.monotonic.return_value = 100.
        state = self.state()
        state.on_servo_trajectory(JointTrajectory())
        self.assertFalse(state.execution_observer.observations['servo'])

    def test_publish_records_exact_controller_time_and_actual_jointjog_mapping(self):
        state = self.state()
        actual_pose = state.actual.return_value[1]
        target = actual_pose.copy(); target[0] = .025; target[1] = -.01
        requested = state.controller.update(target, actual_pose, 99.98, 99.97, .5)
        state.publish_twist(requested)
        jog = state.jog.publish.call_args.args[0]
        self.assertEqual(jog.joint_names, JOINTS)
        expected_velocity = damped_joint_velocity(self.q, requested, state.model, state.config, .5)
        np.testing.assert_allclose(jog.velocities, expected_velocity)
        when, recorded, mapped = state.execution_observer.requests[-1]
        self.assertEqual(when, 99.98)  # Exact controller update, not current 100. receipt.
        np.testing.assert_array_equal(recorded, requested)
        np.testing.assert_allclose(mapped, (self.matrix+np.diag(self.q*.1)) @ np.asarray(jog.velocities))
        diagnostic = state.twist.publish.call_args.args[0]
        self.assertEqual(diagnostic.header.frame_id, 'base_link')
        np.testing.assert_array_equal([diagnostic.twist.linear.x, diagnostic.twist.linear.y,
            diagnostic.twist.linear.z, diagnostic.twist.angular.x, diagnostic.twist.angular.y,
            diagnostic.twist.angular.z], requested)

    def test_inactive_paused_or_noncontroller_output_does_not_record_request(self):
        for changes in ({'active': False}, {'guard_state': 'holding'}, {'different_output': True}):
            with self.subTest(changes=changes):
                state = self.state()
                pose = state.actual.return_value[1]
                target = pose.copy(); target[0] = .02
                command = state.controller.update(target, pose, 100., 99.99, .5)
                for name, value in changes.items():
                    if name != 'different_output':
                        setattr(state, name, value)
                if changes.get('different_output'):
                    command = command*.5
                state.publish_twist(command)
                self.assertFalse(state.execution_observer.requests)
                state.jog.publish.assert_called_once()

    def test_failed_actual_feedback_never_records_a_fake_capability_or_nonzero_jog(self):
        state = self.state()
        pose = state.actual.return_value[1]
        target = pose.copy(); target[0] = .02
        command = state.controller.update(target, pose, 100., 99.99, .5)
        state.actual.side_effect = ValueError('stale actual')
        state.publish_twist(command)
        self.assertFalse(state.execution_observer.requests)
        np.testing.assert_array_equal(state.jog.publish.call_args.args[0].velocities, np.zeros(6))

    def test_reset_clears_controller_history_and_all_observer_state(self):
        state = self.state()
        pose = state.actual.return_value[1]
        target = pose.copy(); target[0] = .02
        command = state.controller.update(target, pose, 100., 99.99, .5)
        state.publish_twist(command)
        state.on_servo_trajectory(self.trajectory())
        state.on_controller_state(self.controller_state())
        state.on_feedback(self.feedback())
        self.assertTrue(state.execution_observer.requests)
        self.assertTrue(state.execution_observer.joints)
        state.reset_controller()
        self.assertIsNone(state.controller.last_command_time)
        self.assertFalse(state.execution_observer.requests)
        self.assertFalse(state.execution_observer.joints)
        self.assertTrue(all(not samples for samples in state.execution_observer.observations.values()))
        self.assertEqual(state.execution_observer.reason, 'no_request')


if __name__ == '__main__':
    unittest.main()
