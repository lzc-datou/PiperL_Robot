"""Future and sampled positions are not bounded by target/feedback displacement."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest

import numpy as np

from scripts.task.servo_tracking.core import JOINTS, RobotModel, FreshJoints, validate_trajectory

try:
    from tests import test_servo_guard_recovery as recovery
    from sensor_msgs.msg import JointState
    ROS = True
except ImportError:
    ROS = False


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


def fixture(joint=0):
    actual = np.array([0., 1.4, -1.2, .3, 1., 0.])
    points = []
    for t in (.02, .10, .20):
        positions, velocities = actual.copy(), np.zeros(6)
        positions[joint] += 1.2 * t
        velocities[joint] = 1.2
        points.append(NS(positions=positions.tolist(), velocities=velocities.tolist(),
                         time_from_start=NS(sec=0, nanosec=round(t * 1e9))))
    trajectory = NS(joint_names=JOINTS.copy(), header=NS(stamp=NS(sec=1000, nanosec=0)), points=points)
    config = dict(joint_speed=1.5, command_timeout=.2, trajectory_horizon=.3)
    model = RobotModel(URDF.read_text(), tip='link6')
    return actual, trajectory, model, config


class FutureTrajectoryTests(unittest.TestCase):
    def test_each_axis_can_have_valid_future_displacement_above_current_error_cap(self):
        for joint in range(6):
            actual, trajectory, model, config = fixture(joint)
            before = deepcopy(trajectory)
            self.assertGreater(abs(trajectory.points[-1].positions[joint]-actual[joint]), .1)
            with self.subTest(joint=joint+1):
                validate_trajectory(trajectory, actual, model, config, 1000.01)
                self.assertEqual(trajectory, before)
                self.assertEqual(config['joint_speed'], 1.5)

    def test_future_validation_keeps_all_existing_kinematic_and_format_checks(self):
        for failure, reason in (('joint_names', '完整关节'), ('positions', '模型限位'),
                                ('velocity', '关节速度越限'), ('segment', '相邻轨迹点速度'),
                                ('duplicate_time', '轨迹时间'), ('horizon', '轨迹时间'),
                                ('source_time', '时间戳')):
            actual, trajectory, model, config = fixture()
            if failure == 'joint_names':
                trajectory.joint_names.pop()
            elif failure == 'positions':
                trajectory.points[-1].positions[0] = model.upper[0] + .001
            elif failure == 'velocity':
                trajectory.points[-1].velocities[0] = 1.6
            elif failure == 'segment':
                trajectory.points[-1].positions[0] = actual[0] + .4
            elif failure == 'duplicate_time':
                trajectory.points[-1].time_from_start.nanosec = 100_000_000
            elif failure == 'horizon':
                trajectory.points[-1].time_from_start.nanosec = 310_000_000
            else:
                trajectory.header.stamp.sec -= 1
            with self.subTest(failure=failure), self.assertRaisesRegex(ValueError, reason):
                validate_trajectory(trajectory, actual, model, config, 1000.01)


@unittest.skipUnless(ROS, 'Use ROS runtime wrapper; every transport remains fake')
class CurrentCommandTrackingTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.).start()
        self.wall = patch('scripts.task.servo_tracking.guard.time.time', return_value=1000.01).start()
        self.addCleanup(patch.stopall)
        self.actual, self.trajectory, model, config = fixture()
        self.g = recovery.ServoGuardRecoveryTests.guard(self)
        self.g.state = 'running'
        self.g.model = model
        self.g.config.update(config)
        self.g.health.return_value = self.actual.copy()
        self.g.feedback.get.return_value = self.actual.copy()
        self.g.commanded = FreshJoints()
        self.g.get_clock = Mock()
        self.g.get_clock.return_value.now.return_value.to_msg.return_value = JointState().header.stamp

    def commanded(self, displacement):
        msg = JointState()
        msg.name = JOINTS.copy()
        msg.position = (self.actual + [displacement, 0., 0., 0., 0., 0.]).tolist()
        msg.header.stamp.sec = 1000
        msg.header.stamp.nanosec = 10_000_000
        return msg

    def test_valid_future_and_sampled_reference_can_both_lead_feedback(self):
        self.g.on_trajectory(self.trajectory)
        self.assertEqual(self.g.state, 'running')
        self.g.output.publish.assert_called_once_with(self.trajectory)
        self.g.driver_output.publish.assert_not_called()
        # A legal current sampled command is not rejected merely for leading feedback.
        self.g.on_commanded(self.commanded(.2))
        self.assertEqual(self.g.state, 'running')
        self.g.driver_output.publish.assert_called_once()

    def test_visual_guard_forwards_bounded_position_knots_after_checking_velocities(self):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self.g.visual_mode = True
        msg = JointTrajectory(joint_names=JOINTS)
        msg.header.stamp.sec = 1000
        for knot in self.trajectory.points:
            point = JointTrajectoryPoint(positions=knot.positions, velocities=knot.velocities)
            point.time_from_start.nanosec = knot.time_from_start.nanosec
            msg.points.append(point)
        self.g.on_trajectory(msg)
        forwarded = self.g.output.publish.call_args.args[0]
        self.assertEqual(forwarded.header, msg.header)
        self.assertEqual(forwarded.joint_names, msg.joint_names)
        for new, original in zip(forwarded.points, msg.points, strict=True):
            self.assertEqual(new.positions, original.positions)
            self.assertEqual(new.time_from_start, original.time_from_start)
            self.assertFalse(new.velocities or new.accelerations)
            self.assertTrue(original.velocities)
        self.g.output.publish.reset_mock()
        msg.points[0].velocities[0] = 10.
        self.g.on_trajectory(msg)
        self.assertEqual(self.g.state, 'fault')
        hold = self.g.output.publish.call_args.args[0]
        self.assertEqual(len(hold.points), 1)
        np.testing.assert_allclose(hold.points[0].positions, self.actual)
        np.testing.assert_array_equal(hold.points[0].velocities, np.zeros(6))

    def test_old_point_one_threshold_does_not_interrupt_tracking(self):
        self.g.on_commanded(self.commanded(.100001))
        self.assertEqual(self.g.state, 'running')
        self.g.driver_output.publish.assert_called_once()

    def test_current_command_with_small_reference_lead_can_pass(self):
        self.g.on_commanded(self.commanded(.05))
        self.assertEqual(self.g.state, 'running')
        self.g.driver_output.publish.assert_called_once()
        np.testing.assert_allclose(self.g.driver_output.publish.call_args.args[0].position,
                                   self.actual + [.05, 0., 0., 0., 0., 0.])


if __name__ == '__main__':
    unittest.main()
