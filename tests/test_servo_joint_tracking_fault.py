"""Retired joint tracking-error threshold stays removed; no hardware I/O."""
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

try:
    from scripts.task.servo_tracking.core import FreshJoints, JOINTS, RobotModel, load_config
    from sensor_msgs.msg import JointState
    from tests import test_servo_guard_recovery as recovery
    ROS = recovery.ROS
except ImportError:
    ROS = False

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


@unittest.skipUnless(ROS, 'Use ROS wrapper; all publishers/services are fake')
class JointTrackingErrorRemovedTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.).start()
        self.wall = patch('scripts.task.servo_tracking.guard.time.time', return_value=1000.).start()
        self.addCleanup(patch.stopall)
        self.g = recovery.ServoGuardRecoveryTests.guard(self)
        self.actual = np.array([0., 1.4, -1.2, .3, .9, 0.])
        self.g.state = 'running'
        self.g.model = RobotModel(URDF.read_text(), tip='link6')
        self.g.health.return_value = self.actual.copy()
        self.g.feedback, self.g.commanded = FreshJoints(), FreshJoints()
        # Even a legacy tiny threshold must not re-enable the retired rule.
        self.g.config['max_joint_tracking_error'] = .000001

    def command(self, q):
        message = JointState()
        message.name = list(reversed(JOINTS))
        message.position = list(map(float, reversed(q)))
        ns = round(self.wall.return_value*1e9)
        message.header.stamp.sec = ns//1000000000
        message.header.stamp.nanosec = ns%1000000000
        return message

    def test_each_axis_can_exceed_old_point_one_without_a_tracking_fault(self):
        for axis in range(6):
            for sign in (-1., 1.):
                with self.subTest(axis=axis, sign=sign):
                    self.now.return_value += .01
                    self.wall.return_value += .01
                    self.g.state, self.g.last_forwarded = 'running', None
                    candidate = self.actual.copy(); candidate[axis] += sign*.25
                    self.g.on_commanded(self.command(candidate))
                    self.assertEqual(self.g.state, 'running')
                    np.testing.assert_allclose(self.g.driver_output.publish.call_args.args[0].position, candidate)
        self.g.get_logger.return_value.error.assert_not_called()
        self.assertEqual(self.g.gate.calls, [])

    def test_persistent_error_has_no_delayed_replacement_stop(self):
        # A slow, legal reference can move >.1rad while feedback remains fixed;
        # the removed test must not be reintroduced as a timer or new threshold.
        for index in range(251):
            self.now.return_value = 100.+index*.01
            self.wall.return_value = 1000.+index*.01
            candidate = self.actual.copy(); candidate[3] += index*.001
            self.g.on_commanded(self.command(candidate))
            self.assertEqual(self.g.state, 'running')
        self.assertEqual(self.g.driver_output.publish.call_count, 251)
        self.assertGreater(abs(self.g.last_forwarded[3]-self.actual[3]), .1)
        self.assertEqual(self.g.gate.calls, [])

    def test_actual_position_limits_are_still_validated(self):
        candidate = self.actual.copy(); candidate[2] = self.g.model.upper[2]+.01
        self.g.on_commanded(self.command(candidate))
        self.assertEqual(self.g.state, 'fault')
        self.assertIn('模型限位', self.g.reason)
        self.g.driver_output.publish.assert_not_called()

    def test_command_step_velocity_check_is_independent_and_retained(self):
        self.g.last_forwarded, self.g.last_forwarded_time = self.actual.copy(), 99.995
        candidate = self.actual.copy(); candidate[3] += .25
        self.g.on_commanded(self.command(candidate))
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.g.reason, '控制器关节指令跳变')
        self.g.driver_output.publish.assert_not_called()

    def test_stale_measurements_still_revoke_forwarding(self):
        self.g.health.side_effect = ValueError('关节反馈缺失或超时')
        candidate = self.actual.copy(); candidate[0] += .25
        self.g.on_commanded(self.command(candidate))
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.g.reason, '关节反馈缺失或超时')
        self.g.driver_output.publish.assert_not_called()

    def test_task_configuration_no_longer_contains_tracking_threshold(self):
        config = load_config(ROOT/'scripts/task/servo_tracking/config/config.yaml')
        self.assertNotIn('max_joint_tracking_error', config)


if __name__ == '__main__':
    unittest.main()
