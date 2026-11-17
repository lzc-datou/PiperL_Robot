"""Settled Servo hold latch with fake services only; never connects hardware."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest

try:
    from scripts.task.servo_tracking.guard import Guard
    from scripts.task.servo_tracking.core import JOINTS
    from tests import test_servo_guard_recovery as recovery
    from sensor_msgs.msg import JointState
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; all control services are fakes')
class ServoHoldGuardTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.g = recovery.ServoGuardRecoveryTests.guard(self)
        self.g.state = 'running'

    def enter_hold(self):
        result = self.g.hold_service(NS(), NS())
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.g.state, 'holding')

    def acknowledge_stop(self):
        self.g.gate.complete()
        self.g.hold.complete()
        self.g.tick()

    def test_hold_pauses_servo_closes_gate_and_requests_official_hold(self):
        self.enter_hold()
        self.assertTrue(self.g.pause_servo.calls[-1][0].data)
        self.assertFalse(self.g.gate.calls[-1][0].data)
        self.assertEqual(len(self.g.hold.calls), 1)
        point = self.g.output.publish.call_args.args[0].points[0]
        self.assertEqual(list(point.positions), self.g.feedback.get.return_value.tolist())
        self.assertEqual(list(point.velocities), [0.] * 6)

    def test_held_session_never_forwards_old_controller_or_servo_commands(self):
        self.enter_hold()
        self.g.output.publish.reset_mock()
        message = JointState()
        message.name, message.position = JOINTS, [2.] * 6
        self.g.on_commanded(message)
        self.g.on_trajectory(NS())
        self.g.driver_output.publish.assert_not_called()
        self.g.output.publish.assert_not_called()

    def test_hold_only_accepts_a_healthy_running_session(self):
        self.g.state = 'idle'
        self.assertFalse(self.g.hold_service(NS(), NS()).success)
        self.assertEqual(self.g.gate.calls, [])
        self.g.state = 'running'
        self.g.health.side_effect = ValueError('stale real feedback')
        self.assertFalse(self.g.hold_service(NS(), NS()).success)
        self.assertEqual(self.g.state, 'fault')
        self.assertFalse(self.g.gate.calls[-1][0].data)

    def test_held_session_keeps_backend_heartbeat_watchdog(self):
        self.enter_hold()
        self.g.heartbeat = 99.
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertIn('心跳超时', self.g.reason)
        self.assertFalse(self.g.gate.calls[-1][0].data)

    def test_held_session_keeps_real_feedback_watchdog(self):
        self.enter_hold()
        self.g.health.side_effect = ValueError('feedback expired')
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertIn('feedback expired', self.g.reason)

    def test_held_session_does_not_require_servo_output_or_auto_resume(self):
        self.enter_hold()
        self.acknowledge_stop()
        self.g.last_trajectory = self.g.servo_received = 0.
        self.g.on_servo(NS(code=0, message='no warning'))
        self.g.tick()
        self.assertEqual(self.g.state, 'holding')
        self.assertFalse(any(request.data for request, _ in self.g.gate.calls))

    def test_resume_waits_for_gate_close_and_official_hold_completion(self):
        self.enter_hold()
        self.assertFalse(self.g.enable(NS(data=True), NS()).success)
        self.g.gate.complete()
        self.g.tick()
        self.assertTrue(self.g.closed)
        self.assertFalse(self.g.enable(NS(data=True), NS()).success)
        self.g.hold.complete()
        response = self.g.enable(NS(data=True), NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.g.state, 'pausing')
        self.assertFalse(any(request.data for request, _ in self.g.gate.calls))
        recovery.ServoGuardRecoveryTests.sync_and_reset(self, self.g)
        self.g.pause_servo.complete()
        self.assertEqual(self.g.state, 'opening')
        self.g.gate.complete()
        self.assertEqual(self.g.state, 'running')

    def test_resume_also_waits_for_old_joint_action_terminal_barrier(self):
        self.enter_hold()
        self.acknowledge_stop()
        self.g.joint = NS(has_unresolved_actions=Mock(return_value=True), invalidate=Mock())
        response = self.g.enable(NS(data=True), NS())
        self.assertFalse(response.success)
        self.assertIn('终态', response.message)
        self.assertEqual(self.g.state, 'holding')

    def test_late_old_gate_callback_recloses_and_preserves_hold(self):
        old_epoch = self.g.epoch
        self.enter_hold()
        old_open = Future()
        old_open.set_result(NS(success=True))
        self.g.opened(old_open, old_epoch)
        self.assertEqual(self.g.state, 'holding')
        self.assertFalse(self.g.gate.calls[-1][0].data)
        self.assertFalse(any(request.data for request, _ in self.g.gate.calls))

    def test_late_old_unpause_callback_never_reopens_held_session(self):
        old_epoch = self.g.epoch
        self.enter_hold()
        old_unpause = Future()
        old_unpause.set_result(NS(success=True))
        self.g.reset_servo(old_unpause, old_epoch)
        self.assertEqual(self.g.state, 'holding')
        self.assertTrue(self.g.pause_servo.calls[-1][0].data)
        self.assertFalse(any(request.data for request, _ in self.g.gate.calls))

    def test_competing_controller_source_revokes_held_session(self):
        self.enter_hold()
        self.g.count_publishers.return_value = 2
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertIn('其他轨迹控制来源', self.g.reason)


if __name__ == '__main__':
    unittest.main()
