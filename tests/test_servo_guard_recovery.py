"""Guard recovery protocol with fake services only; no ROS init or hardware."""
from concurrent.futures import Future
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest
import numpy as np

try:
    from scripts.task.servo_tracking.guard import Guard
    from scripts.task.servo_tracking.core import JOINTS
    from scripts.task.servo_tracking.stop_observer import StopObserver
    from sensor_msgs.msg import JointState
    ROS = True
except ImportError:
    ROS = False


class FakeService:
    def __init__(self):
        self.ready = True
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future

    def complete(self, index=-1, success=True):
        self.calls[index][1].set_result(NS(success=success, message=''))


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; every service is fake')
class ServoGuardRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def guard(self, real=True):
        q = np.array([0., 1.4, -1.2, .3, 1., 0.])
        state = NS(real=real, state='idle', reason='idle', epoch=0, closed=True,
                   gamepad_mode=False, gamepad_received=0., gamepad_error='',
                   stop_has_been_requested=False, stop_requested_at=0., stop_generation=0,
                   stop_settled=False,
                   stop_confirmed=False, stop_hold_required=False, stop_observer=StopObserver(),
                   closing=None, stop_future=None, hold_pending=False, heartbeat=100., stage_time=0.,
                   stable_since=0., stable_q=None, last_trajectory=100.,
                   servo_code=0, servo_received=100., last_forwarded=None,
                   last_forwarded_time=0., action_active=False,
                   config={'feedback_timeout': .25, 'hard_stop_singularity_threshold': 30.,
                           'max_joint_tracking_error': .1, 'joint_speed': .3},
                   feedback=Mock(), commanded=Mock(), model=Mock(), output=Mock(),
                   driver_output=Mock(), gate=FakeService(), hold=FakeService(),
                   pause_servo=FakeService(), cancel_action=FakeService(), get_logger=Mock(), get_clock=Mock(),
                   count_publishers=Mock(return_value=1))
        for name, method in Guard.__dict__.items():
            if callable(method) and not name.startswith('__'):
                setattr(state, name, MethodType(method, state))
        state.feedback.get.return_value = q.copy()
        state.commanded.get.return_value = q.copy()
        state.feedback.stamp = state.commanded.stamp = 100.
        state.model.condition_number.return_value = 10.
        state.output.get_subscription_count.return_value = 1
        state.health = Mock(return_value=q.copy())
        return state

    def start(self, state):
        response = state.enable(NS(data=True), NS())
        self.assertTrue(response.success, response.message)
        return response

    def sync_and_reset(self, state):
        state.pause_servo.complete()  # pause acknowledged before controller alignment
        self.assertEqual(state.state, 'syncing')
        self.now.return_value = 100.01
        state.feedback.stamp = state.commanded.stamp = 100.01
        state.tick()
        self.now.return_value = 100.32
        state.feedback.stamp = state.commanded.stamp = 100.32
        state.heartbeat = 100.32
        state.tick()
        self.assertEqual(state.state, 'resetting')
        self.assertFalse(state.pause_servo.calls[-1][0].data)

    def test_start_orders_pause_alignment_reset_and_real_gate(self):
        state = self.guard()
        self.start(state)
        self.assertEqual(state.state, 'pausing')
        self.assertTrue(state.pause_servo.calls[-1][0].data)
        state.output.publish.assert_not_called()
        self.assertEqual(state.gate.calls, [])
        self.sync_and_reset(state)
        self.assertEqual(state.gate.calls, [])
        state.pause_servo.complete()
        self.assertEqual(state.state, 'opening')
        self.assertTrue(state.gate.calls[-1][0].data)
        state.gate.complete()
        self.assertEqual(state.state, 'running')

    def test_sim_uses_the_same_reset_without_a_real_gate(self):
        state = self.guard(real=False)
        self.start(state)
        self.sync_and_reset(state)
        state.pause_servo.complete()
        self.assertEqual(state.state, 'running')
        self.assertEqual(state.gate.calls, [])

    def test_singularity_pauses_closes_holds_and_never_auto_resumes(self):
        state = self.guard()
        state.state = 'running'
        state.on_servo(NS(code=2, message='singularity'))
        self.assertEqual(state.state, 'paused')
        self.assertFalse(state.gate.calls[-1][0].data)
        self.assertEqual(len(state.hold.calls), 1)
        self.assertTrue(state.pause_servo.calls[-1][0].data)
        state.on_servo(NS(code=0, message='ok'))
        self.assertEqual(state.state, 'paused')
        state.output.publish.reset_mock()
        state.on_trajectory(NS())
        msg = JointState()
        msg.name, msg.position = JOINTS, [0.] * 6
        state.on_commanded(msg)
        state.driver_output.publish.assert_not_called()
        state.output.publish.assert_not_called()
        state.tick()
        self.assertEqual(state.state, 'paused')

    def test_paused_state_keeps_feedback_and_heartbeat_watchdogs(self):
        for failure in ('feedback', 'heartbeat'):
            with self.subTest(failure=failure):
                state = self.guard()
                state.state = 'paused'
                if failure == 'feedback':
                    state.health.side_effect = ValueError('stale feedback')
                else:
                    state.heartbeat = 99.
                state.tick()
                self.assertEqual(state.state, 'fault')
                self.assertFalse(state.gate.calls[-1][0].data)

    def test_hard_servo_faults_do_not_become_recoverable_pauses(self):
        for code in (-1, 5):
            with self.subTest(code=code):
                state = self.guard()
                state.state = 'paused'
                state.on_servo(NS(code=code, message='fault'))
                self.assertEqual(state.state, 'fault')

    def test_resume_waits_for_gate_close_and_official_hold_acknowledgement(self):
        state = self.guard()
        state.state = 'running'
        state.on_servo(NS(code=2, message='singularity'))
        response = state.enable(NS(data=True), NS())
        self.assertFalse(response.success)
        state.gate.complete()
        state.tick()
        self.assertTrue(state.closed)
        response = state.enable(NS(data=True), NS())
        self.assertFalse(response.success)  # hold is still outstanding
        state.hold.complete()
        self.start(state)
        self.assertEqual(state.state, 'pausing')

    def test_guard_does_not_duplicate_native_servo_conditioning_gate(self):
        for value in (30., 80., float('inf'), float('nan')):
            with self.subTest(value=value):
                state = self.guard()
                state.model.condition_number.return_value = value
                response = state.enable(NS(data=True), NS())
                self.assertTrue(response.success, response.message)
                self.assertEqual(len(state.pause_servo.calls), 1)
                self.assertEqual(state.state, 'pausing')
        state = self.guard()
        self.start(state)
        self.sync_and_reset(state)
        state.pause_servo.complete()
        state.model.condition_number.return_value = 31.
        state.gate.complete()
        self.assertEqual(state.state, 'running')
        # Native Servo still owns the decision to scale or stop Cartesian motion.
        state.on_servo(NS(code=2, message='native constraint'))
        self.assertEqual(state.state, 'paused')
        self.assertFalse(state.gate.calls[-1][0].data)

    def test_native_joint_boundary_holds_session_instead_of_faulting(self):
        state = self.guard()
        state.state = 'running'
        state.on_servo(NS(code=6, message='joint bound'))
        self.assertEqual(state.state, 'paused')
        self.assertIn('边界暂停', state.reason)
        self.assertEqual(state.pause_cause, 'joint_bound')
        self.assertFalse(state.gate.calls[-1][0].data)
        state.on_servo(NS(code=0, message='clear'))
        self.assertEqual(state.state, 'paused')
        self.assertEqual(state.pause_cause, 'joint_bound')

    def test_visual_bound_hint_uses_only_a_fresh_validated_servo_endpoint(self):
        for age in (.03, .25):
            state = self.guard()
            state.state, state.visual_mode = 'running', True
            state.config['command_timeout'] = .2
            state.last_trajectory = 100.-age
            state.last_visual_knot = [0., 1., -1., 2.20, .5, -1.]
            state.on_servo(NS(code=6, message='joint bound'))
            self.assertEqual(state.state, 'paused')
            self.assertEqual(state.bound_reference, state.last_visual_knot if age < .2 else None)
            self.assertFalse(state.gate.calls[-1][0].data)

    def test_native_pause_cause_survives_status_updates_and_stop_clears_it(self):
        state = self.guard()
        state.state = 'running'
        state.on_servo(NS(code=2, message='native singular halt'))
        self.assertEqual(state.pause_cause, 'singularity')
        state.on_servo(NS(code=0, message='zero status'))
        self.assertEqual(state.pause_cause, 'singularity')
        state.stop('user stop')
        self.assertEqual(state.pause_cause, '')

    def test_missing_pause_service_and_failed_pause_reject_start(self):
        state = self.guard()
        state.pause_servo.ready = False
        self.assertFalse(state.enable(NS(data=True), NS()).success)
        self.assertEqual(state.gate.calls, [])
        state.pause_servo.ready = True
        self.start(state)
        state.pause_servo.complete(success=False)
        self.assertEqual(state.state, 'fault')
        self.assertFalse(state.gate.calls[-1][0].data)

    def test_failed_hold_never_authorizes_resume(self):
        state = self.guard()
        state.stop_future = Future()
        state.stop_future.set_exception(RuntimeError('hold unavailable'))
        self.assertFalse(state.enable(NS(data=True), NS()).success)
        self.assertEqual(state.pause_servo.calls, [])

    def test_missing_hold_service_stays_blocked_until_later_acknowledgement(self):
        state = self.guard()
        state.state = 'running'
        state.hold.ready = False
        state.stop('singularity', paused=True)
        state.gate.complete()
        state.tick()
        self.assertFalse(state.enable(NS(data=True), NS()).success)
        state.hold.ready = True
        state.tick()
        self.assertEqual(len(state.hold.calls), 1)
        self.assertFalse(state.enable(NS(data=True), NS()).success)
        state.hold.complete()
        self.start(state)

    def test_each_preparation_stage_has_a_timeout(self):
        for phase in ('pausing', 'syncing', 'resetting', 'opening'):
            with self.subTest(phase=phase):
                state = self.guard()
                state.state, state.stage_time = phase, 96.
                state.tick()
                self.assertEqual(state.state, 'fault')
                self.assertIn('超时', state.reason)

    def test_late_reset_ack_rechecks_backend_before_opening_gate(self):
        state = self.guard()
        self.start(state)
        self.sync_and_reset(state)
        state.heartbeat = 99.
        state.pause_servo.complete()
        self.assertEqual(state.state, 'fault')
        self.assertFalse(any(request.data for request, _ in state.gate.calls))

    def test_gate_ack_rechecks_feedback_before_entering_running(self):
        state = self.guard()
        self.start(state)
        self.sync_and_reset(state)
        state.pause_servo.complete()
        state.health.side_effect = ValueError('stale feedback')
        state.gate.complete()
        self.assertEqual(state.state, 'fault')
        self.assertFalse(state.gate.calls[-1][0].data)

    def test_late_pause_ack_cannot_restart_after_stop(self):
        state = self.guard()
        self.start(state)
        pending = state.pause_servo.calls[-1][1]
        state.stop('user stop')
        state.output.publish.reset_mock()
        pending.set_result(NS(success=True, message=''))
        self.assertEqual(state.state, 'idle')
        state.output.publish.assert_not_called()
        self.assertFalse(any(request.data for request, _ in state.gate.calls))

    def test_late_unpause_ack_reasserts_pause_and_cannot_open_gate(self):
        state = self.guard()
        self.start(state)
        self.sync_and_reset(state)
        pending = state.pause_servo.calls[-1][1]
        state.stop('user stop')
        pending.set_result(NS(success=True, message=''))
        self.assertEqual(state.state, 'idle')
        self.assertTrue(state.pause_servo.calls[-1][0].data)
        self.assertFalse(any(request.data for request, _ in state.gate.calls))

    def test_late_unpause_from_previous_epoch_revokes_new_preparation(self):
        state = self.guard()
        self.start(state)
        self.sync_and_reset(state)
        pending = state.pause_servo.calls[-1][1]
        state.stop('user stop')
        state.gate.complete()
        state.hold.complete()
        state.tick()
        self.start(state)
        pending.set_result(NS(success=True, message=''))
        self.assertEqual(state.state, 'fault')
        self.assertTrue(state.pause_servo.calls[-1][0].data)
        self.assertFalse(any(request.data for request, _ in state.gate.calls))

    def test_pause_also_cancels_competing_actions(self):
        state = self.guard()
        state.state = 'paused'
        state.on_action_status(NS(status_list=[NS(status=2)]))
        self.assertEqual(state.state, 'fault')
        self.assertEqual(len(state.cancel_action.calls), 1)


if __name__ == '__main__':
    unittest.main()
