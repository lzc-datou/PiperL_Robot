"""Explicit official home planning; fake feedback/services, never ROS or CAN."""
from concurrent.futures import Future
import json
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from scripts.task.servo_tracking.node import Backend
    from tests import test_servo_raw_reset as raw_reset
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; every service is fake')
class ServoHomeTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)

    def state(self, **changes):
        state, raw = raw_reset.ServoRawResetTests.state(self, [0., 1.4, -1.2, .3, 1., 0.])
        state.active = True
        state.target = state.model.pose(raw)
        state.approved_target = state.target.copy()
        state.motion_recovery_required = state.recovery_hold = True
        state.joint_revision = -1
        state.joint_plan, state.joint_command = Mock(), Mock()
        for client in (state.joint_plan, state.joint_command):
            client.service_is_ready.return_value = True
            client.call_async.side_effect = lambda request: Future()
        for name in ('queue_joint_plan', 'joint_request_tick', 'joint_plan_done'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        for name, value in changes.items():
            setattr(state, name, value)
        return state

    def send_plan(self, state):
        state.guard_can_plan = True  # Independent guard confirmed stop/hold/action retirement.
        Backend.joint_request_tick(state)
        return state.joint_plan.call_async.call_args.args[0], state.joint_future

    def done(self, state, plan_id='home-plan'):
        Backend.on_guard(state, NS(data=json.dumps(dict(state='joint_done', reason='到位',
            plan_id=plan_id, can_plan=True))))

    def test_home_revokes_tracking_and_waits_for_closed_barrier_then_requests_hard_bound_zero(self):
        for mode in ('servo', 'point'):
            with self.subTest(mode=mode):
                state = self.state(control_mode=mode)
                self.assertEqual(state.model.condition_number(np.zeros(6)), float('inf'))
                with self.assertRaises(ValueError):
                    state.model.check_joints(np.zeros(6), state.config['joint_margin'])
                response = Backend.home_service(state, None, NS())
                self.assertTrue(response.success, response.message)
                self.assertFalse(state.active or state.motion_recovery_required or state.recovery_hold)
                self.assertIsNone(state.approved_target)
                self.assertTrue(state.joint_busy)
                self.assertEqual(state.control_mode, mode)
                Backend.joint_request_tick(state)
                state.joint_plan.call_async.assert_not_called()
                request, _ = self.send_plan(state)
                self.assertEqual(request.purpose, 'home')
                self.assertTrue(request.auto_execute)
                np.testing.assert_array_equal(request.joints, np.zeros(6))
                self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))
                state.begin_tracking.assert_not_called()

    def test_completion_aligns_to_raw_measured_fk_and_leaves_tracking_off(self):
        state = self.state()
        old_target = state.target.copy()
        Backend.home_service(state, None, NS())
        _, plan = self.send_plan(state)
        plan.set_result(NS(success=True, plan_id='home-plan', message='执行中'))
        measured = np.array([0., -.0005, .0002, 0., 0., 0.])
        state.feedback.get.return_value = measured
        self.done(state)
        self.assertFalse(state.active or state.start_pending or state.joint_busy)
        self.assertEqual(state.reason, '已回零')
        np.testing.assert_array_equal(measured, [0., -.0005, .0002, 0., 0., 0.])
        np.testing.assert_allclose(state.target, state.model.pose(measured))
        self.assertFalse(np.allclose(state.target, old_target))
        np.testing.assert_array_equal(state.publish_ghost.call_args.args[0], measured)
        state.begin_tracking.assert_not_called()
        self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_stale_invalid_or_missing_panel_feedback_blocks_home_after_stopping(self):
        for failure in ('stale', 'outside', 'nonfinite', 'panel'):
            with self.subTest(failure=failure):
                state = self.state()
                if failure == 'stale':
                    state.feedback.get.side_effect = ValueError('关节反馈缺失或超时')
                elif failure == 'outside':
                    state.feedback.get.return_value = np.array([0., -.004, 0., 0., 0., 0.])
                elif failure == 'nonfinite':
                    state.feedback.get.return_value = np.array([float('nan')] * 6)
                else:
                    state.panel_seen = 99.
                response = Backend.home_service(state, None, NS())
                self.assertFalse(response.success)
                self.assertFalse(state.active or state.joint_busy)
                self.assertIsNone(state.approved_target)
                state.joint_plan.call_async.assert_not_called()
                self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_disabled_motor_response_never_auto_enables_or_restarts_tracking(self):
        state = self.state()
        Backend.home_service(state, None, NS())
        _, plan = self.send_plan(state)
        plan.set_result(NS(success=False, plan_id='', message='实机未使能，请使用官方使能服务'))
        self.assertFalse(state.active or state.joint_busy)
        self.assertIn('实机未使能', state.reason)
        state.begin_tracking.assert_not_called()
        self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_stop_cancels_late_home_plan_and_ignores_old_completion(self):
        state = self.state()
        Backend.home_service(state, None, NS())
        _, plan = self.send_plan(state)
        target = state.target.copy()
        Backend.stop_service(state, None, NS())
        plan.set_result(NS(success=True, plan_id='home-plan', message='late'))
        cancel = state.joint_command.call_async.call_args.args[0]
        self.assertEqual((cancel.plan_id, cancel.command), ('home-plan', 'cancel'))
        self.done(state)
        self.assertEqual(state.reason, '已停止')
        self.assertFalse(state.active or state.joint_busy)
        np.testing.assert_array_equal(state.target, target)
        state.begin_tracking.assert_not_called()

    def test_other_joint_goals_keep_margin_and_mode_switch_cancels_queued_home(self):
        state = self.state()
        with self.assertRaises(ValueError):
            Backend.queue_joint_plan(state, np.zeros(6), 'point')
        Backend.home_service(state, None, NS())
        Backend.mode_service(state, NS(data=True), NS())
        self.assertFalse(state.active or state.joint_busy)
        self.assertIsNone(state.joint_wait)
        Backend.joint_request_tick(state)
        state.joint_plan.call_async.assert_not_called()


if __name__ == '__main__':
    unittest.main()
