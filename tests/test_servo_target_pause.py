"""Rejected targets pause a session; fake transports never initialize ROS or CAN."""
from concurrent.futures import Future
import json
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from scripts.task.servo_tracking.core import JOINTS
    from scripts.task.servo_tracking.node import Backend, pose_message
    from tests import test_servo_hold_backend as hold_backend
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; all services are fake')
class ServoTargetPauseTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)

    def state(self, **changes):
        state, q, actual = hold_backend.ServoHoldBackendTests.state(self)
        state.ik = Mock()
        state.ik.service_is_ready.return_value = True
        state.ik.call_async.side_effect = lambda request: Future()
        state.publish_ghost = Mock()
        state.preview_q = q.copy()
        state.ghost_revision = state.revision
        state.hold_reason = ''
        state.mode, state.workspace_visible = 'real', False
        state.raw_actual = state.actual
        state.status, state.server = Mock(), Mock()
        state.marker_color = ''
        state.server.get.return_value = None
        state.model.joint_bound_violations.return_value = []
        state.model.radius = 1.
        state.model.lower = np.array([-2.6, 0., -2.9, -2.2, -1.7, -3.1])
        state.model.upper = np.array([2.6, 3.14, 0., 2.2, 1.7, 3.1])
        state.model.planning_joints.side_effect = lambda q, zero_tolerance=0.: np.asarray(q).copy()
        state.get_clock = Mock()
        for name in ('reject', 'validation_tick', 'ik_done', 'update_target'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        for name, value in changes.items():
            setattr(state, name, value)
        return state, q, actual

    def holding(self, state):
        Backend.on_guard(state, NS(data=json.dumps(dict(state='holding', reason='保持中'))))

    def assert_paused(self, state, reason):
        self.assertTrue(state.active)
        self.assertFalse(state.valid)
        self.assertIsNone(state.approved_target)
        self.assertIsNone(state.preview_q)
        self.assertEqual(state.pause_revision, state.revision)
        self.assertEqual(state.valid_revision, state.revision)
        self.assertIn(reason, state.reason)
        state.publish_ghost.assert_called_with()
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))
        self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_unreachable_ik_revokes_previous_goal_and_requests_one_hold(self):
        state, _, _ = self.state()
        pending = Future()
        state.pending = pending
        pending.set_result(NS(error_code=NS(val=-31, SUCCESS=1)))
        Backend.ik_done(state, pending, state.revision)
        self.assert_paused(state, '目标不可达或碰撞')
        state.hold_gate.call_async.assert_called_once()
        self.assertIsNotNone(state.hold_future)
        Backend.reject(state, '目标不可达或碰撞')
        state.hold_gate.call_async.assert_called_once()

    def test_inactive_rejection_preserves_preparation_completion_or_explicit_stop_reason(self):
        for reason in ('准备位已到达；原目标保留，请调整或重置目标后开启跟踪', '已停止'):
            with self.subTest(reason=reason):
                state, _, _ = self.state(active=False, reason=reason)
                epoch = state.joint_epoch
                Backend.reject(state, '目标处于关节限位保护区')
                self.assertEqual(state.reason, reason)
                self.assertEqual(state.reachability, '目标处于关节限位保护区')
                self.assertFalse(state.active or state.valid)
                self.assertEqual(state.valid_revision, state.revision)
                self.assertIsNone(state.approved_target)
                self.assertEqual(state.joint_epoch, epoch)
                state.hold_gate.call_async.assert_not_called()
                state.gate.call_async.assert_not_called()
                np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))

    def test_target_limit_margin_rejection_keeps_session_without_permission(self):
        state, q, _ = self.state()
        state.model.check_joints.side_effect = [q, ValueError('保护区')]
        pending = Future()
        state.pending = pending
        pending.set_result(NS(error_code=NS(val=1, SUCCESS=1),
            solution=NS(joint_state=NS(name=JOINTS, position=q))))
        Backend.ik_done(state, pending, state.revision)
        self.assert_paused(state, '目标处于关节限位保护区')

    def test_ik_timeout_cancels_old_reply_and_keeps_session(self):
        state, _, _ = self.state(pending=Future(), pending_since=99.)
        pending = state.pending
        Backend.validation_tick(state)
        self.assertTrue(pending.cancelled())
        self.assertIsNone(state.pending)
        self.assert_paused(state, '逆解检查超时')
        Backend.ik_done(state, pending, state.revision)
        self.assertIsNone(state.approved_target)

    def test_waiting_too_long_for_current_verification_holds_instead_of_stopping(self):
        state, _, _ = self.state(verification_gap_since=99., pending=Future())
        pending = state.pending
        Backend.tick(state)
        self.assertTrue(pending.cancelled())
        self.assert_paused(state, '等待目标验证')
        state.hold_gate.call_async.assert_called_once()

    def test_already_held_or_paused_rejection_does_not_send_redundant_hold(self):
        for guard_state in ('holding', 'paused'):
            with self.subTest(guard_state=guard_state):
                state, _, _ = self.state(guard_state=guard_state)
                Backend.reject(state, '目标超出可达范围')
                self.assert_paused(state, '目标超出可达范围')
                state.hold_gate.call_async.assert_not_called()

    def test_preparation_or_joint_action_still_stops_on_target_rejection(self):
        for changes in (dict(guard_state='syncing'), dict(joint_busy=True),
                        dict(plan_ready=True), dict(control_mode='point')):
            with self.subTest(changes=changes):
                state, _, _ = self.state(**changes)
                Backend.reject(state, '目标不可用')
                self.assertFalse(state.active)
                self.assertFalse(state.gate.call_async.call_args.args[0].data)
                state.hold_gate.call_async.assert_not_called()

    def test_holding_status_preserves_invalid_reason_and_never_says_arrived(self):
        state, _, _ = self.state()
        Backend.reject(state, '目标超出可达范围')
        Backend.publish_status(state)
        pending_status = json.loads(state.status.publish.call_args.args[0].data)
        self.assertIn('目标不可用', pending_status['state'])
        self.holding(state)
        self.assertTrue(state.active)
        self.assertIn('目标不可用', state.reason)
        self.assertNotIn('已到位', state.reason)
        Backend.publish_status(state)
        status = json.loads(state.status.publish.call_args.args[0].data)
        self.assertIn('目标不可用', status['state'])
        self.assertNotIn('已到位', status['state'])

    def test_only_new_verified_target_can_resume_and_hold_ack_cannot_reopen(self):
        state, q, _ = self.state()
        Backend.reject(state, '目标不可达或碰撞')
        hold = state.hold_future
        self.holding(state)
        Backend.tick(state)
        Backend.validation_tick(state)
        state.ik.call_async.assert_not_called()
        state.gate.call_async.assert_not_called()
        target = state.target.copy()
        target[1] += .01
        Backend.update_target(state, pose_message(target))
        Backend.tick(state)
        state.gate.call_async.assert_not_called()  # Editing alone has no authority.
        Backend.validation_tick(state)
        pending = state.pending
        state.model.pose.return_value = target
        pending.set_result(NS(error_code=NS(val=1, SUCCESS=1),
            solution=NS(joint_state=NS(name=JOINTS, position=q))))
        Backend.ik_done(state, pending, state.revision)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.approved_target, target)
        hold.set_result(NS(success=True, message='late hold ack'))
        state.gate.call_async.assert_not_called()
        Backend.tick(state)
        state.gate.call_async.assert_called_once()
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))

    def test_target_validated_before_hold_status_can_resume_after_the_status(self):
        state, _, _ = self.state()
        Backend.reject(state, '旧目标不可达')
        rejected_revision = state.revision
        state.revision += 1
        state.valid_revision, state.valid = state.revision, True
        state.approved_revision = state.revision
        state.approved_target = state.target.copy()
        self.holding(state)
        self.assertEqual(state.pause_revision, rejected_revision)
        self.assertEqual(state.reason, '等待恢复跟踪')
        Backend.tick(state)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)

    def test_hold_failure_or_unavailability_stops_and_closes_gate(self):
        for failure in ('response', 'exception', 'unavailable'):
            with self.subTest(failure=failure):
                state, _, _ = self.state()
                if failure == 'unavailable':
                    state.hold_gate.service_is_ready.return_value = False
                Backend.reject(state, '目标不可达')
                if failure == 'response':
                    state.hold_future.set_result(NS(success=False, message='denied'))
                elif failure == 'exception':
                    state.hold_future.set_exception(RuntimeError('transport failed'))
                self.assertFalse(state.active)
                self.assertIsNone(state.approved_target)
                self.assertFalse(state.gate.call_async.call_args.args[0].data)

    def test_pending_hold_times_out_even_without_an_approved_target(self):
        state, _, _ = self.state()
        Backend.reject(state, '目标不可达')
        state.hold_future.set_result(NS(success=True, message='ack without guard state'))
        self.now.return_value = 101.01
        state.panel_seen = 101.01
        Backend.tick(state)
        self.assertFalse(state.active)
        self.assertIn('保持响应超时', state.reason)
        self.assertFalse(state.gate.call_async.call_args.args[0].data)

    def test_stop_invalidates_late_hold_replies_without_changing_reason_or_opening(self):
        for success in (False, True):
            with self.subTest(success=success):
                state, _, _ = self.state()
                Backend.reject(state, '目标不可达')
                pending, epoch = state.hold_future, state.joint_epoch
                Backend.stop_service(state, None, NS())
                reason = state.reason
                self.assertGreater(state.joint_epoch, epoch)
                stopped_epoch = state.joint_epoch
                pending.set_result(NS(success=success, message='late'))
                self.assertEqual(state.joint_epoch, stopped_epoch)
                self.assertEqual(state.reason, reason)
                self.assertFalse(state.active)
                self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_native_pause_supersedes_a_concurrent_hold_without_reopening_or_disarming(self):
        state, _, _ = self.state()
        Backend.reject(state, '目标不可达')
        pending = state.hold_future
        Backend.on_guard(state, NS(data=json.dumps(dict(state='paused', reason='原生限位暂停'))))
        reason = state.reason
        pending.set_result(NS(success=False, message='当前已暂停，无法再次保持'))
        self.assertTrue(state.active)
        self.assertIsNone(state.approved_target)
        self.assertIsNone(state.hold_future)
        self.assertEqual(state.reason, reason)
        state.gate.call_async.assert_not_called()
        Backend.tick(state)
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))

    def test_real_fault_and_panel_loss_still_end_session(self):
        state, _, _ = self.state()
        Backend.reject(state, '目标不可达')
        Backend.on_guard(state, NS(data=json.dumps(dict(state='fault', reason='反馈超时'))))
        self.assertFalse(state.active)
        self.assertEqual(state.reason, '反馈超时')
        state, _, _ = self.state()
        Backend.reject(state, '目标不可达')
        self.holding(state)
        self.now.return_value = 100.61
        Backend.tick(state)
        self.assertFalse(state.active)
        self.assertIn('面板连接中断', state.reason)


if __name__ == '__main__':
    unittest.main()
