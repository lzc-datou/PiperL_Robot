"""Backend settled-hold/resume races with fake services; never starts a ROS node."""
from concurrent.futures import Future
import json
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

try:
    from scripts.task.servo_tracking.node import Backend, pose_message
    from scripts.task.servo_tracking.core import load_config
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the built ROS runtime wrapper; all control calls are mocked')
class ServoHoldBackendTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)

    def state(self, **changes):
        config = load_config(Path(__file__).resolve().parents[1] /
                             'scripts/task/servo_tracking/config/config.yaml')
        q = np.array(config['ready_joints'])
        actual = np.array([.3, 0., .4, 0., 0., 0., 1.])
        target = actual.copy()
        target[0] += .002
        state = NS(config=config, actual=Mock(return_value=(q, actual)), model=Mock(),
                   controller=Mock(antiwindup_state='inactive'),
                   target=target.copy(), approved_target=target.copy(), revision=8,
                   valid_revision=8, approved_revision=8, valid=True, active=True, dragging=False,
                   guard_state='running', guard_reason='', guard_can_plan=False,
                   hold_future=None, hold_revision=-1, hold_requested_at=0.,
                   settle_since=0., settle_q=None, joint_epoch=5,
                   joint_busy=False, plan_ready=False, joint_id='', joint_purpose='',
                   joint_wait=None, joint_future=None, joint_revision=-1,
                   pending=None, start_pending=False, resume_future=None,
                   pause_revision=-1, last_resume_attempt=0., verification_gap_since=None,
                   control_mode='servo', panel_seen=100., drag_seen=100., switched=True,
                   heartbeat=Mock(), publish_twist=Mock(), align=Mock(), reason='跟踪中',
                   reachability='可达', scale=.5, hold_gate=Mock(), gate=Mock())
        state.model.condition_number.return_value = 10.
        state.model.radius = 1.
        for service in (state.hold_gate, state.gate):
            service.service_is_ready.return_value = True
            service.call_async.side_effect = lambda request: Future()
        for name in ('maybe_hold', 'hold_done', 'maybe_resume', 'resume_done', 'stop_tracking'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        for name, value in changes.items():
            setattr(state, name, value)
        return state, q, actual

    def request_hold(self, state, q, actual):
        self.now.return_value = 100.
        Backend.maybe_hold(state, q, actual)
        self.now.return_value = 100.31
        Backend.maybe_hold(state, q, actual)
        self.assertIsNotNone(state.hold_future)
        return state.hold_future

    def holding(self, state):
        Backend.on_guard(state, NS(data=json.dumps(dict(state='holding', reason='保持目标'))))

    def test_hold_requires_pose_tolerance_and_stable_joints_for_settle_time(self):
        state, q, actual = self.state()
        state.approved_target = np.r_[actual[:3] + [.0029, 0., 0.],
                                      Rotation.from_rotvec([0., .0199, 0.]).as_quat()]
        Backend.maybe_hold(state, q, actual)
        self.now.return_value = 100.29
        Backend.maybe_hold(state, q + .0029, actual)
        state.hold_gate.call_async.assert_not_called()
        self.now.return_value = 100.31
        Backend.maybe_hold(state, q + .0029, actual)
        state.hold_gate.call_async.assert_called_once()
        self.assertEqual(state.hold_revision, 8)
        self.assertEqual(state.hold_requested_at, 100.31)

    def test_position_or_rotation_outside_tolerance_cannot_request_hold(self):
        for position, rotation in ((.0031, 0.), (0., .0201)):
            with self.subTest(position=position, rotation=rotation):
                state, q, actual = self.state(settle_since=99., settle_q=np.zeros(6))
                state.approved_target = np.r_[actual[:3] + [position, 0., 0.],
                                              Rotation.from_rotvec([rotation, 0., 0.]).as_quat()]
                Backend.maybe_hold(state, q, actual)
                self.assertEqual(state.settle_since, 0.)
                self.assertIsNone(state.settle_q)
                state.hold_gate.call_async.assert_not_called()

    def test_joint_motion_restarts_settle_interval(self):
        state, q, actual = self.state()
        Backend.maybe_hold(state, q, actual)
        self.now.return_value = 100.29
        moved = q.copy()
        moved[0] += .0031
        Backend.maybe_hold(state, moved, actual)
        self.assertEqual(state.settle_since, 100.29)
        self.now.return_value = 100.31
        Backend.maybe_hold(state, moved, actual)
        state.hold_gate.call_async.assert_not_called()
        self.now.return_value = 100.60
        Backend.maybe_hold(state, moved, actual)
        state.hold_gate.call_async.assert_called_once()

    def test_dragging_unchecked_or_pending_hold_never_starts_another_hold(self):
        for changes in (dict(dragging=True), dict(valid=False), dict(valid_revision=7),
                        dict(approved_target=None), dict(hold_future=Future())):
            with self.subTest(changes=changes):
                state, q, actual = self.state(settle_since=99., settle_q=np.zeros(6), **changes)
                Backend.maybe_hold(state, q, actual)
                self.assertEqual(state.settle_since, 0.)
                self.assertIsNone(state.settle_q)
                state.hold_gate.call_async.assert_not_called()

    def test_entering_pose_tolerance_while_still_moving_does_not_latch_hold(self):
        state, q, actual = self.state(settle_since=99., settle_q=np.array([0., 1.4, -1.2, .3, 1., 0.]))
        state.controller.motion_settled = False
        Backend.maybe_hold(state, q, actual)
        self.assertEqual(state.settle_since, 0.)
        self.assertIsNone(state.settle_q)
        state.hold_gate.call_async.assert_not_called()
        state.controller.motion_settled = True
        Backend.maybe_hold(state, q, actual)
        state.hold_gate.call_async.assert_not_called()
        self.now.return_value = 100.31
        Backend.maybe_hold(state, q, actual)
        state.hold_gate.call_async.assert_called_once()

    def test_entering_hold_preserves_goal_and_locks_requested_revision(self):
        state, q, actual = self.state()
        target = state.target.copy()
        future = self.request_hold(state, q, actual)
        self.holding(state)
        self.assertTrue(state.active)
        self.assertEqual(state.pause_revision, 8)
        self.assertIsNone(state.hold_future)
        self.assertEqual(state.reason, '已到位 · 保持目标')
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target, target)
        # Service ACK may follow the state message; it cannot overwrite holding.
        future.set_result(NS(success=True, message='ack'))
        self.assertEqual(state.reason, '已到位 · 保持目标')

    def test_new_target_during_hold_request_is_not_locked_out_by_arriving_status(self):
        state, q, actual = self.state()
        self.request_hold(state, q, actual)
        target = state.target.copy()
        target[0] += .01
        Backend.update_target(state, pose_message(target))
        self.assertEqual(state.revision, 9)
        self.holding(state)
        self.assertEqual(state.pause_revision, 8)  # The revision settled was 8, not new target 9.
        np.testing.assert_array_equal(state.target, target)
        Backend.maybe_resume(state, q, actual)
        state.gate.call_async.assert_not_called()  # New target has not passed IK yet.
        state.valid = True
        state.valid_revision = 9
        state.approved_revision = 9
        state.approved_target = target.copy()
        Backend.maybe_resume(state, q, actual)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        state.align.assert_not_called()

    def test_feedback_noise_or_old_target_revision_cannot_resume_holding(self):
        state, q, actual = self.state(guard_state='holding', pause_revision=8)
        noisy = actual.copy()
        noisy[0] += .005
        Backend.maybe_resume(state, q + .001, noisy)
        state.gate.call_async.assert_not_called()
        state.model.condition_number.assert_not_called()
        for valid, valid_revision in ((False, 9), (True, 8)):
            state.revision = 9
            state.valid, state.valid_revision = valid, valid_revision
            Backend.maybe_resume(state, q, actual)
        state.gate.call_async.assert_not_called()

    def test_external_hold_without_own_request_locks_current_revision(self):
        state, q, actual = self.state(hold_revision=6, hold_future=None)
        self.holding(state)
        self.assertEqual(state.pause_revision, state.revision)
        self.assertEqual(state.pause_revision, 8)
        Backend.maybe_resume(state, q, actual)
        state.gate.call_async.assert_not_called()
        self.assertTrue(state.active)
        state.model.condition_number.assert_not_called()

    def test_only_new_verified_revision_requests_holding_resume_once(self):
        state, q, actual = self.state(guard_state='holding', pause_revision=7)
        Backend.maybe_resume(state, q, actual)
        Backend.maybe_resume(state, q, actual)
        state.gate.call_async.assert_called_once()
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        self.assertIsNotNone(state.resume_future)

    def test_stop_or_mode_switch_rejects_a_late_failed_hold_response(self):
        for mode_switch in (False, True):
            with self.subTest(mode_switch=mode_switch):
                state, q, actual = self.state()
                future = self.request_hold(state, q, actual)
                if mode_switch:
                    Backend.mode_service(state, NS(data=True), NS())
                else:
                    Backend.stop_service(state, None, NS())
                reason = state.reason
                epoch = state.joint_epoch
                future.set_result(NS(success=False, message='late failure'))
                self.assertEqual(state.reason, reason)
                self.assertEqual(state.joint_epoch, epoch)
                self.assertFalse(state.active)
                self.assertIsNone(state.hold_future)
                self.assertTrue(all(not call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_holding_ticks_publish_no_motion_and_panel_disconnect_stops(self):
        state, q, actual = self.state(guard_state='holding', pause_revision=8)
        Backend.tick(state)
        self.assertTrue(state.active)
        state.gate.call_async.assert_not_called()
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))
        self.now.return_value = 101.
        Backend.tick(state)
        self.assertFalse(state.active)
        self.assertIn('面板连接中断', state.reason)
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))


if __name__ == '__main__':
    unittest.main()
