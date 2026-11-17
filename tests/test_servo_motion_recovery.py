"""Native halt and measured-stall recovery via official joint plans; fake I/O."""
from concurrent.futures import Future
import json
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from scripts.task.servo_tracking.node import Backend, pose_message
    from tests import test_servo_ik_seeds as seeds
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; no ROS nodes or hardware')
class ServoMotionRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)

    def state(self, **changes):
        state, raw, goal = seeds.ServoIKSeedTests.state(self, active=True, guard_state='running')
        state.valid, state.valid_revision = True, state.revision
        state.approved_target = state.target.copy()
        state.preview_q = goal.copy()
        state.last_target_change = 99.
        state.motion_recovery_required = state.recovery_hold = False
        state.recovery_attempt_revision = -1
        state.recovery_since = 0.
        state.joint_plan, state.joint_command = Mock(), Mock()
        for client in (state.joint_plan, state.joint_command):
            client.service_is_ready.return_value = True
            client.call_async.side_effect = lambda request: Future()
        for name in ('queue_joint_plan', 'joint_request_tick', 'joint_plan_done'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        Backend.clear_stall_window(state)
        for name, value in changes.items():
            setattr(state, name, value)
        return state, raw, goal

    def status(self, state, phase, **values):
        data = dict(state=phase, reason=phase, can_plan=phase in ('paused', 'holding', 'joint_done'))
        data.update(values)
        Backend.on_guard(state, NS(data=json.dumps(data)))

    def finish_candidates(self, state):
        replies = 0
        while state.pending is not None:
            seeds.ServoIKSeedTests.reply(self, state)
            replies += 1
            self.assertLessEqual(replies, 6)

    def test_native_pauses_keep_goal_and_drag_but_revalidate_same_revision(self):
        for cause in ('joint_bound', 'singularity'):
            with self.subTest(cause=cause):
                state, raw, _ = self.state(dragging=True)
                target, revision = state.target.copy(), state.revision
                self.status(state, 'paused', pause_cause=cause)
                self.assertTrue(state.active and state.dragging and state.motion_recovery_required)
                self.assertIsNone(state.approved_target)
                self.assertFalse(state.valid)
                self.assertEqual(state.revision, revision)
                np.testing.assert_array_equal(state.target, target)
                Backend.validation_tick(state)
                self.assertIsNotNone(state.pending)
                # Seed continuity starts at fresh feedback; ready is one of
                # several bounded numerical guesses, never a κ motion gate.
                np.testing.assert_array_equal(seeds.ServoIKSeedTests.seed(self, state), raw)
                self.assertTrue(any(np.allclose(seed, state.config['ready_joints'])
                                    for seed in state.ik_seeds))
                self.now.return_value = 100.2
                Backend.recovery_tick(state, *state.actual())
                state.joint_plan.call_async.assert_not_called()  # Never plan during drag.

    def test_editing_executing_recovery_retires_old_action_and_keeps_latest_session(self):
        state, raw, goal = self.state(joint_busy=True, joint_purpose='limit_recovery',
                                      joint_id='old-plan', dragging=True)
        epoch, revision = state.joint_epoch, state.revision
        target = state.target.copy(); target[1] += .005
        Backend.update_target(state, pose_message(target))
        self.assertTrue(state.active and state.motion_recovery_required)
        self.assertFalse(state.joint_busy)
        self.assertGreater(state.joint_epoch, epoch)
        self.assertEqual(state.revision, revision + 1)
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        self.assertIsNone(state.approved_target)
        self.now.return_value = 100.3
        state.dragging = False
        state.valid, state.valid_revision, state.preview_q = True, state.revision, goal
        state.guard_state, state.guard_can_plan = 'joint_executing', False
        Backend.recovery_tick(state, raw, state.model.pose(raw))
        state.joint_plan.call_async.assert_not_called()
        state.guard_state, state.guard_can_plan = 'idle', True
        Backend.recovery_tick(state, raw, state.model.pose(raw))
        self.assertTrue(state.active and state.joint_busy)
        self.assertEqual(state.joint_revision, state.revision)
        self.assertEqual(state.joint_purpose, 'limit_recovery')

    def test_recovery_compares_legal_branch_margins_before_authorizing_goal(self):
        state, _, _ = self.state()
        near = np.array([.1, 1.4, -1.2, 2.1, 0., -1.8])
        roomy = np.array([.1, 1.4, -1.2, .3, 0., 0.])
        state.target = state.model.pose(roomy)
        np.testing.assert_allclose(state.model.pose(near), state.target, atol=1e-7)
        self.status(state, 'paused', pause_cause='joint_bound')
        Backend.validation_tick(state)
        seeds.ServoIKSeedTests.reply(self, state, near)
        self.assertFalse(state.valid)
        self.assertIsNone(state.approved_target)
        seeds.ServoIKSeedTests.reply(self, state, roomy)
        self.finish_candidates(state)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.preview_q, roomy)
        state.gate.call_async.assert_not_called()

    def test_recovery_keeps_a_verified_branch_when_other_seed_has_no_solution(self):
        state, _, goal = self.state()
        self.status(state, 'paused', pause_cause='singularity')
        Backend.validation_tick(state)
        seeds.ServoIKSeedTests.reply(self, state, goal)
        self.assertFalse(state.valid)
        seeds.ServoIKSeedTests.reply(self, state)
        self.finish_candidates(state)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.preview_q, goal)

    def test_release_queues_one_auto_official_plan_then_completion_stays_closed(self):
        state, q, goal = self.state()
        target, revision = state.target.copy(), state.revision
        self.status(state, 'paused', pause_cause='joint_bound')
        Backend.accept_ik(state, goal, revision)
        self.now.return_value = 100.16
        Backend.recovery_tick(state, q, state.model.pose(q))
        self.assertTrue(state.active and state.joint_busy)
        self.assertEqual(state.joint_purpose, 'limit_recovery')
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        state.guard_can_plan = True  # Official stop/hold acknowledgement.
        Backend.joint_request_tick(state)
        request = state.joint_plan.call_async.call_args.args[0]
        self.assertEqual((request.purpose, request.target_revision), ('limit_recovery', revision))
        self.assertTrue(request.auto_execute)
        np.testing.assert_array_equal(request.joints, goal)
        state.joint_id, state.guard_state = 'owned', 'joint_executing'
        state.actual.return_value = (goal.copy(), state.model.pose(goal))
        self.status(state, 'joint_done', plan_id='owned')
        self.assertTrue(state.active and state.recovery_hold)
        self.assertFalse(state.motion_recovery_required or state.joint_busy)
        self.assertIsNone(state.approved_target)
        self.assertEqual(state.reason, '已到位 · 保持目标')
        self.assertEqual(state.revision, revision)
        np.testing.assert_array_equal(state.target, target)
        Backend.tick(state)
        self.assertEqual(state.joint_plan.call_async.call_count, 1)
        self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))
        np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))

    def test_same_revision_cannot_repeat_recovery_but_new_drag_can_queue_latest_goal(self):
        state, q, goal = self.state(motion_recovery_required=True, guard_state='paused',
            guard_can_plan=True, recovery_attempt_revision=8, recovery_since=99.)
        Backend.recovery_tick(state, q, state.model.pose(q))
        self.assertFalse(state.joint_busy)
        next_goal = goal.copy()
        next_goal[0] += .02
        Backend.update_target(state, pose_message(state.model.pose(next_goal)))
        Backend.accept_ik(state, next_goal, state.revision)
        self.now.return_value = 100.16
        Backend.recovery_tick(state, q, state.model.pose(q))
        self.assertTrue(state.active and state.joint_busy)
        self.assertEqual(state.recovery_attempt_revision, 9)
        np.testing.assert_array_equal(state.joint_wait[0], next_goal)

    def test_after_recovery_new_goal_resumes_without_condition_number_gate(self):
        for singular in (False, True):
            with self.subTest(singular=singular):
                state, _, goal = self.state(recovery_hold=True, guard_state='joint_done', pause_revision=7)
                current = np.array(state.config['ready_joints'])
                if singular:
                    current[4] = 0.
                actual = state.model.pose(current)
                state.model.condition_number = Mock(side_effect=AssertionError('No κ resume gate'))
                Backend.maybe_resume(state, current, actual)
                self.assertFalse(state.motion_recovery_required)
                self.assertTrue(state.gate.call_async.call_args.args[0].data)
                state.model.condition_number.assert_not_called()

    def test_warning_stall_requires_full_observation_window_then_holds_before_planning(self):
        state, _, _ = self.state()
        q = np.array([0., 1.4, -1.2, .3, .4, 0.])
        self.assertGreater(state.model.condition_number(q), state.config['lower_singularity_threshold'])
        actual = state.model.pose(q)
        Backend.recovery_tick(state, q, actual)
        self.now.return_value = 101.49
        Backend.recovery_tick(state, q, actual)
        state.hold_gate.call_async.assert_not_called()
        self.now.return_value = 101.51
        Backend.recovery_tick(state, q, actual)
        self.assertTrue(state.motion_recovery_required and state.active)
        self.assertFalse(state.valid)
        self.assertIsNone(state.approved_target)
        state.hold_gate.call_async.assert_called_once()
        state.joint_plan.call_async.assert_not_called()
        state.gate.call_async.assert_not_called()

    def test_drag_new_revision_and_measured_progress_prevent_stall_recovery(self):
        for progress in ('drag', 'new_revision', 'joint_motion', 'pose_progress'):
            with self.subTest(progress=progress):
                self.now.return_value = 100.
                state, _, _ = self.state()
                q = np.array([0., 1.4, -1.2, .3, .4, 0.])
                actual = state.model.pose(q)
                Backend.recovery_tick(state, q, actual)
                self.now.return_value = 101.6
                if progress == 'drag':
                    state.dragging = True
                elif progress == 'new_revision':
                    state.revision += 1
                    state.last_target_change = self.now.return_value
                elif progress == 'joint_motion':
                    q = q.copy()
                    q[0] += .004
                else:
                    actual = actual.copy()
                    actual[:3] += (state.target[:3]-actual[:3])*.1
                    actual[3:] = state.target[3:]
                Backend.recovery_tick(state, q, actual)
                self.assertFalse(state.motion_recovery_required)
                state.hold_gate.call_async.assert_not_called()

    def test_regular_pose_stall_also_recovers_without_condition_number_check(self):
        state, _, _ = self.state()
        q = np.array(state.config['ready_joints'])
        actual = state.model.pose(q)
        state.model.condition_number = Mock(side_effect=AssertionError('No κ stall gate'))
        Backend.recovery_tick(state, q, actual)
        self.now.return_value = 101.51
        Backend.recovery_tick(state, q, actual)
        self.assertTrue(state.motion_recovery_required)
        state.hold_gate.call_async.assert_called_once()
        state.model.condition_number.assert_not_called()

    def test_resume_authoritative_running_retires_old_reply_and_stop_invalidates_new_reply(self):
        state, q, _ = self.state(guard_state='holding', pause_revision=7)
        Backend.maybe_resume(state, q, state.model.pose(q))
        old = state.resume_future
        self.status(state, 'running')
        self.assertIsNone(state.resume_future)
        self.status(state, 'holding')
        state.pause_revision = 7
        self.now.return_value = 100.11
        Backend.maybe_resume(state, q, state.model.pose(q))
        current = state.resume_future
        reason = state.reason
        old.set_result(NS(success=False, message='old failure'))
        self.assertIs(state.resume_future, current)
        self.assertEqual(state.reason, reason)
        Backend.stop_service(state, None, NS())
        current.set_result(NS(success=True, message='late success'))
        self.assertFalse(state.active)
        self.assertEqual(state.reason, '已停止')

    def test_resume_request_has_a_bounded_timeout(self):
        state, q, _ = self.state(guard_state='holding', pause_revision=7)
        Backend.maybe_resume(state, q, state.model.pose(q))
        self.now.return_value = 101.01
        state.panel_seen = 101.01
        Backend.tick(state)
        self.assertFalse(state.active)
        self.assertIn('恢复响应超时', state.reason)
        self.assertFalse(state.gate.call_async.call_args.args[0].data)


if __name__ == '__main__':
    unittest.main()
