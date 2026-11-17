"""Bounded IK seed selection and revision-owned retry; fake services only."""
from pathlib import Path
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from scripts.task.servo_tracking.core import JOINTS, RobotModel
    from scripts.task.servo_tracking.node import Backend, pose_message
    from scripts.task.servo_tracking.ik_selection import make_ik_seeds
    from tests import test_servo_target_pause as target_pause
    ROS = True
except ImportError:
    ROS = False


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; all services are fake')
class ServoIKSeedTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)

    def state(self, raw=None, **changes):
        state, _, _ = target_pause.ServoTargetPauseTests.state(self)
        state.model = RobotModel(URDF.read_text(), tip='link6')
        raw = np.array([0., 1.4, -1.2, .3, .9, 0.] if raw is None else raw, dtype=float)
        goal = np.array([.1, 1.4, -1.2, .3, 1.05, 0.])
        state.actual = Mock(return_value=(raw, state.model.pose(raw)))
        state.target = state.model.pose(goal)
        state.active, state.valid, state.valid_revision = False, False, -1
        state.guard_state, state.approved_target = 'idle', None
        state.preview_q, state.ghost_revision = None, -1
        state.reason = '已停止'
        state.reject = Mock(wraps=state.reject)
        for name, value in changes.items():
            setattr(state, name, value)
        return state, raw, goal

    def seed(self, state, index=-1):
        request = state.ik.call_async.call_args_list[index].args[0].ik_request
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(request.group_name, 'arm')
        self.assertEqual(request.ik_link_name, 'tcp_link')
        self.assertEqual(request.robot_state.joint_state.name, JOINTS)
        self.assertEqual(request.timeout.sec, 0)
        self.assertEqual(request.timeout.nanosec, 30_000_000)
        return np.array(request.robot_state.joint_state.position)

    def reply(self, state, q=None, error=-31, future=None):
        future = state.pending if future is None else future
        result = NS(error_code=NS(val=error if q is None else 1, SUCCESS=1))
        if q is not None:
            result.solution = NS(joint_state=NS(name=JOINTS, position=list(q)))
        future.set_result(result)

    def finish_candidates(self, state, q=None, error=-31):
        """Finish the finite seed set; static goals compare all checked solutions."""
        replies = 0
        while state.pending is not None:
            self.reply(state, q, error=error)
            replies += 1
            self.assertLessEqual(replies, 6, 'A revision must have a bounded seed budget')
        return replies

    def test_real_zero_edge_uses_legal_reference_without_kappa_seed_switch_or_raw_change(self):
        state, raw, goal = self.state([0., -.0012, .0003, 0., 0., 0.])
        before = raw.copy()
        Backend.validation_tick(state)
        np.testing.assert_array_equal(self.seed(state), np.zeros(6))
        first = state.pending
        self.reply(state)
        self.assertIsNot(state.pending, first)
        np.testing.assert_array_equal(raw, before)
        np.testing.assert_array_equal(state.actual.return_value[0], before)
        state.reject.assert_not_called()
        self.finish_candidates(state, goal)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.preview_q, goal)
        self.assertEqual(state.ik_seeds, [])
        state.gate.call_async.assert_not_called()

    def test_static_reference_collects_finite_distinct_seeds_before_selecting(self):
        state, raw, goal = self.state()
        Backend.validation_tick(state)
        np.testing.assert_array_equal(self.seed(state), raw)
        budget = len(state.ik_seeds)
        self.assertGreaterEqual(budget, 3)
        self.assertLessEqual(budget, 6)
        expected = make_ik_seeds(raw, state.model.lower, state.model.upper,
                                 ready=state.config['ready_joints'], max_seeds=6)
        self.assertEqual(budget, len(expected))
        for seed, wanted in zip(state.ik_seeds, expected):
            np.testing.assert_array_equal(seed, wanted)
        self.reply(state)
        np.testing.assert_array_equal(self.seed(state), state.config['ready_joints'])
        self.finish_candidates(state, goal)
        Backend.validation_tick(state)
        self.assertEqual(state.ik.call_async.call_count, budget)
        self.assertTrue(state.valid)
        state.reject.assert_not_called()

    def test_dragging_success_on_first_seed_keeps_fast_path(self):
        state, _, goal = self.state(active=True, dragging=True, guard_state='running')
        Backend.validation_tick(state)
        self.reply(state, goal)
        Backend.validation_tick(state)
        self.assertEqual(state.ik.call_async.call_count, 1)
        self.assertTrue(state.valid)

    def test_exhausted_failures_reject_once_and_do_not_restart_same_revision(self):
        state, _, _ = self.state(active=True, guard_state='running')
        Backend.validation_tick(state)
        self.reply(state, error=-12)
        state.reject.assert_not_called()
        state.hold_gate.call_async.assert_not_called()
        self.finish_candidates(state)
        state.reject.assert_called_once_with('目标不可达或碰撞')
        self.assertTrue(state.active)
        self.assertIsNone(state.approved_target)
        state.hold_gate.call_async.assert_called_once()
        Backend.validation_tick(state)
        self.assertGreaterEqual(state.ik.call_async.call_count, 3)
        self.assertLessEqual(state.ik.call_async.call_count, 6)
        self.assertFalse(state.valid)
        self.assertEqual(state.valid_revision, state.revision)

    def test_each_returned_branch_must_pass_strict_bounds_margin_and_fk(self):
        for q in ([0., -.002, 0., 0., 0., 0.], [0.] * 6,
                  [0., 1.4, -1.2, .3, .9, 0.]):
            with self.subTest(q=q):
                state, _, goal = self.state()
                Backend.validation_tick(state)
                self.reply(state, q)
                self.assertFalse(state.valid)
                self.assertIsNone(state.approved_target)
                self.assertEqual(state.ik.call_async.call_count, 2)
                self.finish_candidates(state, goal)
                self.assertTrue(state.valid)
                state.reject.assert_not_called()

    def test_optional_ready_is_only_a_bounded_seed_not_kappa_motion_permission(self):
        for ready in ([float('nan')]*6, [0.]*6, [0., 1.4, -1.2, .3, .02, 0.]):
            with self.subTest(ready=ready):
                state, raw, _ = self.state()
                state.config['ready_joints'] = ready
                Backend.validation_tick(state)
                np.testing.assert_array_equal(self.seed(state), raw)
                for seed in state.ik_seeds:
                    state.model.check_joints(seed)
                self.finish_candidates(state)
                state.reject.assert_called_once()
                state.gate.call_async.assert_not_called()

    def test_duplicate_actual_and_ready_seed_is_only_attempted_once(self):
        state, _, _ = self.state([0., 1.4, -1.2, .3, 1., 0.])
        Backend.validation_tick(state)
        seeds = [seed.copy() for seed in state.ik_seeds]
        self.assertGreaterEqual(len(seeds), 2)  # Middle/wrist guesses remain.
        self.assertTrue(all(not np.allclose(seed, other, rtol=0, atol=1e-10)
                            for index, seed in enumerate(seeds) for other in seeds[index+1:]))
        self.finish_candidates(state)
        self.assertEqual(state.ik.call_async.call_count, len(seeds))
        state.reject.assert_called_once()

    def test_sim_and_excessive_real_feedback_deviations_cannot_be_normalized(self):
        for raw, mode in (([0., -.001, .0003, 0., 0., 0.], 'sim'),
                          ([0., -.002, 0., 0., 0., 0.], 'real'),
                          ([3., 0., 0., 0., 0., 0.], 'real')):
            with self.subTest(raw=raw, mode=mode):
                state, measured, _ = self.state(raw, mode=mode)
                before = measured.copy()
                Backend.validation_tick(state)
                state.ik.call_async.assert_not_called()
                state.reject.assert_called_once()
                np.testing.assert_array_equal(measured, before)

    def test_old_revision_reply_cannot_validate_or_retry_after_new_drag(self):
        state, _, goal = self.state()
        Backend.validation_tick(state)
        old = state.pending
        next_goal = goal.copy()
        next_goal[0] += .05
        Backend.update_target(state, pose_message(state.model.pose(next_goal)))
        Backend.validation_tick(state)
        self.assertEqual(state.ik.call_async.call_count, 1)  # Only one pending request.
        self.reply(state, goal, future=old)
        self.assertFalse(state.valid)
        self.assertIsNone(state.approved_target)
        Backend.validation_tick(state)
        self.assertEqual(state.ik.call_async.call_count, 2)
        self.finish_candidates(state, next_goal)
        self.assertTrue(state.valid)

    def test_current_request_timeout_does_not_extend_budget_by_retrying(self):
        state, _, _ = self.state()
        Backend.validation_tick(state)
        pending = state.pending
        self.now.return_value = 100.121
        Backend.validation_tick(state)
        self.assertTrue(pending.cancelled())
        self.assertEqual(state.ik_seeds, [])
        self.assertEqual(state.ik.call_async.call_count, 1)
        state.reject.assert_called_once_with('逆解检查超时')

    def test_old_request_timeout_does_not_reject_new_revision_or_accept_late_result(self):
        state, _, goal = self.state()
        Backend.validation_tick(state)
        old = state.pending
        old.set_running_or_notify_cancel()
        next_goal = goal.copy()
        next_goal[0] += .05
        Backend.update_target(state, pose_message(state.model.pose(next_goal)))
        self.now.return_value = 100.121
        Backend.validation_tick(state)
        state.reject.assert_not_called()
        Backend.validation_tick(state)
        current = state.pending
        self.reply(state, goal, future=old)
        self.assertIs(state.pending, current)
        self.assertFalse(state.valid)
        self.finish_candidates(state, next_goal)
        self.assertTrue(state.valid)

    def test_static_multi_solution_scoring_chooses_continuous_checked_branch(self):
        goal = np.array([.1, 1.4, -1.2, 1.8, .6, 1.7])
        reference = goal.copy()
        reference[3] -= .01
        flipped = goal.copy()
        flipped[3] -= np.pi
        flipped[4] *= -1.
        flipped[5] -= np.pi
        state, _, _ = self.state(raw=reference)
        state.target = state.model.pose(goal)
        # URDF wrist-axis rotations are decimal approximations to pi/2, so the
        # analytic flip has a tiny residual; still far below the IK FK limits.
        np.testing.assert_allclose(state.model.pose(flipped), state.target, rtol=0, atol=3e-8)
        Backend.validation_tick(state)
        self.reply(state, flipped)  # First valid branch is far from actual joints.
        self.assertFalse(state.valid, 'Static preview must compare the remaining legal candidates')
        self.finish_candidates(state, goal)
        self.assertTrue(state.valid)
        np.testing.assert_allclose(state.preview_q, goal, atol=1e-10)
        state.gate.call_async.assert_not_called()

    def test_stop_or_native_pause_clears_seeds_and_ignores_uncancellable_old_result(self):
        for pause in (False, True):
            with self.subTest(pause=pause):
                state, _, goal = self.state(active=True, guard_state='running')
                Backend.validation_tick(state)
                old = state.pending
                old.set_running_or_notify_cancel()
                if pause:
                    Backend.on_guard(state, NS(data=json.dumps(dict(state='paused', reason='限位暂停'))))
                else:
                    Backend.stop_service(state, None, NS())
                self.assertEqual(state.ik_seeds, [])
                self.assertIsNone(state.pending)
                self.reply(state, goal, future=old)
                self.assertFalse(state.valid)
                self.assertIsNone(state.approved_target)
                self.assertEqual(state.ik.call_async.call_count, 1)
                self.assertFalse(any(call.args[0].data for call in state.gate.call_async.call_args_list))

    def test_legacy_direct_callback_without_seed_metadata_rejects_immediately(self):
        state, _, _ = self.state()
        Backend.validation_tick(state)
        state.ik_seed_revision = -1
        self.reply(state)
        state.reject.assert_called_once()
        self.assertEqual(state.ik.call_async.call_count, 1)


if __name__ == '__main__':
    unittest.main()
