"""Guard joint-planning lifecycle/ownership tests; all ROS services are fakes."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import threading
import time
import unittest
import numpy as np

try:
    from scripts.task.servo_tracking.joint_motion import JointMotion
    from scripts.task.servo_tracking.guard import Guard
    from scripts.task.servo_tracking.core import JOINTS
    from tests import test_servo_guard_recovery as recovery
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from unique_identifier_msgs.msg import UUID
    ROS = True
except ImportError:
    ROS = False


class FakeController:
    def __init__(self):
        self.calls = []
        self.ready = True

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, goal_uuid=None):
        future = Future()
        self.calls.append((goal, goal_uuid, future))
        return future


@unittest.skipUnless(ROS, 'Use the ROS runtime wrapper; never initializes ROS')
class ServoJointMotionTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)
        self.g = recovery.ServoGuardRecoveryTests.guard(self)
        self.g.config.update(planning_time=5., plan_settle_timeout=5., joint_margin=.015,
                             lower_singularity_threshold=17.)
        self.g.model.check_joints.side_effect = lambda q, margin=0.: np.asarray(q, dtype=float)
        self.g.model.planning_joints.side_effect = lambda q, zero_tolerance=0.: np.asarray(q, dtype=float).copy()
        self.planner, self.controller = recovery.FakeService(), FakeController()
        self.g.create_client, self.g.create_service = Mock(return_value=self.planner), Mock()
        with patch('scripts.task.servo_tracking.joint_motion.ActionClient', return_value=self.controller):
            self.motion = JointMotion(self.g)
        self.g.joint = self.motion
        self.addCleanup(self.motion.shutdown)
        self.validator = patch('scripts.task.servo_tracking.core.validate_joint_plan', return_value=2.).start()
        self.q = self.g.health.return_value.copy()
        self.goal = self.q + .02

    def request(self, purpose='point', auto_execute=False):
        return NS(joints=self.goal.tolist(), speed_scale=.5, purpose=purpose,
                  target_revision=4, auto_execute=auto_execute)

    def plan(self, purpose='point', auto_execute=False):
        response = self.motion.plan(self.request(purpose, auto_execute), NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual(response.plan_id, self.motion.plan_id)
        return response

    def planning_result(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINTS
        first = JointTrajectoryPoint(positions=self.q.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6)
        last = JointTrajectoryPoint(positions=self.goal.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6)
        last.time_from_start.sec = 2
        trajectory.points = [first, last]
        return NS(motion_plan_response=NS(error_code=NS(val=1), trajectory=NS(
            joint_trajectory=trajectory, multi_dof_joint_trajectory=NS(points=[]))))

    def ready(self, purpose='point', auto_execute=False):
        self.plan(purpose, auto_execute)
        self.g.pause_servo.complete()
        self.assertEqual(self.g.state, 'joint_planning')
        self.planner.calls[-1][1].set_result(self.planning_result())
        validation = self.motion.pending_validation
        validation.exception(timeout=1.)
        self.g.tick()

    def execute(self):
        response = self.motion.command(NS(plan_id=self.motion.plan_id, command='execute'), NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.g.state, 'joint_syncing')
        self.now.return_value = 100.01
        self.g.tick()
        self.now.return_value = 100.32
        self.g.heartbeat = 100.32
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_opening')
        self.g.gate.complete()
        self.assertEqual(self.g.state, 'joint_executing')

    def accepted(self):
        handle = NS(accepted=True, cancel_goal_async=Mock(), get_result_async=Mock(return_value=Future()))
        self.controller.calls[-1][2].set_result(handle)
        return handle

    def test_point_plan_is_display_only_until_explicit_execute(self):
        self.ready()
        self.assertEqual(self.g.state, 'joint_ready')
        self.assertTrue(self.motion.status()['plan_ready'])
        self.assertFalse(self.motion.status()['joint_busy'])
        self.assertEqual(self.g.gate.calls, [])
        self.assertEqual(self.controller.calls, [])
        request = self.planner.calls[-1][0].motion_plan_request
        self.assertEqual(request.pipeline_id, 'ompl')
        self.assertEqual(list(request.start_state.joint_state.position), self.q.tolist())
        self.assertAlmostEqual(request.max_velocity_scaling_factor, .4)
        self.assertAlmostEqual(request.max_acceleration_scaling_factor, .125)

    def test_ready_plan_autoexecutes_but_is_not_a_servo_running_state(self):
        self.ready('ready', True)
        self.assertEqual(self.g.state, 'joint_syncing')
        self.assertTrue(self.motion.status()['joint_busy'])
        self.assertEqual(self.controller.calls, [])

    def test_constraint_pause_can_plan_only_after_gate_and_hold_are_confirmed(self):
        self.g.state = 'paused'
        self.g.closed = False
        self.assertFalse(self.motion.can_plan())
        self.g.closed, self.g.hold_pending = True, True
        self.assertFalse(self.motion.can_plan())
        self.g.hold_pending = False
        self.assertTrue(self.motion.can_plan())

    def test_limit_recovery_uses_same_validated_auto_joint_execution(self):
        self.g.state = 'paused'
        self.g.model.condition_number.return_value = 45.
        # Conditioning does not invalidate a bounded joint-space endpoint.
        self.ready('limit_recovery', True)
        self.assertEqual(self.g.state, 'joint_syncing')
        self.assertEqual(self.motion.purpose, 'limit_recovery')
        self.assertEqual(self.motion.target_revision, 4)
        self.validator.assert_called_once()
        self.assertEqual(self.controller.calls, [])

    def test_recovery_cannot_be_submitted_with_manual_execution_semantics(self):
        result = self.motion.plan(self.request('limit_recovery', False), NS())
        self.assertFalse(result.success)
        self.assertEqual(self.planner.calls, [])
        self.assertEqual(self.controller.calls, [])

    def test_home_is_automatic_zero_joint_plan_with_hard_bounds_only(self):
        self.goal = np.zeros(6)
        self.g.model.condition_number.return_value = float('inf')
        self.ready('home', True)
        self.assertEqual(self.g.state, 'joint_syncing')
        self.assertEqual(self.motion.purpose, 'home')
        self.g.model.check_joints.assert_any_call(self.goal.tolist(), 0.)
        np.testing.assert_array_equal(self.motion.goal_q, np.zeros(6))
        self.assertEqual(self.validator.call_args.args[4]['joint_margin'], 0.)
        self.assertEqual(self.g.config['joint_margin'], .015)
        self.assertEqual(self.controller.calls, [])

    def test_home_rejects_a_nonzero_or_nonautomatic_target(self):
        result = self.motion.plan(self.request('home', True), NS())
        self.assertFalse(result.success)
        self.assertIn('回零目标', result.message)
        self.goal = np.zeros(6)
        result = self.motion.plan(self.request('home', False), NS())
        self.assertFalse(result.success)
        self.assertEqual(self.planner.calls, [])
        self.assertEqual(self.controller.calls, [])

    def test_explicit_uuid_exempts_only_the_owned_controller_goal(self):
        self.ready()
        self.execute()
        message, goal_id, _ = self.controller.calls[-1]
        self.assertEqual(bytes(goal_id.uuid), self.motion.owned_uuid)
        self.assertEqual(list(message.trajectory.points[-1].positions), self.goal.tolist())
        self.g.on_action_status(NS(status_list=[NS(status=2, goal_info=NS(goal_id=goal_id))]))
        self.assertFalse(self.g.action_active)
        self.assertEqual(self.g.state, 'joint_executing')
        self.g.on_action_status(NS(status_list=[NS(status=2, goal_info=NS(goal_id=UUID(uuid=[9] * 16)))]))
        self.assertTrue(self.g.action_active)
        self.assertEqual(self.g.state, 'fault')

    def test_stale_servo_singularity_status_does_not_cancel_joint_action(self):
        self.ready()
        self.execute()
        self.g.on_servo(NS(code=2, message='old Cartesian singularity'))
        self.g.on_servo(NS(code=-1, message='paused Cartesian status'))
        self.assertEqual(self.g.state, 'joint_executing')

    def test_stop_invalidates_plan_and_cancels_late_accepted_action(self):
        self.ready()
        self.execute()
        pending = self.controller.calls[-1][2]
        self.g.stop('stop')
        handle = NS(accepted=True, cancel_goal_async=Mock(), get_result_async=Mock(return_value=Future()))
        pending.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.g.state, 'idle')
        self.assertEqual(self.motion.plan_id, '')
        self.assertIsNone(self.motion.owned_uuid)

    def test_stopped_pending_send_blocks_new_control_until_terminal_result(self):
        self.ready()
        self.execute()
        pending = self.controller.calls[-1][2]
        self.g.stop('stop')
        self.g.gate.complete()
        self.g.hold.complete()
        self.g.tick()
        self.assertFalse(self.motion.can_plan())
        self.assertFalse(self.g.enable(NS(data=True), NS()).success)
        handle = NS(accepted=True, cancel_goal_async=Mock(), get_result_async=Mock(return_value=Future()))
        pending.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.motion.can_plan())  # cancel request does not prove termination
        handle.get_result_async.return_value.set_result(NS(status=5))
        self.assertTrue(self.motion.can_plan())

    def test_terminal_status_also_releases_the_stopped_action_barrier(self):
        self.ready()
        self.execute()
        self.accepted()
        action_id = self.controller.calls[-1][1]
        self.g.stop('stop')
        self.assertTrue(self.motion.has_unresolved_actions())
        self.g.on_action_status(NS(status_list=[NS(status=5, goal_info=NS(goal_id=action_id))]))
        self.assertFalse(self.motion.has_unresolved_actions())

    def test_stopped_owned_action_is_not_another_task_while_canceling(self):
        self.ready()
        self.execute()
        self.accepted()
        action_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        for code in (2, 3):
            self.g.on_action_status(NS(status_list=[NS(status=code, goal_info=NS(goal_id=action_id))]))
            self.assertFalse(self.g.action_active)
        self.assertTrue(self.motion.has_unresolved_actions())
        self.assertFalse(self.motion.can_plan())
        self.assertFalse(self.g.enable(NS(data=True), NS()).success)
        self.assertEqual(self.g.cancel_action.calls, [])

    def test_terminal_result_without_another_status_unblocks_servo_restart(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        action_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        self.g.on_action_status(NS(status_list=[NS(status=3, goal_info=NS(goal_id=action_id))]))
        handle.get_result_async.return_value.set_result(NS(status=5))
        self.g.gate.complete()
        self.g.hold.complete()
        self.g.tick()
        # The result and status transports can arrive in either order. No new
        # status publication is needed to forget an already terminal own goal.
        self.assertFalse(self.g.action_active)
        self.assertTrue(self.motion.can_plan())
        response = self.g.enable(NS(data=True), NS())
        self.assertTrue(response.success, response.message)

    def test_terminal_owned_goal_stays_identifiable_after_plan_invalidation(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        action_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        handle.get_result_async.return_value.set_result(NS(status=5))
        self.assertFalse(self.motion.has_unresolved_actions())
        self.g.on_action_status(NS(status_list=[NS(status=2, goal_info=NS(goal_id=action_id))]))
        self.assertFalse(self.g.action_active)
        self.assertTrue(self.motion.owns_action(action_id))
        self.assertEqual(self.g.cancel_action.calls, [])

    def test_old_owned_status_does_not_cancel_a_new_owned_action(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        old_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        handle.get_result_async.return_value.set_result(NS(status=5))
        self.g.gate.complete()
        self.g.hold.complete()
        self.g.tick()
        self.ready()
        self.execute()
        new_id = self.controller.calls[-1][1]
        self.g.on_action_status(NS(status_list=[
            NS(status=3, goal_info=NS(goal_id=old_id)),
            NS(status=2, goal_info=NS(goal_id=new_id))]))
        self.assertFalse(self.g.action_active)
        self.assertEqual(self.g.state, 'joint_executing')
        self.assertEqual(self.g.cancel_action.calls, [])

    def test_terminal_status_before_late_acceptance_does_not_restore_barrier(self):
        self.ready()
        self.execute()
        pending = self.controller.calls[-1][2]
        action_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        self.g.on_action_status(NS(status_list=[NS(status=5, goal_info=NS(goal_id=action_id))]))
        self.assertFalse(self.motion.has_unresolved_actions())
        handle = NS(accepted=True, cancel_goal_async=Mock(), get_result_async=Mock(return_value=Future()))
        pending.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.motion.has_unresolved_actions())
        self.assertTrue(self.motion.owns_action(action_id))
        self.assertEqual(self.g.state, 'idle')

    def test_retired_ownership_does_not_exempt_an_unknown_active_goal(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        old_id = self.controller.calls[-1][1]
        self.g.stop('target changed')
        handle.get_result_async.return_value.set_result(NS(status=5))
        self.g.on_action_status(NS(status_list=[
            NS(status=5, goal_info=NS(goal_id=old_id)),
            NS(status=2, goal_info=NS(goal_id=UUID(uuid=[9] * 16)))]))
        self.assertTrue(self.g.action_active)
        self.assertEqual(len(self.g.cancel_action.calls), 1)
        response = self.g.enable(NS(data=True), NS())
        self.assertFalse(response.success)
        self.assertEqual(response.message, '等待其他轨迹任务取消')

    def test_rejected_old_goal_releases_barrier_without_any_execution(self):
        self.ready()
        self.execute()
        pending = self.controller.calls[-1][2]
        self.g.stop('stop')
        pending.set_result(NS(accepted=False))
        self.assertFalse(self.motion.has_unresolved_actions())

    def test_stop_cancels_an_accepted_action(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        self.g.stop('stop')
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.motion.plan_id, '')

    def test_late_planning_result_cannot_rearm_after_stop(self):
        self.plan()
        self.g.pause_servo.complete()
        pending = self.planner.calls[-1][1]
        # Simulate a response already in flight, which cannot be canceled.
        pending.set_running_or_notify_cancel()
        self.g.stop('stop')
        pending.set_result(self.planning_result())
        self.assertEqual(self.g.state, 'idle')
        self.assertFalse(self.motion.ready)
        self.assertEqual(self.controller.calls, [])

    def test_late_gate_open_response_recloses_without_dispatch(self):
        self.ready()
        self.motion.command(NS(plan_id=self.motion.plan_id, command='execute'), NS())
        self.now.return_value = 100.01
        self.g.tick()
        self.now.return_value = 100.32
        self.g.heartbeat = 100.32
        self.g.tick()
        pending = self.g.gate.calls[-1][1]
        self.g.stop('stop')
        pending.set_result(NS(success=True))
        self.assertEqual(self.controller.calls, [])
        self.assertFalse(self.g.gate.calls[-1][0].data)

    def test_feedback_drift_invalidates_plan_before_execute(self):
        self.ready()
        self.g.health.return_value = self.q + .006
        result = self.motion.command(NS(plan_id=self.motion.plan_id, command='execute'), NS())
        self.assertFalse(result.success)
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.controller.calls, [])

    def test_idle_plan_expires_before_execute_is_clicked(self):
        self.ready()
        self.now.return_value = 131.
        self.g.heartbeat = 131.
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertFalse(self.motion.ready)
        self.assertEqual(self.motion.plan_id, '')
        self.assertEqual(self.controller.calls, [])

    def test_invalid_polynomial_plan_never_opens_gate(self):
        self.validator.side_effect = ValueError('quintic speed exceeded')
        self.ready()
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.g.gate.calls[-1][0].data, False)
        self.assertEqual(self.controller.calls, [])

    def finish_validation(self):
        self.motion.pending_validation.exception(timeout=1.)
        self.g.tick()

    def test_limit_only_failure_requests_slower_official_plans_with_unchanged_caps(self):
        self.validator.side_effect = [ValueError('关节规划多项式加速度越限'),
                                      ValueError('关节规划多项式速度越限'), 2.]
        self.ready()
        plan_id = self.motion.plan_id
        self.assertEqual(self.g.state, 'joint_planning')
        self.assertEqual(self.motion.replans, 1)
        motion = self.planner.calls[-1][0].motion_plan_request
        self.assertAlmostEqual(motion.max_velocity_scaling_factor, .24)
        self.assertAlmostEqual(motion.max_acceleration_scaling_factor, .075)
        self.planner.calls[-1][1].set_result(self.planning_result())
        self.finish_validation()
        self.assertEqual(self.motion.replans, 2)
        motion = self.planner.calls[-1][0].motion_plan_request
        self.assertAlmostEqual(motion.max_velocity_scaling_factor, .144)
        self.assertAlmostEqual(motion.max_acceleration_scaling_factor, .045)
        self.planner.calls[-1][1].set_result(self.planning_result())
        self.finish_validation()
        self.assertEqual(self.g.state, 'joint_ready')
        self.assertEqual(self.motion.plan_id, plan_id)
        self.assertEqual(len(self.planner.calls), 3)
        self.assertTrue(all(call.args[-1] == .5 for call in self.validator.call_args_list))
        self.assertEqual(self.controller.calls, [])
        self.assertEqual(self.g.gate.calls, [])

    def test_limit_replanning_is_bounded_to_two_retries(self):
        self.validator.side_effect = ValueError('关节规划多项式加速度越限')
        self.ready()
        for _ in range(2):
            self.planner.calls[-1][1].set_result(self.planning_result())
            self.finish_validation()
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(len(self.planner.calls), 3)
        self.assertEqual(self.controller.calls, [])

    def test_format_error_never_triggers_adaptive_replanning(self):
        self.validator.side_effect = ValueError('关节规划格式或数值无效')
        self.ready()
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(len(self.planner.calls), 1)
        self.assertEqual(self.motion.replans, 0)

    def test_cumulative_deadline_is_not_reset_by_replanning(self):
        self.validator.side_effect = ValueError('关节规划多项式加速度越限')
        self.ready()
        self.assertEqual(self.g.state, 'joint_planning')
        self.now.return_value = self.motion.plan_deadline+.01
        self.g.heartbeat = self.now.return_value
        self.g.stage_time = self.now.return_value-.01  # local stage deadline has not elapsed
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertIn('累计超时', self.g.reason)
        self.assertEqual(self.controller.calls, [])

    def test_long_math_validation_does_not_block_guard_feedback_or_heartbeat(self):
        entered, release = threading.Event(), threading.Event()
        worker_thread = []
        def slow_math(*args):
            worker_thread.append(threading.get_ident())
            entered.set()
            if not release.wait(2.):
                raise RuntimeError('test worker not released')
            return 2.
        self.validator.side_effect = slow_math
        self.plan()
        self.g.pause_servo.complete()
        started = time.perf_counter()
        self.planner.calls[-1][1].set_result(self.planning_result())
        elapsed = time.perf_counter()-started
        try:
            self.assertTrue(entered.wait(1.))
            self.assertLess(elapsed, .15)
            self.assertNotEqual(worker_thread[0], threading.get_ident())
            self.assertEqual(self.g.state, 'joint_validating')
            self.assertTrue(self.g.closed)
            self.assertEqual(self.g.gate.calls, [])
            before = self.g.health.call_count
            self.now.return_value = 100.1
            self.g.heartbeat = 100.1
            self.g.tick()
            self.assertGreater(self.g.health.call_count, before)
            self.assertEqual(self.g.state, 'joint_validating')
            self.assertEqual(self.controller.calls, [])
        finally:
            release.set()
        self.motion.pending_validation.result(timeout=1.)
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_ready')
        self.motion.command(NS(plan_id=self.motion.plan_id, command='execute'), NS())
        self.assertEqual(self.validator.call_count, 1)  # no second full validation at execution

    def test_validation_deadline_and_old_worker_result_cannot_rearm(self):
        entered, release = threading.Event(), threading.Event()
        def slow_math(*args):
            entered.set()
            release.wait(2.)
            return 2.
        self.validator.side_effect = slow_math
        self.plan()
        self.g.pause_servo.complete()
        self.planner.calls[-1][1].set_result(self.planning_result())
        self.assertTrue(entered.wait(1.))
        pending = self.motion.pending_validation
        try:
            self.now.return_value = 106.
            self.g.heartbeat = 106.
            self.g.tick()
            self.assertEqual(self.g.state, 'fault')
            self.assertEqual(self.motion.plan_id, '')
        finally:
            release.set()
        pending.result(timeout=1.)
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.controller.calls, [])

    def test_validated_plan_identity_is_required_without_revalidating_math(self):
        self.ready()
        self.motion.target_revision += 1
        result = self.motion.command(NS(plan_id=self.motion.plan_id, command='execute'), NS())
        self.assertFalse(result.success)
        self.assertEqual(self.g.state, 'fault')
        self.assertEqual(self.validator.call_count, 1)

    def test_mock_success_waits_for_real_feedback_and_close_hold_ack(self):
        self.ready()
        self.execute()
        handle = self.accepted()
        handle.get_result_async.return_value.set_result(NS(status=4, result=NS(error_code=0)))
        self.assertEqual(self.g.state, 'joint_settling')
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_settling')  # mock succeeded, physical state still at start
        self.g.health.return_value = self.goal.copy()
        self.g.feedback.get.return_value = self.goal.copy()
        self.g.commanded.get.return_value = self.goal.copy()
        self.now.return_value = 100.4
        self.g.heartbeat = 100.4
        self.g.tick()
        self.now.return_value = 100.41
        self.g.tick()
        self.now.return_value = 100.72
        self.g.heartbeat = 100.72
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_finishing')
        self.assertFalse(self.motion.status()['can_plan'])
        self.g.gate.complete()
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_finishing')
        self.g.hold.complete()
        self.g.tick()
        self.assertEqual(self.g.state, 'joint_done')
        self.assertTrue(self.motion.status()['can_plan'])
        self.assertEqual(self.motion.purpose, 'point')
        self.assertEqual(self.motion.target_revision, 4)
        self.assertFalse(self.motion.ready)

    def test_old_plan_cancel_cannot_cancel_a_new_plan(self):
        first = self.plan().plan_id
        self.g.stop('stop')
        self.g.gate.complete()
        self.g.hold.complete()
        self.g.tick()
        second = self.plan().plan_id
        response = self.motion.command(NS(plan_id=first, command='cancel'), NS())
        self.assertFalse(response.success)
        self.assertEqual(self.motion.plan_id, second)
        self.assertEqual(self.g.state, 'joint_pausing')

    def test_true_feedback_failure_still_stops_joint_mode(self):
        self.ready()
        self.execute()
        self.g.health.side_effect = ValueError('stale actual feedback')
        self.g.tick()
        self.assertEqual(self.g.state, 'fault')
        self.assertFalse(self.g.gate.calls[-1][0].data)


if __name__ == '__main__':
    unittest.main()
