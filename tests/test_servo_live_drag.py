"""Mouse/IK timing regressions: no ROS node, native Servo, SDK or CAN starts.

The GUI produces 60 Hz targets, GetPositionIK returns each captured target after
45 ms, validation polls at 20 Hz and control ticks at 100 Hz.  Every reply is
therefore older than the newest mouse revision.  Sim's normally fast local IK
does not reliably exercise that order of events.
"""
from concurrent.futures import Future
import json
from pathlib import Path
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

try:
    from scripts.task.servo_tracking.cartesian_control import CartesianController
    from scripts.task.servo_tracking.core import JOINTS, RobotModel, load_config, joint_speed_limits
    from scripts.task.servo_tracking.node import Backend, pose_message, pose_values
    from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
    from builtin_interfaces.msg import Time
    ROS = True
except ImportError:
    ROS = False


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


@unittest.skipUnless(ROS, 'Use ROS runtime wrapper; all transports are fake')
class ServoLiveDragTests(unittest.TestCase):
    def setUp(self):
        self.now = patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.).start()
        self.addCleanup(patch.stopall)
        self.scheduled = []
        self.replies = []
        self.enable_times = []
        self.outputs = []
        self.fail_goal_poses = []
        self.fail_reference_seed = False

    def state(self, guard_state='running', ik_delay=.045):
        config = load_config(ROOT / 'scripts/task/servo_tracking/config/config.yaml')
        model = RobotModel(URDF.read_text(), tip='link6')
        q = np.asarray(config['ready_joints'], dtype=float)
        actual = model.pose(q)
        state = NS(config=config, controller=Mock(wraps=CartesianController(config['control'])), model=model,
            actual=Mock(return_value=(q, actual)), raw_actual=Mock(return_value=(q, actual)),
            mode='sim', target=actual.copy(), approved_target=actual.copy(), approved_target_time=99.9,
            revision=10, valid_revision=10, valid=True, active=True, start_pending=False,
            pending=None, pending_since=0., ik_pending_revision=-1, ik_seed_revision=-1,
            ik_seeds=[], ik_seed_index=0, ik_best_q=None, ik_best_margin=-1.,
            verification_gap_since=None, last_target_change=99.9, aligned_revision=-1,
            preview_q=q.copy(), ghost_revision=10, publish_ghost=Mock(),
            dragging=False, drag_client='', drag_seen=100., panel_seen=100., switched=True,
            guard_state=guard_state, guard_reason='', guard_can_plan=guard_state == 'holding',
            pause_revision=10 if guard_state == 'holding' else -1,
            resume_future=None, resume_request_id=0, resume_requested_at=0., last_resume_attempt=0.,
            hold_future=None, hold_revision=-1, hold_requested_at=0., hold_reason='',
            settle_since=0., settle_q=None, joint_epoch=1, joint_busy=False, plan_ready=False,
            joint_id='', joint_purpose='', joint_revision=-1, joint_wait=None, joint_future=None,
            control_mode='servo', motion_recovery_required=False, recovery_hold=False,
            recovery_attempt_revision=-1, recovery_since=0., stall_since=0., stall_error=None,
            stall_q=None, reason='跟踪中', reachability='可达', scale=.5,
            heartbeat=Mock(), publish_twist=Mock(), ik=Mock(), gate=Mock(), hold_gate=Mock(),
            get_clock=Mock(), queue_joint_plan=Mock())
        for name in ('update_target', 'on_marker', 'ik_done', 'validation_tick', 'reject',
                     'maybe_hold', 'hold_done', 'maybe_resume', 'resume_done', 'stop_tracking',
                     'begin_tracking', 'start_done'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        for service in (state.ik, state.gate, state.hold_gate):
            service.service_is_ready.return_value = True
        state.get_clock.return_value.now.return_value.to_msg.return_value = Time()

        def ik_request(request):
            captured = pose_values(request.ik_request.pose_stamped.pose)
            # The test generates goals using FK of a known strictly legal q.
            # Return that captured goal, never the later mutable GUI target.
            distances = [np.linalg.norm(pose-captured) for pose, _ in self.goals]
            index = int(np.argmin(distances))
            self.assertLess(distances[index], 1e-10)
            captured_q = self.goals[index][1].copy()
            seed = np.asarray(request.ik_request.robot_state.joint_state.position)
            reference_failed = self.fail_reference_seed and np.allclose(seed, q, rtol=0, atol=1e-10)
            future = Future()
            future.set_running_or_notify_cancel()  # Stop cannot erase a late transport reply.

            def reply():
                self.replies.append((self.now.return_value, captured.copy()))
                error = -31 if reference_failed or any(np.allclose(captured, pose, rtol=0, atol=1e-10)
                                                       for pose in self.fail_goal_poses) else 1
                future.set_result(NS(error_code=NS(val=error, SUCCESS=1),
                    solution=NS(joint_state=NS(name=JOINTS, position=captured_q.tolist()))))

            self.scheduled.append((self.now.return_value + ik_delay, reply))
            return future

        def gate_request(request):
            future = Future()
            if request.data:
                self.enable_times.append(self.now.return_value)
                # Fake only the Guard's authoritative state transition, delayed
                # beyond the enable reply, as in the real protocol.
                self.scheduled.append((self.now.return_value + .030, lambda:
                    Backend.on_guard(state, NS(data=json.dumps(dict(state='running', reason='跟踪中'))))))
            self.scheduled.append((self.now.return_value + .010,
                                   lambda: future.set_result(NS(success=True, message='fake ACK'))))
            return future

        state.ik.call_async.side_effect = ik_request
        state.gate.call_async.side_effect = gate_request
        def hold_request(request):
            future = Future()
            self.scheduled.append((self.now.return_value + .010,
                                   lambda: future.set_result(NS(success=True, message='fake hold ACK'))))
            self.scheduled.append((self.now.return_value + .030, lambda:
                Backend.on_guard(state, NS(data=json.dumps(dict(state='holding', reason='保持目标'))))))
            return future

        state.hold_gate.call_async.side_effect = hold_request
        state.publish_twist.side_effect = lambda values: self.outputs.append(
            (self.now.return_value, np.asarray(values).copy()))
        self.goals = [(actual.copy(), q.copy())]
        self.q, self.actual_pose, self.backend = q, actual, state
        return state

    def event(self, kind, pose):
        message = Feedback()
        message.header.frame_id = 'base_link'
        message.marker_name, message.client_id = 'tcp_target', 'timing_test'
        message.control_name, message.event_type = 'move_free', kind
        message.pose = pose_message(pose)
        Backend.on_marker(self.backend, message)

    def advance(self, until):
        """Deterministic scheduler; validation and feedback have different phases."""
        while self.now.return_value < until - 1e-9:
            self.now.return_value = round(self.now.return_value + .001, 6)
            state = self.backend
            state.panel_seen = state.drag_seen = self.now.return_value
            due = [item for item in self.scheduled if item[0] <= self.now.return_value + 1e-9]
            self.scheduled = [item for item in self.scheduled if item not in due]
            for _, callback in due:
                callback()
            elapsed_ms = round((self.now.return_value-100.)*1000)
            if elapsed_ms % 50 == 3:
                Backend.validation_tick(state)
            if elapsed_ms % 10 == 7:
                Backend.tick(state)

    def continuous_drag(self, guard_state='running', ik_delay=.045):
        state = self.state(guard_state, ik_delay=ik_delay)
        self.event(Feedback.MOUSE_DOWN, self.actual_pose)
        for index in range(60):
            self.advance(100. + index/60.)
            q = self.q.copy()
            q[0] += .012 + (index+1)*.0008
            pose = state.model.pose(q)
            self.goals.append((pose.copy(), q))
            self.event(Feedback.POSE_UPDATE, pose)
        self.advance(101.)
        self.assertTrue(state.dragging, 'Assertion must run before MOUSE_UP')
        return state

    def test_continuous_updates_publish_verified_motion_before_mouse_release(self):
        state = self.continuous_drag()
        moving = [(stamp, command) for stamp, command in self.outputs if np.linalg.norm(command) > 1e-6]
        self.assertGreater(len(moving), 30, 'Legal delayed IK replies must not starve a moving target')
        self.assertLess(moving[0][0], 100.20)
        self.assertTrue(state.active)
        self.assertFalse(state.motion_recovery_required)
        state.hold_gate.call_async.assert_not_called()
        self.assertIsNotNone(state.approved_target)
        self.assertGreater(np.linalg.norm(state.approved_target[:3]-self.actual_pose[:3]), .008)
        verified = [self.actual_pose] + [pose for _, pose in self.replies]
        for call in state.controller.update.call_args_list:
            self.assertTrue(any(np.allclose(call.args[0], pose, rtol=0, atol=1e-10) for pose in verified),
                            'Controller must see a checked snapshot, never a newer unchecked mouse goal')

    def test_settled_session_resumes_during_drag_despite_newer_unchecked_revision(self):
        state = self.continuous_drag('holding')
        self.assertTrue(self.enable_times, 'Holding must resume while dragging, not only after MOUSE_UP')
        self.assertLess(self.enable_times[0], 100.20)
        self.assertEqual(state.guard_state, 'running')
        self.assertTrue(any(np.linalg.norm(command) > 1e-6 for _, command in self.outputs))
        self.assertTrue(state.active)

    def test_fast_local_ik_positive_control_does_not_hide_delayed_reply_order(self):
        state = self.continuous_drag(ik_delay=.005)
        self.assertTrue(state.dragging and state.active)
        self.assertGreater(sum(int(np.linalg.norm(command)>1e-6) for _, command in self.outputs), 30)
        state.hold_gate.call_async.assert_not_called()

    def test_continuous_drag_can_reach_fallback_when_reference_seed_fails_locally(self):
        self.fail_reference_seed = True
        state = self.continuous_drag()
        moving = [(stamp, values) for stamp, values in self.outputs if np.linalg.norm(values) > 1e-6]
        self.assertTrue(moving, 'Repeated local failures must not retry only the same reference seed until release')
        self.assertLess(moving[0][0], 100.35)
        self.assertTrue(state.active and state.dragging)

    def test_released_final_target_is_eventually_verified_without_a_second_start(self):
        state = self.continuous_drag()
        final = state.target.copy()
        self.event(Feedback.MOUSE_UP, final)
        # Static comparison may finish up to six 45ms candidates. This bound
        # covers that finite budget without weakening any pre-release check.
        self.advance(101.40)
        self.assertIsNotNone(state.approved_target)
        np.testing.assert_allclose(state.approved_target, final, atol=1e-10)
        self.assertEqual(state.valid_revision, state.revision)
        self.assertTrue(state.active)

    def test_stop_ignores_uncancellable_reply_and_cannot_reenable(self):
        state = self.state('holding')
        q = self.q.copy()
        q[0] += .04
        pose = state.model.pose(q)
        self.goals.append((pose, q))
        self.event(Feedback.MOUSE_DOWN, self.actual_pose)
        self.event(Feedback.POSE_UPDATE, pose)
        self.advance(100.020)
        self.assertIsNotNone(state.pending)
        Backend.stop_service(state, None, NS())
        # First reply at .048; stop checking before the next .053 validation
        # timer can intentionally request a new inactive ghost-only preview.
        self.advance(100.051)
        self.assertTrue(self.replies, 'The old reply must actually arrive after stop')
        self.assertFalse(state.active)
        self.assertIsNone(state.approved_target)
        self.assertEqual(self.enable_times, [])
        self.assertFalse(any(np.linalg.norm(command) > 1e-6 for _, command in self.outputs))

    def test_known_outside_workspace_edit_cannot_be_authorized_by_old_legal_reply(self):
        state = self.state()
        q = self.q.copy()
        q[0] += .04
        pose = state.model.pose(q)
        self.goals.append((pose, q))
        self.event(Feedback.MOUSE_DOWN, self.actual_pose)
        self.event(Feedback.POSE_UPDATE, pose)
        self.advance(100.020)
        pending = state.pending
        self.assertIsNotNone(pending)
        outside = pose.copy()
        outside[:3] = [3., 3., 3.]
        self.event(Feedback.POSE_UPDATE, outside)
        self.advance(100.100)
        self.assertTrue(self.replies, 'The previously legal result must actually arrive')
        self.assertIsNone(state.approved_target)
        self.assertFalse(state.valid)
        self.assertTrue(state.active, 'A rejected edit holds, preserving the session')
        self.assertIn('超出可达范围', state.reachability)
        self.assertEqual(self.enable_times, [])
        self.assertFalse(any(stamp > 100.020 and np.linalg.norm(command) > 1e-6
                             for stamp, command in self.outputs))

    def test_failed_latest_ik_revokes_stream_checkpoint_and_duplicate_old_reply(self):
        # Isolate explicit NO_IK handling: all six failures fit within the
        # verification deadline, rather than testing its separate timeout path.
        state = self.state(ik_delay=.020)
        q = self.q.copy()
        q[0] += .04
        legal = state.model.pose(q)
        self.goals.append((legal, q.copy()))
        self.event(Feedback.MOUSE_DOWN, self.actual_pose)
        self.event(Feedback.POSE_UPDATE, legal)
        self.advance(100.020)
        old, old_revision = state.pending, state.revision
        q[0] += .005
        colliding = state.model.pose(q)
        self.goals.append((colliding, q.copy()))
        self.fail_goal_poses.append(colliding)
        self.event(Feedback.POSE_UPDATE, colliding)
        self.advance(100.250)  # Both bounded IK seed attempts finish unsuccessfully.
        self.assertFalse(state.valid)
        self.assertIsNone(state.approved_target)
        self.assertIn('不可达或碰撞', state.reachability)
        self.assertTrue(state.active)
        Backend.ik_done(state, old, old_revision)  # A duplicate late callback has no authority.
        self.assertIsNone(state.approved_target)
        self.assertEqual(self.enable_times, [])
        self.advance(100.300)
        self.assertFalse(any(stamp > 100.250 and np.linalg.norm(command) > 1e-6
                             for stamp, command in self.outputs))

    def test_selected_joint_jog_input_is_nonzero_bounded_and_zeroed_on_stop_command(self):
        state = self.state()
        state.jog, state.twist = Mock(), Mock()
        Backend.publish_twist(state, [.15, .05, .02, .2, .5, .1])
        message = state.jog.publish.call_args.args[0]
        velocity = np.asarray(message.velocities)
        self.assertEqual(message.joint_names, JOINTS)
        self.assertEqual(message.header.frame_id, 'base_link')
        self.assertEqual(velocity.shape, (6,))
        self.assertTrue(np.all(np.isfinite(velocity)))
        self.assertGreater(np.linalg.norm(velocity), 1e-5)
        self.assertTrue(np.all(np.abs(velocity) <= joint_speed_limits(state.config, state.scale)+1e-12))
        Backend.publish_twist(state, np.zeros(6))
        np.testing.assert_array_equal(state.jog.publish.call_args.args[0].velocities, np.zeros(6))

    def test_joint_jog_input_is_zero_when_fresh_feedback_is_unavailable(self):
        state = self.state()
        state.jog, state.twist = Mock(), Mock()
        state.actual.side_effect = ValueError('feedback expired')
        Backend.publish_twist(state, [.1, 0., 0., 0., 0., 0.])
        np.testing.assert_array_equal(state.jog.publish.call_args.args[0].velocities, np.zeros(6))
        self.assertEqual(state.twist.publish.call_args.args[0].twist.linear.x, 0.)


if __name__ == '__main__':
    unittest.main()
