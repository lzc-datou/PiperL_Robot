"""Mode switching, joint planning and preparation races; no ROS nodes or hardware."""
from concurrent.futures import Future
import json
from pathlib import Path
import time
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

try:
    from scripts.task.servo_tracking.node import Backend, pose_message
    from scripts.task.servo_tracking.core import load_config
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the built ROS runtime wrapper; all service calls are mocked')
class ServoModeTests(unittest.TestCase):
    def state(self, **changes):
        config = load_config(Path(__file__).resolve().parents[1] /
                             'scripts/task/servo_tracking/config/config.yaml')
        actual = np.array([.3, 0., .4, 0., 0., 0., 1.])
        target = actual.copy()
        target[0] += .04
        preview_q = np.array(config['ready_joints'])
        preview_q[0] += .04
        state = NS(config=config, model=Mock(), target=target, approved_target=target.copy(),
                   actual=Mock(return_value=(np.array(config['ready_joints']), actual)),
                   revision=8, valid_revision=8, aligned_revision=-1, valid=True,
                   preview_q=preview_q, ghost_revision=8,
                   control_mode='servo', active=False, dragging=False, start_pending=False,
                   joint_epoch=2, joint_id='', joint_purpose='', joint_revision=-1,
                   joint_busy=False, plan_ready=False, joint_wait=None, joint_future=None,
                   guard_can_plan=False, guard_state='idle', guard_reason='待启动',
                   pending=None, resume_future=None, verification_gap_since=None,
                   switched=True, switch_future=None, scale=.5, panel_seen=time.monotonic(),
                   reason='待启动', reachability='可达', heartbeat=Mock(), publish_twist=Mock(),
                   ik=Mock(), gate=Mock(), joint_plan=Mock(), joint_command=Mock(), align=Mock())
        for client in (state.ik, state.gate, state.joint_plan, state.joint_command):
            client.service_is_ready.return_value = True
            client.call_async.side_effect = lambda request: Future()
        state.model.condition_number.return_value = 10.
        state.model.radius = 1.
        for name in ('stop_tracking', 'begin_tracking', 'queue_joint_plan', 'finish_ready',
                     'joint_request_tick', 'joint_plan_done', 'joint_execute_done'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        state.start_done = Mock()
        for name, value in changes.items():
            setattr(state, name, value)
        return state

    def test_switching_mode_stops_active_motion_and_preserves_user_target(self):
        state = self.state(active=True, joint_busy=True, joint_id='old', joint_purpose='ready')
        target = state.target.copy()
        response = Backend.mode_service(state, NS(data=True), NS())
        self.assertTrue(response.success)
        self.assertEqual(state.control_mode, 'point')
        self.assertFalse(state.active or state.joint_busy or state.plan_ready)
        self.assertEqual(state.joint_epoch, 3)
        self.assertEqual(state.joint_id, '')
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target, target)

    def test_selecting_current_mode_does_not_interrupt_tracking(self):
        state = self.state(active=True)
        response = Backend.mode_service(state, NS(data=False), NS())
        self.assertTrue(response.success)
        self.assertTrue(state.active)
        self.assertEqual(state.joint_epoch, 2)
        state.gate.call_async.assert_not_called()

    def test_late_plan_response_after_mode_switch_is_cancelled(self):
        state = self.state(control_mode='point', joint_busy=True)
        old_epoch = state.joint_epoch
        Backend.mode_service(state, NS(data=False), NS())
        future = Future()
        future.set_result(NS(success=True, plan_id='late-old-plan', message='ready'))
        Backend.joint_plan_done(state, future, old_epoch)
        self.assertEqual(state.joint_id, '')
        self.assertFalse(state.plan_ready or state.joint_busy or state.active)
        request = state.joint_command.call_async.call_args.args[0]
        self.assertEqual((request.plan_id, request.command), ('late-old-plan', 'cancel'))

    def test_late_execute_failure_cannot_overwrite_new_session(self):
        state = self.state(active=True, reason='new-session', joint_epoch=7)
        future = Future()
        future.set_result(NS(success=False, message='old failure'))
        Backend.joint_execute_done(state, future, 6)
        self.assertTrue(state.active)
        self.assertEqual(state.reason, 'new-session')
        state.gate.call_async.assert_not_called()

    def test_new_target_invalidates_ready_plan_before_it_can_be_executed(self):
        state = self.state(control_mode='point', plan_ready=True,
                           joint_id='old-plan', joint_revision=8)
        target = state.target.copy()
        target[1] += .01
        Backend.update_target(state, pose_message(target))
        self.assertEqual(state.revision, 9)
        self.assertFalse(state.plan_ready or state.joint_busy)
        self.assertEqual(state.joint_id, '')
        self.assertIsNone(state.approved_target)
        response = Backend.execute_service(state, None, NS())
        self.assertFalse(response.success)
        state.joint_command.call_async.assert_not_called()
        np.testing.assert_array_equal(state.target, target)

    def test_execute_rejects_plan_for_an_older_revision(self):
        state = self.state(control_mode='point', plan_ready=True,
                           joint_id='old-plan', joint_revision=7)
        response = Backend.execute_service(state, None, NS())
        self.assertFalse(response.success)
        self.assertIn('重新规划', response.message)
        state.joint_command.call_async.assert_not_called()

    def test_single_point_plan_requests_never_auto_execute(self):
        state = self.state(control_mode='point')
        response = Backend.plan_service(state, None, NS())
        self.assertTrue(response.success)
        self.assertTrue(state.joint_busy)
        state.joint_plan.call_async.assert_not_called()
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        state.guard_can_plan = True
        Backend.joint_request_tick(state)
        request = state.joint_plan.call_async.call_args.args[0]
        self.assertEqual(request.purpose, 'point')
        self.assertFalse(request.auto_execute)
        self.assertEqual(request.target_revision, state.revision)

    def test_single_point_already_at_goal_neither_plans_nor_executes(self):
        for offset in (0., .0019):
            with self.subTest(joint_error=offset):
                state = self.state(control_mode='point')
                actual_q, actual_pose = state.actual()
                state.preview_q = actual_q.copy()
                state.preview_q[0] += offset
                state.target = actual_pose.copy()
                response = Backend.plan_service(state, None, NS())
                self.assertTrue(response.success)
                self.assertIn('已在目标位置', response.message)
                self.assertFalse(state.joint_busy or state.plan_ready or state.active)
                self.assertIsNone(state.joint_wait)
                state.joint_plan.call_async.assert_not_called()
                state.gate.call_async.assert_not_called()
                execute = Backend.execute_service(state, None, NS())
                self.assertFalse(execute.success)
                state.joint_command.call_async.assert_not_called()

    def test_singular_start_uses_bounded_servo_without_extra_prepare_motion(self):
        state = self.state()
        target = state.target.copy()
        state.model.condition_number.side_effect = AssertionError('No κ motion gate')
        response = Backend.start(state, None, NS())
        self.assertTrue(response.success)
        self.assertTrue(state.active)
        self.assertFalse(state.joint_busy)
        self.assertEqual(state.joint_purpose, '')
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target, target)
        state.joint_plan.call_async.assert_not_called()
        state.model.condition_number.assert_not_called()

    def test_ready_completion_waits_for_guard_before_enabling_servo(self):
        state = self.state(active=True)
        target = state.target.copy()
        Backend.finish_ready(state)
        self.assertTrue(state.joint_busy)
        self.assertEqual(state.joint_wait[1], 'handoff')
        Backend.joint_request_tick(state)
        state.gate.call_async.assert_not_called()
        state.guard_can_plan = True
        Backend.joint_request_tick(state)
        self.assertTrue(state.gate.call_async.call_args.args[0].data)
        self.assertFalse(state.joint_busy)
        self.assertIsNone(state.joint_wait)
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target, target)

    def test_ready_completion_does_not_gate_handoff_by_condition_number(self):
        state = self.state(active=True)
        state.model.condition_number.side_effect = AssertionError('No κ handoff gate')
        target = state.target.copy()
        Backend.finish_ready(state)
        self.assertTrue(state.active and state.joint_busy)
        self.assertEqual(state.joint_wait[1], 'handoff')
        state.gate.call_async.assert_not_called()
        state.model.condition_number.assert_not_called()
        state.align.assert_not_called()
        np.testing.assert_array_equal(state.target, target)

    def test_ready_completion_still_rejects_an_unverified_target(self):
        state = self.state(active=True, valid=False)
        target = state.target.copy()
        Backend.finish_ready(state)
        self.assertFalse(state.active)
        self.assertIsNone(state.joint_wait)
        self.assertFalse(state.gate.call_async.call_args.args[0].data)
        self.assertIn('请调整或重置目标', state.reason)
        np.testing.assert_array_equal(state.target, target)

    def test_guard_ready_status_cannot_restore_an_old_plan_revision(self):
        state = self.state(control_mode='point', joint_id='plan', joint_busy=True,
                           joint_revision=7)
        Backend.on_guard(state, NS(data=json.dumps(dict(state='joint_ready', reason='ready',
                              plan_id='plan', plan_ready=True, can_plan=False))))
        self.assertFalse(state.plan_ready)
        self.assertIn('失效', state.reason)

    def test_panel_disconnect_stops_before_ready_handoff_can_open_gate(self):
        state = self.state(active=True, joint_busy=True, guard_can_plan=True,
                           joint_wait=(np.zeros(6), 'handoff', time.monotonic()),
                           panel_seen=time.monotonic()-1.)
        Backend.tick(state)
        self.assertFalse(state.active)
        # No enable request may be sent before noticing the absent operator panel.
        self.assertTrue(state.gate.call_async.called)
        self.assertTrue(all(not call.args[0].data for call in state.gate.call_async.call_args_list))


if __name__ == '__main__':
    unittest.main()
