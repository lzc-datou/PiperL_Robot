"""ROS adapter contract: only verified poses cross the pure controller boundary."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest

import numpy as np

try:
    from scripts.task.servo_tracking.node import Backend
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'ROS runtime wrapper required; no ROS node is created')
class ControllerBoundaryTests(unittest.TestCase):
    def test_unverified_gui_pose_never_enters_controller(self):
        verified = np.array([.3, .0, .4, .0, .0, .0, 1.])
        candidate = verified.copy(); candidate[0] += .10
        measured = verified.copy(); measured[1] += .01
        expected = np.array([.02, -.03, 0., .1, 0., 0.])
        controller = Mock(); controller.update.return_value = expected
        state = NS(controller=controller, approved_target=verified,
                   target=candidate, approved_target_time=99.9, scale=.5, dragging=True)
        with patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.):
            result = Backend.control_twist(state, measured)
        args, kwargs = controller.update.call_args
        np.testing.assert_array_equal(args[0], verified)
        np.testing.assert_array_equal(args[1], measured)
        self.assertEqual(kwargs, dict(now=100., target_time=99.9,
                                     speed_scale=.5, feedforward_active=True))
        np.testing.assert_array_equal(result, expected)

    def test_released_mouse_disables_feedforward_without_disabling_feedback(self):
        state = NS(controller=Mock(), approved_target=np.ones(7),
                   approved_target_time=99.9, scale=.6, dragging=False)
        with patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.):
            Backend.control_twist(state, np.zeros(7))
        self.assertFalse(state.controller.update.call_args.kwargs['feedforward_active'])
        state.controller.update.assert_called_once()

    def test_original_actual_measurement_stamp_reaches_release_velocity_estimator(self):
        state = NS(controller=Mock(), approved_target=np.ones(7), approved_target_time=99.9,
                   scale=.5, dragging=False, feedback=NS(stamp=1794637000.125))
        with patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.):
            Backend.control_twist(state, np.zeros(7))
        self.assertEqual(state.controller.update.call_args.kwargs['actual_time'], 1794637000.125)
        self.assertEqual(state.controller.update.call_args.kwargs['now'], 100.)

    def test_ik_completion_preserves_original_target_receipt_time(self):
        state = NS(target=np.array([.3, 0., .4, 0., 0., 0., 1.]),
                   last_target_change=99.8, active=False, start_pending=False,
                   publish_ghost=Mock())
        with patch('scripts.task.servo_tracking.node.time.monotonic', return_value=100.):
            Backend.accept_ik(state, np.zeros(6), 7)
        self.assertEqual(state.approved_target_time, 99.8)
        self.assertEqual(state.valid_revision, 7)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.approved_target, state.target)

    def test_controller_history_can_be_cleared_without_emitting_a_command(self):
        state = NS(controller=Mock(), publish_twist=Mock())
        Backend.reset_controller(state)
        state.controller.reset.assert_called_once()
        state.controller.update.assert_not_called()
        state.publish_twist.assert_not_called()


if __name__ == '__main__':
    unittest.main()
