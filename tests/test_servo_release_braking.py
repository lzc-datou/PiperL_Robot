"""Measured-motion release braking; deterministic, no nodes or hardware."""
from copy import deepcopy
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.task.servo_tracking.cartesian_control import CartesianController, load_control_config, validate_control_config
from tests.servo_control_benchmark import simulate
from tests.servo_release_braking_benchmark import metrics


def pose(x=0., angle=0.):
    return np.r_[[x, 0., 0.], Rotation.from_rotvec([0., 0., angle]).as_quat()]


class ReleaseBrakingTests(unittest.TestCase):
    def controller(self, **changes):
        config = load_control_config()
        config.update(algorithm='p', **changes)
        return CartesianController(config)

    def moving(self, controller, *, target=None, dragging=False, x=.0008, angle=0.):
        target = pose(.002, .03) if target is None else target
        controller.update(target, pose(), 1., 1., 1., actual_time=1000., feedforward_active=True)
        return controller.update(target, pose(x, angle), 1.02, 1., 1.,
            actual_time=1000.02, feedforward_active=dragging)

    def test_release_brakes_near_goal_using_actual_motion_before_crossing(self):
        controller = self.controller()
        target = pose(.002)
        result = self.moving(controller, target=target)
        self.assertGreater(controller.measured_velocity[0], 0.)
        self.assertLess(result[0], 2.*(.002-.0008))
        self.assertTrue(controller.settling_braking)
        self.assertFalse(controller.motion_settled)

    def test_identical_motion_while_dragging_and_far_target_pursuit(self):
        enabled, disabled = self.controller(), self.controller(settling_linear_prediction_time=0., settling_angular_prediction_time=0.)
        np.testing.assert_array_equal(self.moving(enabled, dragging=True), self.moving(disabled, dragging=True))
        enabled, disabled = self.controller(), self.controller(settling_linear_prediction_time=0., settling_angular_prediction_time=0.)
        np.testing.assert_array_equal(self.moving(enabled, target=pose(.05)), self.moving(disabled, target=pose(.05)))
        self.assertFalse(enabled.settling_braking)

    def test_target_step_does_not_cause_derivative_kick(self):
        controller = self.controller()
        controller.update(pose(), pose(), 1., 1., 1., actual_time=1000.)
        result = controller.update(pose(.05), pose(), 1.02, 1.02, 1.,
                                   actual_time=1000.02, feedforward_active=False)
        np.testing.assert_allclose(result, [.1, 0., 0., 0., 0., 0.])
        np.testing.assert_array_equal(controller.measured_velocity, np.zeros(6))

    def test_only_measurement_source_time_determines_velocity(self):
        controller = self.controller(measured_velocity_filter_time_constant=0.)
        controller.update(pose(.03), pose(), 1., 1., 1., actual_time=1000.)
        controller.update(pose(.03), pose(.001), 1.04, 1., 1., actual_time=1000.02)
        self.assertAlmostEqual(controller.measured_velocity[0], .05, places=9)
        held = controller.measured_velocity.copy()
        controller.update(pose(.03), pose(.001), 1.05, 1., 1., actual_time=1000.02)
        np.testing.assert_array_equal(controller.measured_velocity, held)
        self.assertEqual(controller._actual_received, 1.04)

    def test_stale_measurement_cannot_keep_a_braking_impulse_alive(self):
        controller = self.controller()
        self.moving(controller)
        controller.update(pose(.002, .03), pose(.0008), 1.08, 1., 1.,
                          actual_time=1000.02, feedforward_active=False)
        result = controller.update(pose(.002, .03), pose(.0008), 1.14, 1., 1.,
                          actual_time=1000.02, feedforward_active=False)
        self.assertIsNone(controller.measured_velocity)
        self.assertFalse(controller.settling_braking)
        self.assertFalse(controller.motion_settled)
        self.assertAlmostEqual(result[0], .0024)

    def test_clock_reversal_gap_and_explicit_reset_rewarm_estimator(self):
        for kind in ('backwards', 'gap', 'reset'):
            with self.subTest(kind=kind):
                controller = self.controller()
                self.moving(controller)
                if kind == 'reset':
                    controller.reset()
                stamp = 999. if kind == 'backwards' else 1000.3
                controller.update(pose(.002, .03), pose(.001), 1.04, 1., 1.,
                                  actual_time=stamp, feedforward_active=False)
                self.assertIsNone(controller.measured_velocity)
                self.assertFalse(controller.settling_braking)

    def test_tiny_source_progress_does_not_renew_old_speed(self):
        controller = self.controller()
        self.moving(controller)
        for index in range(1, 8):
            controller.update(pose(.002, .03), pose(.0008), 1.02+index*.02, 1., 1.,
                actual_time=1000.02+index*.0001, feedforward_active=False)
        self.assertIsNone(controller.measured_velocity)
        self.assertFalse(controller.settling_braking)

    def test_orientation_wrap_uses_short_base_frame_actual_velocity(self):
        controller = self.controller(measured_velocity_filter_time_constant=0.)
        target = pose(angle=np.deg2rad(185.))
        controller.update(target, pose(angle=np.deg2rad(179.)), 1., 1., 1., actual_time=1000.)
        result = controller.update(target, pose(angle=np.deg2rad(181.)), 1.02, 1., 1.,
                                  actual_time=1000.02, feedforward_active=False)
        self.assertAlmostEqual(controller.measured_velocity[5], np.deg2rad(2.)/.02, places=8)
        self.assertLess(result[5], 2.*np.deg2rad(4.))
        self.assertTrue(controller.settling_braking)

    def test_missing_velocity_preserves_legacy_p_without_inventing_motion(self):
        controller = self.controller()
        controller.update(pose(.02), pose(), 1., 1., .5, feedforward_active=False)
        result = controller.update(pose(.02), pose(.001), 1.02, 1., .5, feedforward_active=False)
        self.assertAlmostEqual(result[0], .038)
        self.assertIsNone(controller.measured_velocity)
        self.assertFalse(controller.settling_braking)

    def test_stationary_samples_are_settled_and_braking_respects_both_caps(self):
        controller = self.controller()
        controller.update(pose(), pose(), 1., 1., 1., actual_time=1000.)
        controller.update(pose(), pose(), 1.02, 1., 1., actual_time=1000.02, feedforward_active=False)
        self.assertTrue(controller.motion_settled)
        output = controller.update(pose(), pose(.03, .1), 1.03, 1., .1,
                                  actual_time=1000.03, feedforward_active=False)
        self.assertLessEqual(np.linalg.norm(output[:3]), .03+1e-12)
        self.assertLessEqual(np.linalg.norm(output[3:]), .15+1e-12)

    def test_invalid_prediction_and_velocity_parameters_rejected(self):
        for change in ({'settling_linear_prediction_time': -1.},
                       {'settling_angular_prediction_time': float('inf')},
                       {'measured_velocity_filter_time_constant': True},
                       {'measured_velocity_timeout': 0.},
                       {'hold_angular_velocity_tolerance': float('nan')}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_control_config(change)

    def test_delayed_release_overshoot_reduced_without_changing_drag_commands(self):
        config = load_control_config()
        before = deepcopy(config)
        before.update(settling_linear_prediction_time=0., settling_angular_prediction_time=0.)
        for delay, response in ((.04, .15), (.10, .15), (.16, .25)):
            with self.subTest(delay=delay, response=response):
                old, settings = simulate('p', before, plant=dict(transport_delay=delay,
                    velocity_time_constant=response), release_gated=True)
                new, _ = simulate('p', config, plant=dict(transport_delay=delay,
                    velocity_time_constant=response), release_gated=True)
                a, b = metrics(old, settings), metrics(new, settings)
                self.assertLess(b['position_overshoot_mm'], a['position_overshoot_mm']*.4)
                self.assertLess(b['orientation_overshoot_deg'], a['orientation_overshoot_deg']*.4)
                self.assertIsNotNone(b['settle_s'])
                np.testing.assert_array_equal([r['command'] for r in old if r['time'] < 8.],
                                              [r['command'] for r in new if r['time'] < 8.])


if __name__ == '__main__':
    unittest.main()
