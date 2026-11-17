"""Pure-math Cartesian control contract; no ROS imports, nodes, or hardware."""
from copy import deepcopy
from dataclasses import replace
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from scripts.task.servo_tracking.cartesian_control import (
    ALGORITHMS, CartesianController, ExecutionFeedback, load_control_config, validate_control_config)


CONFIG = Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/control.yaml'


def pose(position=(0., 0., 0.), rotation=(0., 0., 0.)):
    return np.r_[position, Rotation.from_rotvec(rotation).as_quat()]


def execution_feedback(controller, now, limited=(False, False), effective=None, **updates):
    if controller.last_command_time is None:
        return None
    result = ExecutionFeedback(stamp=now, command_time=controller.last_command_time,
        requested_twist=controller.last_command_twist, limited=limited,
        effective_twist=effective)
    return replace(result, **updates)


class CartesianControlTests(unittest.TestCase):
    def controller(self, algorithm='feedforward_p', **updates):
        config = load_control_config(CONFIG)
        config.update(algorithm=algorithm, **updates)
        return CartesianController(config)

    def test_p_baseline_units_base_axes_and_shortest_orientation(self):
        controller = self.controller('p')
        actual = pose(rotation=(0., 0., math.pi/2))
        target = pose(position=(.04, .01, -.02), rotation=(0., 0., math.pi/2+.1))
        result = controller.update(target, actual, 1., 1., .5)
        np.testing.assert_allclose(result, [.08, .02, -.04, 0., 0., .2], atol=1e-12)
        self.assertEqual(result.shape, (6,))

    def test_all_algorithms_enforce_separate_linear_and_angular_norm_caps_and_scale(self):
        target, actual = pose((1., 2., 3.), (.5, .5, .5)), pose()
        for algorithm in ALGORITHMS:
            for scale in (0., .1, .5, 1.):
                with self.subTest(algorithm=algorithm, scale=scale):
                    controller = self.controller(algorithm)
                    output = controller.update(target, actual, 1., 1., scale)
                    self.assertTrue(np.isfinite(output).all())
                    self.assertLessEqual(np.linalg.norm(output[:3]), .30*scale+1e-12)
                    self.assertLessEqual(np.linalg.norm(output[3:]), 1.50*scale+1e-12)

    def test_quaternion_sign_and_two_degree_wrap_have_identical_short_arc(self):
        actual = pose(rotation=(0., 0., math.radians(179.)))
        target = pose(rotation=(0., 0., math.radians(-179.)))
        changed_sign = target.copy(); changed_sign[3:] *= -1.
        a = self.controller('p').update(target, actual, 1., 1., 1.)
        b = self.controller('p').update(changed_sign, actual, 1., 1., 1.)
        np.testing.assert_allclose(a, b, atol=1e-12)
        self.assertAlmostEqual(a[5], 2.*math.radians(2.))

    def test_feedforward_estimates_only_new_verified_target_times(self):
        controller = self.controller()
        origin, target = pose(), pose((.002, 0., 0.))
        controller.update(origin, origin, 1., 1., .5)
        moving = controller.update(target, target, 1.02, 1.02, .5)
        self.assertGreater(moving[0], 0.)  # Actual equals target: this is feedforward alone.
        repeated = controller.update(target, target, 1.03, 1.02, .5)
        np.testing.assert_allclose(moving, repeated, atol=1e-12)
        self.assertAlmostEqual(controller._target_velocity[0], .1*(1.-math.exp(-.02/.04)))

    def test_stale_feedforward_fades_then_stops_within_point_one_seconds(self):
        controller = self.controller()
        first, target = pose(), pose((.002, 0., 0.), (0., 0., .004))
        controller.update(first, first, 1., 1., 1.)
        moving = controller.update(target, target, 1.02, 1.02, 1.)
        held = controller.update(target, target, 1.059, 1.02, 1.)
        fading = controller.update(target, target, 1.09, 1.02, 1.)
        stopped = controller.update(target, target, 1.12, 1.02, 1.)
        np.testing.assert_allclose(held, moving)
        self.assertGreater(np.linalg.norm(fading), 0.)
        self.assertLess(np.linalg.norm(fading), np.linalg.norm(held))
        np.testing.assert_allclose(stopped, np.zeros(6), atol=1e-14)

    def test_angular_feedforward_is_expressed_in_base_axes_about_the_tcp(self):
        controller = self.controller()
        first = pose(rotation=(math.pi/2, 0., 0.))
        next_pose = first.copy()
        next_pose[3:] = (Rotation.from_rotvec([0., 0., .02]) *
                         Rotation.from_quat(first[3:])).as_quat()
        controller.update(first, first, 1., 1., 1.)
        output = controller.update(next_pose, next_pose, 1.02, 1.02, 1.)
        self.assertGreater(output[5], 0.)
        np.testing.assert_allclose(output[:5], np.zeros(5), atol=1e-12)

    def test_mouse_release_immediately_clears_feedforward_but_retains_p_correction(self):
        controller = self.controller()
        first, target = pose(), pose((.004, 0., 0.))
        controller.update(first, first, 1., 1., 1.)
        output = controller.update(target, target, 1.02, 1.02, 1.)
        self.assertGreater(output[0], 0.)
        released = controller.update(target, target, 1.03, 1.02, 1., feedforward_active=False)
        np.testing.assert_array_equal(released, np.zeros(6))
        np.testing.assert_array_equal(controller._target_velocity, np.zeros(6))
        self.assertEqual(controller._target_time, 1.02)
        corrected = controller.update(target, first, 1.04, 1.02, 1., feedforward_active=False)
        np.testing.assert_allclose(corrected, [.016, 0., 0., 0., 0., 0.])
        # A release never weakens the verified-target timestamp contract.
        with self.assertRaisesRegex(ValueError, 'requires a new target_time'):
            controller.update(first, first, 1.05, 1.02, 1., feedforward_active=False)

    def test_mouse_release_does_not_reset_pi_integral_or_normal_derivative_history(self):
        controller = self.controller('pi')
        actual, target = pose(), pose((.01, 0., 0.))
        for index in range(20):
            now = 1.+index*.01
            controller.update(target, actual, now, 1., 1.,
                              execution_feedback=execution_feedback(controller, now))
        integral, derivative, previous = (controller._integral.copy(),
            controller._derivative.copy(), controller._previous_error.copy())
        self.assertGreater(integral[0], 0.)
        output = controller.update(target, actual, 1.2, 1., 1., feedforward_active=False,
                                   execution_feedback=execution_feedback(controller, 1.2))
        self.assertGreater(output[0], .02)
        np.testing.assert_allclose(controller._integral, integral + previous*.01)
        np.testing.assert_allclose(controller._derivative, derivative)
        np.testing.assert_array_equal(controller._previous_error, previous)

    def test_feedforward_switch_requires_a_boolean(self):
        for value in (0, 1, 'false', None):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'boolean'):
                self.controller().update(pose(), pose(), 1., 1., 1., feedforward_active=value)

    def test_antipodal_quaternion_repeat_is_not_a_new_target_or_velocity(self):
        controller = self.controller()
        actual = pose()
        target = pose(rotation=(.2, .1, .1))
        controller.update(target, actual, 1., 1., 1.)
        target[3:] *= -1
        controller.update(target, actual, 1.01, 1., 1.)
        np.testing.assert_array_equal(controller._target_velocity, np.zeros(6))

    def test_pd_rotation_derivative_does_not_subtract_opposite_pi_rotation_vectors(self):
        controller = self.controller('pd')
        actual = pose()
        controller.update(pose(rotation=(0., 0., math.radians(179.))), actual, 1., 1., 1.)
        controller.update(pose(rotation=(0., 0., math.radians(181.))), actual, 1.02, 1.02, 1.)
        self.assertLess(np.linalg.norm(controller._derivative[3:]), 2.)

    def test_pi_saturation_does_not_wind_up_and_target_arrival_clears_integral(self):
        controller = self.controller('pi')
        actual, target = pose(), pose((1., 0., 0.), (0., 0., 1.))
        for index in range(50):
            now = 1.+index*.01
            output = controller.update(target, actual, now, 1., .5,
                                       execution_feedback=execution_feedback(controller, now))
            self.assertLessEqual(np.linalg.norm(output[:3]), .15+1e-12)
        np.testing.assert_array_equal(controller._integral, np.zeros(6))
        np.testing.assert_allclose(controller.update(target, target, 1.5, 1., .5), np.zeros(6))

    def test_pi_integral_state_is_bounded_in_both_blocks(self):
        controller = self.controller('pi', integral_position_limit=.002, integral_orientation_limit=.003)
        target, actual = pose((.01, 0., 0.), (0., 0., .02)), pose()
        for index in range(100):
            now = 1.+index*.01
            controller.update(target, actual, now, 1., 1.,
                              execution_feedback=execution_feedback(controller, now))
        self.assertGreater(np.linalg.norm(controller._integral[:3]), 0.)
        self.assertLessEqual(np.linalg.norm(controller._integral[:3]), .002+1e-12)
        self.assertLessEqual(np.linalg.norm(controller._integral[3:]), .003+1e-12)

    def test_missing_feedback_freezes_growth_but_allows_only_unwinding_toward_zero(self):
        controller = self.controller('pi')
        target, actual = pose((.02, 0., 0.)), pose()
        controller.update(target, actual, 1., 1., .5)
        controller._integral[0] = .0001
        output = controller.update(target, actual, 1.01, 1., .5)
        self.assertEqual(controller.antiwindup_state, 'feedback_missing')
        self.assertEqual(controller._integral[0], .0001)
        self.assertGreater(output[0], 0.)
        # Opposite error may remove old I, but cannot create an opposite bias.
        controller.update(pose((-.02, 0., 0.)), actual, 1.02, 1.02, .5)
        self.assertEqual(controller._integral[0], 0.)

    def test_fresh_unlimited_measured_feedback_does_not_mistake_natural_delay_for_saturation(self):
        controller = self.controller('pi')
        target, actual = pose((.02, 0., 0.)), pose()
        controller.update(target, actual, 1., 1., .5)
        feedback = execution_feedback(controller, 1.01, effective=np.zeros(6), source='measured')
        controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        self.assertEqual(controller.antiwindup_state, 'normal')
        self.assertAlmostEqual(controller._integral[0], .02*.01)

    def test_downstream_limit_back_calculates_when_local_tcp_output_is_not_saturated(self):
        controller = self.controller('pi')
        controller._integral[0] = .03
        target, actual = pose((.03, 0., 0.)), pose()
        previous = controller.update(target, actual, 1., 1., .5)
        self.assertLess(previous[0], .15)
        feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
        output = controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        expected = .03-(1.-math.exp(-.01/.2))*previous[0]/.8
        self.assertAlmostEqual(controller._integral[0], expected)
        self.assertLess(output[0], previous[0])
        self.assertEqual(controller.antiwindup_state, 'tracking')

    def test_default_clamped_tracking_does_not_invent_opposite_integral_from_p_saturation(self):
        for mode, expected_negative in (('clamped_tracking', False), ('tracking', True)):
            controller = self.controller('pi', antiwindup_mode=mode)
            target, actual = pose((.03, 0., 0.)), pose()
            controller.update(target, actual, 1., 1., .5)
            feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
            controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
            with self.subTest(mode=mode):
                if expected_negative:
                    self.assertLess(controller._integral[0], 0.)
                else:
                    self.assertEqual(controller._integral[0], 0.)

    def test_clamped_back_calculation_can_remove_existing_integral_but_never_cross_zero(self):
        controller = self.controller('pi', antiwindup_tracking_time_constant=.001)
        controller._integral[0] = .0001
        target, actual = pose((.03, 0., 0.)), pose()
        controller.update(target, actual, 1., 1., .5)
        feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
        controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        self.assertEqual(controller._integral[0], 0.)

    def test_reverse_error_can_unwind_and_cross_zero_even_while_old_direction_is_limited(self):
        controller = self.controller('pi')
        controller._integral[0] = .0001
        actual, target = pose(), pose((.02, 0., 0.))
        controller.update(target, actual, 1., 1., .5)
        feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
        controller.update(pose((-.02, 0., 0.)), actual, 1.01, 1.01, .5,
                          execution_feedback=feedback)
        self.assertAlmostEqual(controller._integral[0], -.0001)

    def test_linear_saturation_does_not_freeze_unlimited_angular_integral(self):
        controller = self.controller('pi')
        controller._integral[[0, 5]] = [.01, .02]
        target, actual = pose((.02, 0., 0.), (0., 0., .03)), pose()
        command = controller.update(target, actual, 1., 1., .5)
        effective = command.copy(); effective[:3] = 0.
        feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=effective)
        controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        self.assertLess(controller._integral[0], .01)
        self.assertAlmostEqual(controller._integral[5], .02+.03*.01)

    def test_angular_saturation_does_not_apply_back_calculation_to_linear_block(self):
        controller = self.controller('pi', antiwindup_tracking_time_constant=.001)
        controller._integral[[0, 5]] = [.01, .0001]
        target, actual = pose((.02, 0., 0.), (0., 0., .03)), pose()
        command = controller.update(target, actual, 1., 1., .5)
        effective = command.copy(); effective[3:] = 0.
        feedback = execution_feedback(controller, 1.01, limited=(False, True), effective=effective)
        controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        self.assertAlmostEqual(controller._integral[0], .01+.02*.01)
        self.assertEqual(controller._integral[5], 0.)

    def test_limit_without_effective_twist_freezes_growth_without_guessing_back_calculation(self):
        controller = self.controller('pi')
        target, actual = pose((.02, 0., 0.)), pose()
        controller.update(target, actual, 1., 1., .5)
        feedback = execution_feedback(controller, 1.01, limited=(True, False))
        output = controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        self.assertEqual(controller._integral[0], 0.)
        self.assertAlmostEqual(output[0], .04)
        self.assertEqual(controller.antiwindup_state, 'limited_freeze')

    def test_variable_execution_delay_matches_history_and_rejects_late_older_commands(self):
        controller = self.controller('pi')
        target, actual = pose((.02, 0., 0.)), pose()
        history = []
        for index in range(26):
            now = 1.+index*.01
            command = controller.update(target, actual, now, 1., .5)
            history.append((now, command))
        old = ExecutionFeedback(stamp=1.25, command_time=history[0][0],
            requested_twist=history[0][1], limited=(False, False))
        controller.update(target, actual, 1.26, 1., .5, execution_feedback=old)
        self.assertEqual(controller.antiwindup_state, 'normal')  # 260 ms variable delay is accepted.
        newer = replace(old, stamp=1.27, command_time=history[20][0], requested_twist=history[20][1])
        controller.update(target, actual, 1.27, 1., .5, execution_feedback=newer)
        previous = controller._integral.copy()
        late = replace(old, stamp=1.28, command_time=history[10][0], requested_twist=history[10][1])
        controller.update(target, actual, 1.28, 1., .5, execution_feedback=late)
        self.assertEqual(controller.antiwindup_state, 'feedback_out_of_order')
        np.testing.assert_array_equal(controller._integral, previous)

    def test_stale_source_expired_command_and_wrong_request_cannot_authorize_integral_growth(self):
        for failure in ('stale', 'expired', 'mismatch', 'invalid', 'future'):
            with self.subTest(failure=failure):
                controller = self.controller('pi')
                target, actual = pose((.02, 0., 0.)), pose()
                command = controller.update(target, actual, 1., 1., .5)
                end = 1.6 if failure == 'expired' else 1.12
                for index in range(1, round((end-1.)/.01)):
                    controller.update(target, actual, 1.+index*.01, 1., .5)
                feedback = ExecutionFeedback(stamp=end, command_time=1., requested_twist=command,
                                              limited=(False, False))
                if failure == 'stale':
                    feedback = replace(feedback, stamp=1.)
                elif failure == 'mismatch':
                    feedback = replace(feedback, requested_twist=np.zeros(6))
                elif failure == 'invalid':
                    feedback = replace(feedback, valid=False)
                elif failure == 'future':
                    feedback = replace(feedback, stamp=end+.01)
                output = controller.update(target, actual, end, 1., .5, execution_feedback=feedback)
                self.assertEqual(controller._integral[0], 0.)
                self.assertNotEqual(controller.antiwindup_state, 'normal')
                self.assertGreater(output[0], 0.)

    def test_reset_rejects_pre_reset_feedback_even_with_nearly_equal_command_time(self):
        controller = self.controller('pi')
        target, actual = pose((.02, 0., 0.)), pose()
        command = controller.update(target, actual, 1., 1., .5)
        controller.reset()
        controller.update(target, actual, 1.02, 1.02, .5)
        feedback = ExecutionFeedback(stamp=1.03, command_time=1.02-.0000005,
            requested_twist=command, limited=(False, False))
        controller.update(target, actual, 1.03, 1.02, .5, execution_feedback=feedback)
        self.assertEqual(controller.antiwindup_state, 'feedback_before_reset')
        self.assertEqual(controller._integral[0], 0.)

    def test_zero_ki_and_arrival_never_receive_back_calculated_integral(self):
        for failure in ('zero_ki', 'arrived'):
            controller = self.controller('pi')
            if failure == 'zero_ki':
                controller.config['gains']['pi']['position_i_gain'] = 0.
            target, actual = pose((.02, 0., 0.)), pose()
            controller.update(target, actual, 1., 1., .5)
            feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
            output = controller.update(target, target if failure == 'arrived' else actual,
                1.01, 1., .5, execution_feedback=feedback)
            with self.subTest(failure=failure):
                self.assertEqual(controller._integral[0], 0.)
                self.assertTrue(np.isfinite(output).all())
                if failure == 'arrived':
                    np.testing.assert_array_equal(output, np.zeros(6))

    def test_external_saturation_release_restores_normal_integration(self):
        controller = self.controller('pi')
        controller._integral[0] = .02
        target, actual = pose((.02, 0., 0.)), pose()
        controller.update(target, actual, 1., 1., .5)
        feedback = execution_feedback(controller, 1.01, limited=(True, False), effective=np.zeros(6))
        controller.update(target, actual, 1.01, 1., .5, execution_feedback=feedback)
        before = controller._integral[0]
        controller.update(target, actual, 1.02, 1., .5,
            execution_feedback=execution_feedback(controller, 1.02))
        self.assertAlmostEqual(controller._integral[0], before+.02*.01)
        self.assertEqual(controller.antiwindup_state, 'normal')

    def test_optional_cartesian_acceleration_and_reduced_speed_cap(self):
        controller = self.controller('p', linear_acceleration=2., angular_acceleration=4.)
        actual, target = pose(), pose((1., 0., 0.), (0., 0., 1.))
        previous = controller.update(target, actual, 1., 1., .5)
        for index in range(1, 10):
            output = controller.update(target, actual, 1.+index*.01, 1., .5)
            self.assertLessEqual(np.linalg.norm(output[:3]-previous[:3]), .01+1e-12)
            self.assertLessEqual(np.linalg.norm(output[3:]-previous[3:]), .02+1e-12)
            previous = output
        np.testing.assert_array_equal(controller.update(target, actual, 1.1, 1., 0.), np.zeros(6))

    def test_reset_and_large_timing_gap_clear_integral_estimation_and_derivative(self):
        controller = self.controller()
        a, b = pose(), pose((.002, 0., 0.), (0., 0., .004))
        controller.update(a, a, 1., 1., 1.)
        controller.update(b, b, 1.02, 1.02, 1.)
        controller._integral[:] = 1.
        controller.reset()
        np.testing.assert_array_equal(controller.update(b, b, 2., 2., 1.), np.zeros(6))
        controller.update(pose((.004, 0., 0.)), b, 2.02, 2.02, 1.)
        np.testing.assert_array_equal(controller.update(b, b, 2.5, 2.5, 1.), np.zeros(6))
        for state in (controller._target_velocity, controller._integral, controller._derivative):
            np.testing.assert_array_equal(state, np.zeros(6))

    def test_bad_poses_clocks_and_scale_reject_before_producing_commands(self):
        for target in ([0.]*6, [0.]*7, [math.nan]*7, [math.inf]*7):
            with self.subTest(target=target), self.assertRaises(ValueError):
                self.controller().update(target, pose(), 1., 1., 1.)
        for now, target_time, scale in ((math.nan, 1., 1.), (1., 1.01, 1.), (1., -1., 1.),
                                         (1., 1., math.inf), (1., 1., -.1), (1., 1., 1.01),
                                         (1., 1., True)):
            with self.subTest(now=now, target_time=target_time, scale=scale), self.assertRaises(ValueError):
                self.controller().update(pose(), pose(), now, target_time, scale)
        controller = self.controller()
        controller.update(pose(), pose(), 1., 1., 1.)
        for now, target_time, target in ((1., 1., pose()), (1.01, .99, pose()),
                                        (1.01, 1., pose((.001, 0., 0.)))):
            with self.subTest(now=now, target_time=target_time), self.assertRaises(ValueError):
                controller.update(target, pose(), now, target_time, 1.)

    def test_input_poses_and_supplied_configuration_are_never_mutated(self):
        config = load_control_config(CONFIG); before = deepcopy(config)
        target, actual = pose((.01, 0., 0.)), pose()
        target[3:] *= 2.; target_before, actual_before = target.copy(), actual.copy()
        controller = CartesianController(config)
        controller.update(target, actual, 1., 1., .5)
        self.assertEqual(config, before)
        np.testing.assert_array_equal(target, target_before)
        np.testing.assert_array_equal(actual, actual_before)

    def test_configuration_defaults_and_invalid_independent_parameters(self):
        config = load_control_config(CONFIG)
        self.assertEqual(set(config['gains']), set(ALGORITHMS))
        self.assertEqual(config['gains']['p']['position_gain'], 2.)
        self.assertEqual(config['gains']['feedforward_p']['position_gain'], 4.)
        for update in ({'algorithm': 'unknown'}, {'linear_speed': 0.}, {'angular_speed': math.inf},
                       {'linear_acceleration': -1.}, {'feedforward_validity': .11},
                       {'execution_feedback_timeout': 0.}, {'execution_command_timeout': .01},
                       {'antiwindup_tracking_time_constant': 0.},
                       {'antiwindup_mode': 'invalid'},
                       {'feedforward_hold_time': .1}, {'min_dt': .2}, {'typo': 1.},
                       {'gains': {'p': {'position_gain': True}}}, {'gains': {'missing': {}}}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                validate_control_config(update)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
            yaml.safe_dump({'algorithm': 'pd', 'linear_acceleration': .4}, stream)
            stream.flush()
            loaded = load_control_config(stream.name)
            self.assertEqual(loaded['algorithm'], 'pd')
            self.assertEqual(loaded['linear_acceleration'], .4)


if __name__ == '__main__':
    unittest.main()
