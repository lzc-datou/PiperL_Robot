"""Two gamepad modes and velocity-intent history; pure offline mathematics."""
import math
import unittest

from scripts.tools.gamepad_control.core import (
    ControlSelection, ControlState, InputFrame, VelocityIntentFilter,
    input_is_neutral, map_single_axis_velocity, map_velocity,
)


def frame(*, axes=(0.,)*4, dpad=(0,)*4, lb=False, b=False, t=10., sequence=1):
    return InputFrame('offline-gamepad', 1, sequence, t, tuple(axes), tuple(dpad), lb, b)


class GamepadModeMappingTests(unittest.TestCase):
    def test_default_selection_and_speed_storage_are_independent(self):
        first, second = ControlSelection(), ControlSelection()
        self.assertEqual(first.control_mode, 'single_axis')
        self.assertEqual(first.joint_index, 1)
        self.assertEqual(first.revision, 0)
        self.assertEqual(first.speed_scales, {'single_axis': .20, 'cartesian': .20})
        first.speed_scales['single_axis'] = .35
        self.assertEqual(first.speed_scales['cartesian'], .20)
        self.assertEqual(second.speed_scales['single_axis'], .20)

    def test_selection_revision_and_speed_follow_the_confirmed_mode(self):
        selection = ControlSelection()
        selection.set_speed(.35)
        self.assertEqual(selection.speed_scale, .35)
        self.assertTrue(selection.select('cartesian', 1))
        self.assertEqual(selection.revision, 1)
        self.assertEqual(selection.speed_scale, .20)
        selection.set_speed(.5)
        self.assertFalse(selection.select('cartesian', 1))
        self.assertEqual(selection.revision, 1)
        self.assertTrue(selection.select('single_axis', 6))
        self.assertEqual(selection.revision, 2)
        self.assertEqual(selection.speed_scale, .35)
        self.assertEqual(selection.speed_scales['cartesian'], .5)

    def test_invalid_selection_cannot_mutate_or_unlock_anything(self):
        selection = ControlSelection()
        for mode, joint in (('joint', 1), ('', 1), ('single_axis', 0), ('cartesian', 7),
                            ('single_axis', True), ('single_axis', 1.5)):
            with self.subTest(mode=mode, joint=joint), self.assertRaises((TypeError, ValueError)):
                selection.select(mode, joint)
            self.assertEqual((selection.control_mode, selection.joint_index, selection.revision),
                             ('single_axis', 1, 0))
        for speed in (.049, 1.01, math.nan, math.inf):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                selection.set_speed(speed)
            self.assertEqual(selection.speed_scale, .20)

    def test_navigation_neutral_requires_released_lb_b_and_centered_sticks(self):
        self.assertTrue(input_is_neutral(frame()))
        for dpad in ((0, 0, 1, 0), (0, 0, 0, 1)):
            with self.subTest(dpad=dpad):
                self.assertFalse(input_is_neutral(frame(dpad=dpad)))
                self.assertTrue(input_is_neutral(frame(dpad=dpad), allow_navigation=True))
        for value in (frame(lb=True), frame(b=True), frame(axes=(0., .081, 0., 0.)),
                      frame(dpad=(1, 0, 0, 0)), frame(dpad=(0, 1, 0, 0))):
            with self.subTest(value=value):
                self.assertFalse(input_is_neutral(value, allow_navigation=True))
                self.assertFalse(ControlState.neutral(value))

    def test_all_twelve_joint_directions_have_only_the_selected_joint_nonzero(self):
        for joint in range(1, 7):
            for sign in (-1., 1.):
                with self.subTest(joint=joint, sign=sign):
                    output = map_single_axis_velocity(frame(axes=(0., -sign, 0., 0.)), joint)
                    expected = [0.]*6
                    expected[joint-1] = sign*.30
                    for index, value in enumerate(output):
                        if index == joint-1:
                            self.assertAlmostEqual(value, expected[index])
                        else:
                            self.assertEqual(value, 0.)

    def test_single_axis_ignores_other_motion_controls(self):
        active = frame(axes=(1., -.7, -1., 1.), dpad=(1, 1, 1, 1))
        clean = frame(axes=(0., -.7, 0., 0.))
        self.assertEqual(map_single_axis_velocity(active, 4), map_single_axis_velocity(clean, 4))
        ignored_only = frame(axes=(1., 0., -1., 1.), dpad=(1, 0, 1, 0))
        self.assertEqual(map_single_axis_velocity(ignored_only, 2), (0.,)*6)

    def test_single_axis_deadzone_and_operator_scale_apply_once(self):
        self.assertEqual(map_single_axis_velocity(frame(axes=(0., .08, 0., 0.)), 3), (0.,)*6)
        self.assertAlmostEqual(
            map_single_axis_velocity(frame(axes=(0., -1., 0., 0.)), 3,
                                     speed_scale=.4, joint_max=.5)[2], .2)
        small = map_single_axis_velocity(frame(axes=(0., -.081, 0., 0.)), 3)[2]
        self.assertGreater(small, 0.)
        self.assertLess(small, .001)

    def test_single_axis_rejects_invalid_joint_and_input(self):
        for joint in (0, 7, -1, 1.5, True, '1'):
            with self.subTest(joint=joint), self.assertRaises((ValueError, TypeError)):
                map_single_axis_velocity(frame(), joint)
        for value in (math.nan, math.inf, -math.inf, 1.1):
            with self.subTest(value=value), self.assertRaises(ValueError):
                map_single_axis_velocity(frame(axes=(0., value, 0., 0.)), 1)
        for scale in (0., .049, 1.01, math.nan, math.inf):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                map_single_axis_velocity(frame(), 1, speed_scale=scale)

    def test_all_twelve_cartesian_directions_keep_the_fixed_base_mapping(self):
        cases = [
            (0, 1, (0., -1., 0., 0.), (0, 0, 0, 0), .06),
            (0, -1, (0., 1., 0., 0.), (0, 0, 0, 0), .06),
            (1, 1, (-1., 0., 0., 0.), (0, 0, 0, 0), .06),
            (1, -1, (1., 0., 0., 0.), (0, 0, 0, 0), .06),
            (2, 1, (0., 0., 0., -1.), (0, 0, 0, 0), .06),
            (2, -1, (0., 0., 0., 1.), (0, 0, 0, 0), .06),
            (3, 1, (0.,)*4, (0, 0, 1, 0), .105),
            (3, -1, (0.,)*4, (0, 0, 0, 1), .105),
            (4, 1, (0.,)*4, (1, 0, 0, 0), .105),
            (4, -1, (0.,)*4, (0, 1, 0, 0), .105),
            (5, 1, (0., 0., -1., 0.), (0, 0, 0, 0), .30),
            (5, -1, (0., 0., 1., 0.), (0, 0, 0, 0), .30),
        ]
        for axis, sign, axes, dpad, limit in cases:
            with self.subTest(axis=axis, sign=sign):
                result = map_velocity(frame(axes=axes, dpad=dpad))
                for index, value in enumerate(result):
                    self.assertAlmostEqual(value, sign*limit if index == axis else 0.)

    def test_new_permit_discards_a_previous_neutral_dwell(self):
        state = ControlState()
        state.start()
        state.update(frame(t=10., sequence=1))
        state.update(frame(t=10.2, sequence=2))
        self.assertTrue(state.neutral_seen)
        state.start()
        state.update(frame(t=20., sequence=3))
        self.assertFalse(state.neutral_seen)
        state.update(frame(t=20.05, sequence=4))
        self.assertFalse(state.neutral_seen)
        state.update(frame(t=20.16, sequence=5))
        self.assertTrue(state.neutral_seen)


class GamepadVelocityFilterTests(unittest.TestCase):
    def test_first_input_is_zero_and_response_uses_actual_elapsed_time(self):
        smoothing = VelocityIntentFilter(.04)
        target = (.3, -.2, .1, 1., -.8, .4)
        self.assertEqual(smoothing.update(target, 10.), (0.,)*6)
        result = smoothing.update(target, 10.04)
        for actual, wanted in zip(result, target):
            self.assertAlmostEqual(actual, wanted*(1.-math.exp(-1.)), places=12)

    def test_constant_intent_has_no_overshoot_or_steady_oscillation(self):
        smoothing = VelocityIntentFilter(.04)
        target = (.3, 0., 0., 0., 0., 0.)
        previous = smoothing.update(target, 1.)[0]
        for index in range(1, 101):
            value = smoothing.update(target, 1.+index*.01)[0]
            self.assertGreaterEqual(value, previous)
            self.assertLessEqual(value, .3)
            previous = value
        self.assertAlmostEqual(previous, .3, places=10)

    def test_equivalent_elapsed_time_gives_the_same_constant_input_response(self):
        regular, uneven = VelocityIntentFilter(.04), VelocityIntentFilter(.04)
        target = (.3, 0., 0., 0., 0., 0.)
        regular.update(target, 1.)
        uneven.update(target, 1.)
        for instant in (1.01, 1.02, 1.03, 1.04):
            regular.update(target, instant)
        for instant in (1.005, 1.017, 1.04):
            uneven.update(target, instant)
        self.assertAlmostEqual(regular.value[0], uneven.value[0], places=12)

    def test_reset_removes_old_axis_and_reverse_direction_history(self):
        smoothing = VelocityIntentFilter(.04)
        smoothing.update((.3, 0., 0., 0., 0., 0.), 1.)
        smoothing.update((.3, 0., 0., 0., 0., 0.), 1.04)
        smoothing.reset()
        new_axis = (0., -.3, 0., 0., 0., 0.)
        self.assertEqual(smoothing.update(new_axis, 2.), (0.,)*6)
        result = smoothing.update(new_axis, 2.01)
        self.assertEqual(result[0], 0.)
        self.assertLess(result[1], 0.)
        self.assertEqual(result[2:], (0.,)*4)

    def test_clock_reversal_duplicate_time_and_long_gap_clear_history(self):
        for instant in (1.01, 1., 1.111):
            with self.subTest(instant=instant):
                smoothing = VelocityIntentFilter(.04)
                smoothing.update((.3,)*6, 1.)
                smoothing.update((.3,)*6, 1.01)
                with self.assertRaises(ValueError):
                    smoothing.update((.3,)*6, instant)
                self.assertEqual(smoothing.value, (0.,)*6)
                self.assertIsNone(smoothing.last_time)

    def test_bad_targets_cannot_produce_a_filtered_command(self):
        for target in ((0.,)*5, (0.,)*7, (math.nan, 0., 0., 0., 0., 0.)):
            with self.subTest(target=target), self.assertRaises(ValueError):
                VelocityIntentFilter(.04).update(target, 1.)
        with self.assertRaises(ValueError):
            VelocityIntentFilter(.04).update((0.,)*6, math.nan)


if __name__ == '__main__':
    unittest.main()
