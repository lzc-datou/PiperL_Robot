"""Directional joint braking bounds without ROS, SDK, or hardware access."""
import math
import unittest

from scripts.tools.gamepad_control.motion import limit_joint_velocity


class GamepadJointBoundaryTests(unittest.TestCase):
    lower = (-2.,)*6
    upper = (2.,)*6

    def limit(self, q, velocity, **kwargs):
        return limit_joint_velocity(q, velocity, self.lower, self.upper, 1.5, **kwargs)

    def test_interior_request_is_unchanged(self):
        velocity = (1.5, -.6, .2, 0., -.4, 1.)
        result, scale = self.limit((0.,)*6, velocity)
        self.assertEqual(tuple(result), velocity)
        self.assertEqual(scale, 1.)

    def test_upper_and_lower_bound_block_outward_but_keep_retreat(self):
        for axis in range(6):
            for sign in (-1., 1.):
                with self.subTest(axis=axis, boundary_sign=sign):
                    q, outward = [0.]*6, [0.]*6
                    q[axis] = sign*2.
                    outward[axis] = sign*.3
                    stopped, scale = self.limit(q, outward)
                    self.assertEqual(tuple(stopped), (0.,)*6)
                    self.assertEqual(scale, 0.)
                    inward = [-v for v in outward]
                    retreat, scale = self.limit(q, inward)
                    self.assertEqual(tuple(retreat), tuple(inward))
                    self.assertEqual(scale, 1.)

    def test_zero_component_at_a_bound_does_not_block_other_joints(self):
        q = (2., 0., 0., 0., 0., 0.)
        velocity = (0., .4, 0., 0., 0., 0.)
        result, scale = self.limit(q, velocity)
        self.assertEqual(tuple(result), velocity)
        self.assertEqual(scale, 1.)

    def test_deceleration_respects_latency_and_stopping_distance(self):
        margin, latency, acceleration = .015, .13, 1.5
        q = (1.95, 0., 0., 0., 0., 0.)
        result, scale = self.limit(q, (1.5, 0., 0., 0., 0., 0.), margin=margin, latency=latency)
        self.assertGreater(scale, 0.)
        self.assertLess(scale, 1.)
        speed = result[0]
        distance = self.upper[0]-q[0]-margin
        self.assertLessEqual(speed*latency+speed*speed/(2.*acceleration), distance+1e-12)
        self.assertAlmostEqual(speed*latency+speed*speed/(2.*acceleration), distance, places=10)

    def test_composite_request_is_scaled_as_one_vector(self):
        velocity = (1.5, -.7, .4, .2, -.1, .6)
        q = (1.95, 0., 0., 0., 0., 0.)
        result, scale = self.limit(q, velocity)
        self.assertGreater(scale, 0.)
        self.assertLess(scale, 1.)
        for actual, requested in zip(result, velocity):
            self.assertAlmostEqual(actual, requested*scale, places=12)

    def test_one_outward_component_can_restrict_the_entire_cartesian_request(self):
        result, scale = self.limit((2., 0., 0., 0., 0., 0.), (1., .5, 0., -.3, 0., 0.))
        self.assertEqual(tuple(result), (0.,)*6)
        self.assertEqual(scale, 0.)

    def test_no_input_has_no_history_or_artificial_motion(self):
        result, scale = self.limit((2., -2., 0., 0., 0., 0.), (0.,)*6)
        self.assertEqual(tuple(result), (0.,)*6)
        self.assertEqual(scale, 1.)

    def test_invalid_numbers_shapes_and_braking_parameters_fail_closed(self):
        for q, velocity in (((0.,)*5, (0.,)*6), ((0.,)*6, (0.,)*5),
                            ((math.nan, 0., 0., 0., 0., 0.), (0.,)*6),
                            ((0.,)*6, (math.inf, 0., 0., 0., 0., 0.))):
            with self.subTest(q=q, velocity=velocity), self.assertRaises(ValueError):
                self.limit(q, velocity)
        for acceleration in (0., -1., math.nan, math.inf):
            with self.subTest(acceleration=acceleration), self.assertRaises(ValueError):
                limit_joint_velocity((0.,)*6, (0.,)*6, self.lower, self.upper, acceleration)
        for kwargs in ({'latency': -.01}, {'latency': math.inf}, {'margin': -.01}, {'margin': math.nan}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.limit((0.,)*6, (0.,)*6, **kwargs)


if __name__ == '__main__':
    unittest.main()
