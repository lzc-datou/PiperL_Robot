"""Bounded velocity kinematics only; no ROS initialization or arm commands."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np
import yaml

from scripts.task.servo_tracking.core import (RobotModel, damped_joint_velocity,
    joint_speed_limits, load_config)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'scripts/task/servo_tracking/config/config.yaml'
URDF = ROOT/'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


class DampedVelocityTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(CONFIG)
        self.model = RobotModel(URDF.read_text(), tip='link6')
        self.q = np.asarray(self.config['initial_joints'])

    def test_exact_and_near_singular_poses_produce_finite_bounded_velocity(self):
        for q in (np.zeros(6), [0., 1.4, -1.2, .3, 0., 0.],
                  [0., 1.4, -1.2, .3, 1e-12, 0.]):
            with self.subTest(q=q):
                velocity = damped_joint_velocity(q, [.1, -.1, .1, .5, -.5, .5],
                                                  self.model, self.config, .5)
                self.assertTrue(np.all(np.isfinite(velocity)))
                self.assertTrue(np.all(np.abs(velocity) <= joint_speed_limits(self.config, .5)+1e-12))
                self.assertGreater(np.linalg.norm(velocity), 0.)

    def test_fixed_damping_matches_weighted_least_squares_without_condition_gate(self):
        model = NS(jacobian=Mock(return_value=np.diag([.5, .4, .3, 1., 1., 0.])),
                   condition_number=Mock(side_effect=AssertionError('must not inspect κ')))
        twist = np.array([.02, -.03, .01, .04, -.02, .05])
        config = dict(self.config, joint_speed=10., dls_damping=.05, dls_rotation_weight=.3)
        weights = np.diag([1., 1., 1., .3, .3, .3])
        a = weights @ model.jacobian.return_value
        expected = np.linalg.solve(a.T @ a + .05**2*np.eye(6), a.T @ weights @ twist)
        actual = damped_joint_velocity(self.q, twist, model, config)
        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
        self.assertEqual(actual[-1], 0.)  # Unavailable direction degrades without NaN/Inf.
        model.condition_number.assert_not_called()

    def test_axis_limits_use_one_common_factor_and_operator_scale(self):
        model = NS(jacobian=lambda q: np.eye(6))
        config = dict(self.config, joint_speed=10.,
                      joint_velocity_limits=[.2, .7, .4, 1.1, .9, 1.3])
        twist = np.array([2., -3., 4., -2., 1., .5])
        weights = np.array([1., 1., 1., .25, .25, .25])
        unconstrained = weights**2/(weights**2+.02**2)*twist
        for scale in (1., .5, .1):
            with self.subTest(scale=scale):
                limits = joint_speed_limits(config, scale)
                expected = unconstrained / max(1., np.max(np.abs(unconstrained)/limits))
                actual = damped_joint_velocity(self.q, twist, model, config, scale)
                np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
                self.assertTrue(np.all(np.abs(actual) <= limits+1e-12))
                self.assertAlmostEqual(np.max(np.abs(actual)/limits), 1.)

    def test_zero_twist_is_zero_and_no_inputs_are_mutated(self):
        q, twist = self.q.copy(), np.zeros(6)
        np.testing.assert_array_equal(damped_joint_velocity(q, twist, self.model, self.config), np.zeros(6))
        np.testing.assert_array_equal(q, self.q)
        np.testing.assert_array_equal(twist, np.zeros(6))
        desired = np.array([.01, -.02, .01, .03, -.02, .01])
        achieved = self.model.jacobian(q) @ damped_joint_velocity(q, desired, self.model, self.config)
        self.assertLess(np.linalg.norm(achieved-desired), .1*np.linalg.norm(desired))

    def test_invalid_numeric_inputs_and_damping_are_rejected(self):
        for config in (dict(self.config, dls_damping=0.), dict(self.config, dls_damping=True),
                       dict(self.config, dls_damping=float('nan')),
                       dict(self.config, dls_rotation_weight=-.1)):
            with self.subTest(config=config), self.assertRaises(ValueError):
                damped_joint_velocity(self.q, np.ones(6), self.model, config)
        for q, twist, scale in (([0.]*5, [0.]*6, 1.),
                                (self.q, [float('inf')]*6, 1.),
                                (self.q, [0.]*6, 0.)):
            with self.assertRaises(ValueError):
                damped_joint_velocity(q, twist, self.model, self.config, scale)
        with self.assertRaises(ValueError):
            damped_joint_velocity(self.q, np.zeros(6),
                                  NS(jacobian=lambda q: np.full((6, 6), np.nan)), self.config)

    def test_old_task_configs_get_defaults_and_invalid_new_parameters_fail(self):
        data = yaml.safe_load(CONFIG.read_text())
        data.pop('dls_damping'); data.pop('dls_rotation_weight')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'task.yaml'
            path.write_text(yaml.safe_dump(data))
            config = load_config(path)
            self.assertEqual(config['dls_damping'], .02)
            self.assertEqual(config['dls_rotation_weight'], .25)
            for key in ('dls_damping', 'dls_rotation_weight'):
                for value in (0., -1., True, float('inf'), float('nan'), '0.02'):
                    with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                        path.write_text(yaml.safe_dump(dict(data, **{key: value})))
                        load_config(path)


try:
    from scripts.task.servo_tracking.guard import Guard
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use the ROS wrapper; no node or hardware is initialized')
class GuardConditionLoggingTests(unittest.TestCase):
    def test_stop_diagnostic_keeps_raw_joints_without_computing_condition(self):
        model = NS(condition_number=Mock(side_effect=AssertionError('κ is disabled')))
        raw = np.array([0., -.0002, .0001, 0., 0., 0.])
        state = NS(config={'feedback_timeout': .25}, feedback=NS(get=lambda timeout: raw), model=model)
        result = json.loads(Guard.condition_snapshot(state))
        self.assertEqual(result, {'joints': raw.tolist()})
        model.condition_number.assert_not_called()


if __name__ == '__main__':
    unittest.main()
