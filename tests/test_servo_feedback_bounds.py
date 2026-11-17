"""Feedback and planning references must not relax commands or alter raw data."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yaml

from scripts.task.servo_tracking.core import (
    JOINTS, MAX_ZERO_FEEDBACK_TOLERANCE, RobotModel, load_config,
    validate_joint_plan, validate_trajectory,
)


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'
CONFIG = ROOT / 'scripts/task/servo_tracking/config/config.yaml'


class FeedbackBoundsTests(unittest.TestCase):
    def setUp(self):
        self.model = RobotModel(URDF.read_text(), tip='link6')
        self.tolerance = MAX_ZERO_FEEDBACK_TOLERANCE
        self.safe = np.array([0., 1.4, -1.2, .3, 1., 0.])

    def test_real_log_samples_are_preserved_without_clamping(self):
        # Read-only fixtures from the 19:24 and 19:25 real-session logs.
        samples = [
            [0., -0.00047123889803846896, -0.00034906585039886593, 0., 0., 0.],
            [0., -0.0012042771838760874, 0.0002617993877991494, 0., 0., 0.],
        ]
        for values in samples:
            raw = np.array(values)
            before = raw.copy()
            with self.subTest(values=values):
                checked = self.model.check_feedback(raw, self.tolerance)
                np.testing.assert_array_equal(checked, before)
                np.testing.assert_array_equal(raw, before)
                self.assertLess(checked[1], 0.)
                self.assertTrue(np.isfinite(self.model.pose(checked)).all())
                with self.assertRaisesRegex(ValueError, '关节角超出模型限位.*joint2'):
                    self.model.check_joints(checked)

    def test_only_j2_lower_and_j3_upper_zero_edges_receive_tolerance(self):
        for joint, sign in ((1, -1.), (2, 1.)):
            raw = np.zeros(6); raw[joint] = sign * self.tolerance
            np.testing.assert_array_equal(self.model.check_feedback(raw, self.tolerance), raw)
            with self.assertRaises(ValueError):
                self.model.check_feedback(raw)
            raw[joint] = np.nextafter(raw[joint], sign * math.inf)
            with self.assertRaises(ValueError):
                self.model.check_feedback(raw, self.tolerance)

    def test_planning_reference_normalizes_only_allowed_zero_edges_and_preserves_input(self):
        samples = [
            [0., -.0003665191429188092, 0., 0., 0., 0.],  # Stable -0.021° in 20:46 log.
            [0., -.0012042771838760874, .0002617993877991494, 0., 0., 0.],
            [0., -.00047123889803846896, -.00034906585039886593, 0., 0., 0.],
        ]
        for values in samples:
            raw = np.asarray(values)
            before = raw.copy()
            expected = before.copy()
            expected[1] = self.model.lower[1]
            if expected[2] > self.model.upper[2]:
                expected[2] = self.model.upper[2]
            with self.subTest(values=values):
                reference = self.model.planning_joints(raw, self.tolerance)
                np.testing.assert_array_equal(reference, expected)
                np.testing.assert_array_equal(raw, before)
                np.testing.assert_array_equal(self.model.check_joints(reference), reference)
                with self.assertRaisesRegex(ValueError, '模型限位'):
                    self.model.check_joints(raw)
                reference[0] = 1.
                np.testing.assert_array_equal(raw, before)

    def test_legal_planning_reference_returns_an_independent_unmodified_copy(self):
        before = self.safe.copy()
        reference = self.model.planning_joints(self.safe, self.tolerance)
        np.testing.assert_array_equal(reference, before)
        reference[0] = 1.
        np.testing.assert_array_equal(self.safe, before)

    def test_planning_reference_cannot_normalize_other_bounds_or_excess_zero_error(self):
        for joint in range(6):
            for side in ('lower', 'upper'):
                raw = self.safe.copy()
                outside = self.tolerance * 1.01 if (joint, side) in ((1, 'lower'), (2, 'upper')) else 1e-6
                raw[joint] = (self.model.lower[joint] - outside if side == 'lower'
                              else self.model.upper[joint] + outside)
                before = raw.copy()
                with self.subTest(joint=joint + 1, side=side), self.assertRaisesRegex(ValueError, '模型限位'):
                    self.model.planning_joints(raw, self.tolerance)
                np.testing.assert_array_equal(raw, before)

    def test_planning_reference_zero_tolerance_remains_strict(self):
        raw = [0., -.0003665191429188092, 0., 0., 0., 0.]
        with self.assertRaisesRegex(ValueError, '模型限位.*joint2'):
            self.model.planning_joints(raw)

    def test_all_other_axes_and_bound_directions_remain_strict(self):
        for joint in range(6):
            for side in ('lower', 'upper'):
                if (joint, side) in ((1, 'lower'), (2, 'upper')):
                    continue
                raw = self.safe.copy()
                raw[joint] = (self.model.lower[joint] - 1e-6 if side == 'lower'
                              else self.model.upper[joint] + 1e-6)
                with self.subTest(joint=joint + 1, side=side), self.assertRaises(ValueError):
                    self.model.check_feedback(raw, self.tolerance)

    def test_exception_requires_the_model_bound_itself_to_be_zero(self):
        self.model.lower[1] = .01
        raw = self.safe.copy(); raw[1] = .01 - self.tolerance / 2
        with self.assertRaises(ValueError):
            self.model.check_feedback(raw, self.tolerance)
        with self.assertRaises(ValueError):
            self.model.planning_joints(raw, self.tolerance)
        self.model.upper[2] = -.01
        raw = self.safe.copy(); raw[2] = -.01 + self.tolerance / 2
        with self.assertRaises(ValueError):
            self.model.check_feedback(raw, self.tolerance)
        with self.assertRaises(ValueError):
            self.model.planning_joints(raw, self.tolerance)

    def test_structured_hard_bound_diagnostics_are_exact_and_json_serializable(self):
        raw = [0., -.001, .0002, 0., 0., 0.]
        violations = self.model.joint_bound_violations(raw)
        self.assertEqual([item['joint'] for item in violations], ['joint2', 'joint3'])
        self.assertEqual([item['side'] for item in violations], ['lower', 'upper'])
        self.assertEqual([item['value'] for item in violations], [-.001, .0002])
        self.assertEqual([item['delta'] for item in violations], [.001, .0002])
        for item in violations:
            self.assertEqual(set(item), {'joint', 'value', 'lower', 'upper', 'delta', 'side'})
            self.assertGreater(item['delta'], 0.)
            for key in ('value', 'lower', 'upper', 'delta'):
                self.assertIs(type(item[key]), float)
        self.assertEqual(json.loads(json.dumps(violations, allow_nan=False)), violations)
        with self.assertRaises(ValueError) as raised:
            self.model.check_joints(raw)
        message = str(raised.exception)
        self.assertTrue(message.startswith('关节角超出模型限位'))
        for token in ('joint2', 'joint3', '°', '越界'):
            self.assertIn(token, message)

    def test_target_and_both_trajectory_validators_remain_strict(self):
        raw = np.zeros(6); raw[1] = -.0005; raw[2] = .0002
        self.model.check_feedback(raw, self.tolerance)
        with self.assertRaises(ValueError):
            self.model.check_joints(raw, margin=.015)
        config = load_config(CONFIG)
        point = NS(positions=raw.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6,
                   time_from_start=NS(sec=0, nanosec=20_000_000))
        servo = NS(joint_names=JOINTS, points=[point], header=NS(stamp=NS(sec=100, nanosec=0)))
        with self.assertRaisesRegex(ValueError, '模型限位'):
            validate_trajectory(servo, raw, self.model, config, 100.01)
        plan = NS(joint_names=JOINTS, points=[
            NS(positions=self.safe.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6,
               time_from_start=NS(sec=0, nanosec=0)),
            NS(positions=raw.tolist(), velocities=[0.] * 6, accelerations=[0.] * 6,
               time_from_start=NS(sec=10, nanosec=0))])
        with self.assertRaisesRegex(ValueError, '模型限位'):
            validate_joint_plan(plan, self.safe, raw, self.model, config)
        with self.assertRaisesRegex(ValueError, '模型限位'):
            validate_joint_plan(plan, raw, self.safe, self.model, config)

    def test_feedback_api_rejects_invalid_data_and_tolerance(self):
        for tolerance in (-1., math.nan, math.inf, True, self.tolerance + 1e-12):
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                self.model.check_feedback(self.safe, tolerance)
            with self.subTest(planning_tolerance=tolerance), self.assertRaises(ValueError):
                self.model.planning_joints(self.safe, tolerance)
        for raw in ([0.] * 5, [math.nan] * 6, [math.inf] * 6):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                self.model.check_feedback(raw, self.tolerance)
            with self.subTest(planning_raw=raw), self.assertRaises(ValueError):
                self.model.planning_joints(raw, self.tolerance)
        self.assertEqual(self.model.joint_bound_violations(self.safe), [])


class FeedbackToleranceConfigTests(unittest.TestCase):
    def config(self, updates):
        data = yaml.safe_load(CONFIG.read_text())
        data.pop('zero_feedback_tolerance', None)
        data.update(updates)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
            yaml.safe_dump(data, stream)
            stream.flush()
            return load_config(stream.name)

    def test_missing_zero_and_maximum_tolerance_are_supported(self):
        self.assertEqual(self.config({})['zero_feedback_tolerance'], math.radians(.1))
        self.assertEqual(self.config({'zero_feedback_tolerance': 0})['zero_feedback_tolerance'], 0.)
        self.assertEqual(self.config({'zero_feedback_tolerance': MAX_ZERO_FEEDBACK_TOLERANCE})[
            'zero_feedback_tolerance'], MAX_ZERO_FEEDBACK_TOLERANCE)

    def test_invalid_tolerance_configuration_is_rejected(self):
        for value in (-1., math.nan, math.inf, True, '0.001', None,
                      MAX_ZERO_FEEDBACK_TOLERANCE + 1e-12):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config({'zero_feedback_tolerance': value})


if __name__ == '__main__':
    unittest.main()
