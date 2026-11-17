"""Validate planned JTC splines without ROS initialization or hardware access."""
import copy
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yaml

from scripts.task.servo_tracking.core import (JOINTS, HARDWARE_JOINT_SPEED_LIMITS,
                                             RobotModel, load_config, validate_joint_plan)


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'
CONFIG = ROOT / 'scripts/task/servo_tracking/config/config.yaml'


def point(t, positions, velocities=None, accelerations=None):
    seconds = int(t)
    return NS(positions=list(positions), velocities=list(velocities if velocities is not None else [0.] * 6),
              accelerations=list(accelerations if accelerations is not None else [0.] * 6),
              time_from_start=NS(sec=seconds, nanosec=round((t - seconds) * 1e9)))


class JointPlanTests(unittest.TestCase):
    def setUp(self):
        self.model = RobotModel(URDF.read_text(), tip='link6')
        self.actual = np.array([0., 1.4, -1.2, .3, 1., 0.])
        self.goal = self.actual + [.02, .01, -.01, 0., 0., 0.]
        self.config = dict(joint_speed=.3, joint_acceleration=.5, joint_margin=.015)

    def plan(self, seconds=2.):
        return NS(joint_names=JOINTS.copy(), points=[point(0., self.actual), point(seconds, self.goal)])

    def check(self, trajectory=None, **kwargs):
        return validate_joint_plan(trajectory if trajectory is not None else self.plan(),
                                   self.actual, self.goal, self.model, self.config, **kwargs)

    def test_valid_plan_duration_and_no_mutation(self):
        trajectory = self.plan()
        snapshot = copy.deepcopy(trajectory)
        self.assertEqual(self.check(trajectory), 2.)
        self.assertEqual(trajectory, snapshot)

    def test_joint_order_is_resolved_by_name_without_reordering_input(self):
        trajectory = self.plan()
        order = [5, 2, 0, 4, 1, 3]
        trajectory.joint_names = [JOINTS[i] for i in order]
        for knot in trajectory.points:
            for field in ('positions', 'velocities', 'accelerations'):
                values = getattr(knot, field)
                setattr(knot, field, [values[i] for i in order])
        snapshot = copy.deepcopy(trajectory)
        self.assertEqual(self.check(trajectory), 2.)
        self.assertEqual(trajectory, snapshot)

    def test_zero_start_can_leave_hard_boundaries_inward(self):
        actual = np.zeros(6)
        goal = np.array([0., .1, -.1, 0., .1, 0.])
        trajectory = NS(joint_names=JOINTS.copy(), points=[point(0., actual), point(3., goal)])
        self.assertEqual(validate_joint_plan(trajectory, actual, goal, self.model, self.config), 3.)

    def test_empty_partial_duplicate_and_missing_derivatives_rejected(self):
        malformed = [NS(joint_names=JOINTS, points=[]),
                     NS(joint_names=JOINTS, points=[point(0., self.actual)]),
                     NS(joint_names=JOINTS[:-1], points=self.plan().points),
                     NS(joint_names=[JOINTS[0]] * 6, points=self.plan().points)]
        for field in ('positions', 'velocities', 'accelerations'):
            trajectory = self.plan()
            setattr(trajectory.points[1], field, [])
            malformed.append(trajectory)
        trajectory = self.plan()
        del trajectory.points[0].accelerations
        malformed.append(trajectory)
        for trajectory in malformed:
            with self.subTest(trajectory=trajectory), self.assertRaises(ValueError):
                self.check(trajectory)

    def test_nonfinite_values_rejected_in_every_vector(self):
        for field in ('positions', 'velocities', 'accelerations'):
            for value in (float('nan'), float('inf')):
                trajectory = self.plan()
                getattr(trajectory.points[1], field)[2] = value
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.check(trajectory)

    def test_stale_start_wrong_goal_and_endpoint_margin_rejected(self):
        trajectory = self.plan()
        trajectory.points[0].positions[0] += .006
        with self.assertRaisesRegex(ValueError, '起点'):
            self.check(trajectory)
        trajectory = self.plan()
        trajectory.points[-1].positions[0] += .004
        with self.assertRaisesRegex(ValueError, '终点'):
            self.check(trajectory)
        goal = self.actual.copy(); goal[1] = 0.
        trajectory = NS(joint_names=JOINTS, points=[point(0., self.actual), point(20., goal)])
        with self.assertRaisesRegex(ValueError, '限位'):
            validate_joint_plan(trajectory, self.actual, goal, self.model, self.config)

    def test_bad_intermediate_knot_cannot_hide_behind_valid_endpoint(self):
        trajectory = self.plan(10.)
        intermediate = self.actual.copy(); intermediate[0] = self.model.upper[0] + .01
        trajectory.points.insert(1, point(5., intermediate))
        with self.assertRaisesRegex(ValueError, '限位'):
            self.check(trajectory)

    def test_time_must_start_at_zero_increase_and_stay_bounded(self):
        for first, last in ((.01, 2.), (0., 0.), (0., 121.)):
            trajectory = NS(joint_names=JOINTS, points=[point(first, self.actual), point(last, self.goal)])
            with self.subTest(first=first, last=last), self.assertRaises(ValueError):
                self.check(trajectory)
        for sec, nanosec in ((-1, 0), (0, -1), (0, 1_000_000_000), (0., 0)):
            trajectory = self.plan()
            trajectory.points[0].time_from_start = NS(sec=sec, nanosec=nanosec)
            with self.subTest(sec=sec, nanosec=nanosec), self.assertRaises(ValueError):
                self.check(trajectory)
        trajectory = self.plan()
        trajectory.points.insert(1, point(2., self.actual))
        with self.assertRaises(ValueError):
            self.check(trajectory)
        trajectory = self.plan()
        trajectory.points[0].time_from_start.nanosec = 1
        self.assertEqual(self.check(trajectory), 2.)
        trajectory = self.plan()
        trajectory.points[0].time_from_start.nanosec = 2
        with self.assertRaises(ValueError):
            self.check(trajectory)
        self.config['plan_max_duration'] = 1.
        with self.assertRaises(ValueError):
            self.check()

    def test_segment_implied_speed_and_knot_limits(self):
        with self.assertRaisesRegex(ValueError, '隐含速度'):
            self.check(self.plan(.01))
        for field, value, message in (('velocities', .31, '速度'), ('accelerations', .51, '加速度')):
            trajectory = self.plan()
            getattr(trajectory.points[1], field)[0] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, message):
                self.check(trajectory)

    def test_requires_stopped_endpoint_velocities_but_allows_acceleration(self):
        trajectory = self.plan()
        trajectory.points[0].velocities[0] = .01
        with self.assertRaisesRegex(ValueError, '首速度'):
            self.check(trajectory)
        trajectory = self.plan()
        trajectory.points[0].accelerations[0] = .01
        trajectory.points[-1].accelerations[0] = -.01
        self.assertEqual(self.check(trajectory), 2.)

    def test_totg_start_velocity_allowance_scales_with_acceleration_cap(self):
        trajectory = self.plan()
        trajectory.points[0].velocities[0] = self.config['joint_acceleration'] * .001 * .5 + 1e-6
        self.assertEqual(self.check(trajectory, speed_scale=.5), 2.)
        trajectory.points[0].velocities[0] += 1e-7
        with self.assertRaisesRegex(ValueError, '首速度'):
            self.check(trajectory, speed_scale=.5)
        self.assertEqual(self.check(trajectory, speed_scale=1.), 2.)

    def test_totg_start_allowance_does_not_relax_final_velocity(self):
        trajectory = self.plan()
        trajectory.points[0].velocities[0] = .0002
        trajectory.points[-1].velocities[0] = 2e-6
        with self.assertRaisesRegex(ValueError, '末速度'):
            self.check(trajectory)

    def test_quintic_position_overshoot_between_legal_knots_rejected(self):
        actual = self.actual.copy(); actual[0] = self.model.upper[0] - .02
        acceleration = [0.1, 0., 0., 0., 0., 0.]
        trajectory = NS(joint_names=JOINTS, points=[point(0., actual, accelerations=acceleration),
                                                     point(4., actual, accelerations=acceleration)])
        # These legal endpoint accelerations produce +0.05 rad at the midpoint.
        with self.assertRaisesRegex(ValueError, '多项式位置'):
            validate_joint_plan(trajectory, actual, actual, self.model, self.config)

    def test_quintic_velocity_peak_between_stationary_knots_rejected(self):
        goal = self.actual.copy(); goal[0] += .2
        trajectory = NS(joint_names=JOINTS, points=[point(0., self.actual), point(1., goal)])
        config = dict(self.config, joint_acceleration=2.)
        # Average speed is .2, while the quintic reaches .375 rad/s.
        with self.assertRaisesRegex(ValueError, '多项式速度'):
            validate_joint_plan(trajectory, self.actual, goal, self.model, config)

    def test_quintic_acceleration_peak_between_zero_acceleration_knots_rejected(self):
        goal = self.actual.copy(); goal[0] += .1
        trajectory = NS(joint_names=JOINTS, points=[point(0., self.actual), point(2., goal)])
        config = dict(self.config, joint_acceleration=.1)
        with self.assertRaisesRegex(ValueError, '多项式加速度'):
            validate_joint_plan(trajectory, self.actual, goal, self.model, config)

    def test_speed_scale_limits_both_velocity_and_acceleration(self):
        self.assertEqual(self.check(speed_scale=.5), 2.)
        with self.assertRaises(ValueError):
            self.check(speed_scale=.01)
        for value in (0., -1., 1.1, float('nan'), float('inf')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.check(speed_scale=value)

    def test_quintic_velocity_extrema_use_each_axis_ceiling(self):
        config = dict(self.config, joint_speed=10., joint_acceleration=20.)
        for joint, peak, allowed in ((0, 3.3, False), (1, 3.3, True),
                                     (1, 3.8, False), (3, 3.8, True)):
            actual = self.actual.copy(); actual[joint] = -.8 if joint == 3 else .0
            goal = actual.copy(); goal[joint] += peak / 1.875
            trajectory = NS(joint_names=JOINTS, points=[point(0., actual), point(1., goal)])
            # A zero-derivative quintic peaks at 1.875 times its average speed.
            with self.subTest(joint=joint + 1, peak=peak):
                if allowed:
                    self.assertEqual(validate_joint_plan(trajectory, actual, goal, self.model, config), 1.)
                else:
                    with self.assertRaisesRegex(ValueError, '多项式速度'):
                        validate_joint_plan(trajectory, actual, goal, self.model, config)

    def test_reordered_knot_velocities_still_use_canonical_axis_limits(self):
        config = dict(self.config, joint_speed=10., joint_acceleration=2000.)
        for joint, allowed in ((0, False), (1, True)):
            velocity = [0.] * 6; velocity[joint] = 3.3
            trajectory = NS(joint_names=JOINTS[::-1], points=[
                point(0., self.actual), point(.01, self.actual, velocities=velocity),
                point(.02, self.actual)])
            for knot in trajectory.points:
                for field in ('positions', 'velocities', 'accelerations'):
                    setattr(knot, field, getattr(knot, field)[::-1])
            with self.subTest(joint=joint + 1):
                if allowed:
                    self.assertEqual(validate_joint_plan(trajectory, self.actual, self.actual,
                                                         self.model, config), .02)
                else:
                    with self.assertRaisesRegex(ValueError, '轨迹点速度'):
                        validate_joint_plan(trajectory, self.actual, self.actual, self.model, config)

    def test_implied_plan_speed_checks_axis_before_polynomial_extrema(self):
        config = dict(self.config, joint_speed=10., joint_acceleration=200.)
        for joint, message in ((0, '隐含速度'), (1, '多项式速度')):
            goal = self.actual.copy(); goal[joint] += .66
            trajectory = NS(joint_names=JOINTS, points=[point(0., self.actual), point(.2, goal)])
            with self.subTest(joint=joint + 1), self.assertRaisesRegex(ValueError, message):
                validate_joint_plan(trajectory, self.actual, goal, self.model, config)


class JointPlanConfigTests(unittest.TestCase):
    def config(self, updates):
        data = yaml.safe_load(CONFIG.read_text())
        for key in ('ready_joints', 'prepare_on_singular_start', 'planning_time',
                    'plan_max_duration', 'plan_settle_timeout', 'joint_velocity_limits',
                    'hold_position_tolerance', 'hold_orientation_tolerance',
                    'hold_joint_stability', 'hold_settle_time'):
            data.pop(key, None)
        data.update(updates)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
            yaml.safe_dump(data, stream)
            stream.flush()
            return load_config(stream.name)

    def test_old_configuration_receives_optional_defaults(self):
        config = self.config({})
        self.assertEqual(config['ready_joints'], [0., 1.4, -1.2, .3, 1., 0.])
        self.assertTrue(config['prepare_on_singular_start'])
        self.assertEqual(config['planning_time'], 5.)
        self.assertEqual(config['plan_max_duration'], 120.)
        self.assertEqual(config['plan_settle_timeout'], 5.)
        np.testing.assert_allclose(config['joint_velocity_limits'], HARDWARE_JOINT_SPEED_LIMITS)

    def test_optional_values_are_validated(self):
        for key in ('planning_time', 'plan_max_duration', 'plan_settle_timeout'):
            for value in (0., -1., float('nan'), float('inf'), True):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.config({key: value})
        for value in ('true', 1, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config({'prepare_on_singular_start': value})
        for value in ([0.] * 5, [math.nan] * 6):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config({'ready_joints': value})

    def test_hardware_speed_caps_reject_bad_shapes_values_and_excess(self):
        excessive = HARDWARE_JOINT_SPEED_LIMITS.tolist(); excessive[1] += .001
        for value in ([1.] * 5, [math.nan] * 6, [math.inf] * 6, [0.] * 6, [-1.] * 6, excessive):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config({'joint_velocity_limits': value})
        configured = [1., 2., 1., 3.5, 3.6, 3.7]
        self.assertEqual(self.config({'joint_velocity_limits': configured})['joint_velocity_limits'], configured)

    def test_hold_defaults_respect_tracking_tolerances_and_reject_invalid_values(self):
        config = self.config({})
        self.assertEqual(config['hold_position_tolerance'], .003)
        self.assertEqual(config['hold_orientation_tolerance'], .02)
        self.assertEqual(config['hold_joint_stability'], .003)
        self.assertEqual(config['hold_settle_time'], .3)
        config = self.config({'position_tolerance': .01, 'orientation_tolerance': .04})
        self.assertEqual(config['hold_position_tolerance'], .01)
        self.assertEqual(config['hold_orientation_tolerance'], .04)
        for key in ('hold_position_tolerance', 'hold_orientation_tolerance',
                    'hold_joint_stability', 'hold_settle_time'):
            for value in (0., -1., math.nan, math.inf, True):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.config({key: value})
        for kind, threshold in (('position', .01), ('orientation', .04)):
            updates = {f'{kind}_tolerance': threshold, f'hold_{kind}_tolerance': threshold / 2}
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, '保持位姿容差'):
                self.config(updates)


if __name__ == '__main__':
    unittest.main()
