"""Synthetic telemetry only: no ROS, device or command publication."""
from types import SimpleNamespace as NS
import unittest

import numpy as np

from scripts.task.servo_tracking.execution_feedback import ExecutionObserver, STAGES
from scripts.task.servo_tracking.cartesian_control import CartesianController


class ExecutionObserverTests(unittest.TestCase):
    def setUp(self):
        self.observer = ExecutionObserver()
        self.request = np.array([.10, 0., 0., 0., 0., .20])

    def sample(self, now, factors=None, missing=()):
        self.observer.record_request(now, self.request, self.request)
        for stage in STAGES:
            if stage not in missing:
                factor = 1. if factors is None else factors.get(stage, 1.)
                self.observer.observe(stage, now, self.request*factor)
        return self.observer.feedback(now)

    def warm(self, factors=None):
        result = None
        for step in range(51):
            result = self.sample(step*.01, factors)
        return result

    def test_natural_transport_delay_and_acceleration_do_not_create_saturation(self):
        feedbacks = []
        for step in range(61):
            now = step*.01
            # 80ms transport delay, followed by an 80ms velocity ramp.
            factor = np.clip((now-.08)/.08, 0., 1.)
            feedbacks.append(self.sample(now, {'measured': factor}))
        self.assertFalse(any(any(item.limited) for item in feedbacks if item.valid))
        self.assertTrue(feedbacks[-1].valid)
        self.assertEqual(feedbacks[-1].limited, (False, False))
        self.assertTrue(all(not item.valid for item in feedbacks[:24]))

    def test_delayed_measurements_do_not_back_calculate_a_reverse_integral(self):
        controller = CartesianController({'algorithm': 'pi'})
        actual = np.array([0., 0., 0., 0., 0., 0., 1.])
        target = actual.copy(); target[0] = .04
        commands, integrals = [], []
        for step in range(81):
            now = step*.01
            feedback = self.observer.feedback(now)
            command = controller.update(target, actual, now, target_time=0., speed_scale=1.,
                                        execution_feedback=feedback)
            commands.append(command.copy())
            self.observer.record_request(now, command, command)
            for stage in ('servo', 'controller'):
                self.observer.observe(stage, now, command)
            delayed = commands[step-8] if step >= 8 else np.zeros(6)
            self.observer.observe('measured', now, delayed)
            integrals.append(controller._integral.copy())
        self.assertGreater(integrals[-1][0], 0.)  # Full-chain proof eventually permits normal I.
        self.assertTrue(all(value[0] >= 0. for value in integrals))
        self.assertTrue(all(value[0] > 0. for value in commands))

    def test_each_downstream_stage_can_qualify_a_sustained_cap(self):
        for stage in STAGES:
            with self.subTest(stage=stage):
                self.observer.reset()
                factors = {name: (.5 if STAGES.index(name) >= STAGES.index(stage) else 1.)
                           for name in STAGES}
                result = self.warm(factors)
                self.assertTrue(result.valid, self.observer.reason)
                self.assertEqual(result.limited, (True, True))
                np.testing.assert_allclose(result.effective_twist, self.request*.5)
                self.assertIn(stage, self.observer.limited_sources)
                self.assertEqual(result.source, 'measured' if stage == 'measured' else 'command')

    def test_dls_known_loss_is_immediate_but_unknown_other_axis_is_frozen(self):
        mapped = self.request.copy(); mapped[:3] *= .5
        self.observer.record_request(1., self.request, mapped)
        result = self.observer.feedback(1.)
        self.assertTrue(result.valid)
        self.assertEqual(result.source, 'command')
        self.assertEqual(result.limited, (True, True))
        np.testing.assert_allclose(result.effective_twist[:3], mapped[:3])
        np.testing.assert_array_equal(result.effective_twist[3:], self.request[3:])
        self.assertEqual(self.observer.limited_sources, ('dls',))
        self.assertFalse(self.observer.feedback(1.2).valid)

    def test_missing_stale_and_malformed_stage_never_advertise_spare_capacity(self):
        self.assertTrue(self.warm().valid)
        for step in range(51, 64):
            result = self.sample(step*.01, missing=('measured',))
        self.assertFalse(result.valid)
        self.assertEqual(self.observer.reason, 'measured_stale')
        self.sample(.64)
        self.assertFalse(self.observer.observe('controller', .641, [1., 2.]))
        self.assertFalse(self.observer.feedback(.641).valid)
        self.assertEqual(self.observer.reason, 'controller_stale')
        self.assertFalse(self.observer.observe('servo', .642, [np.nan]*6))
        self.assertFalse(self.observer.feedback(.642).valid)

    def test_direction_changes_and_changing_request_freeze_during_delay(self):
        self.assertTrue(self.warm().valid)
        self.request *= -1.
        for step in range(51, 67):
            result = self.sample(step*.01, {'measured': -1.})
            self.assertFalse(result.valid)
        for step in range(67, 85):
            now = step*.01
            self.observer.record_request(now, self.request, self.request)
            self.observer.observe('servo', now, self.request)
            self.observer.observe('controller', now, self.request)
            sideways = self.request.copy(); sideways[1] = .1
            self.observer.observe('measured', now, sideways)
            self.assertFalse(self.observer.feedback(now).valid)
        self.assertEqual(self.observer.reason, 'execution_direction_changing')

    def test_source_stamp_alignment_uses_an_existing_older_request(self):
        for step in range(61):
            now = step*.01
            self.observer.record_request(now, self.request, self.request)
            self.observer.observe('controller', now, self.request)
            self.observer.observe('measured', now, self.request)
            if step % 3 == 0:
                self.observer.observe('servo', now, self.request)
            result = self.observer.feedback(now)
        self.assertTrue(result.valid, self.observer.reason)
        self.assertLessEqual(result.command_time, result.stamp)
        self.assertIn(result.command_time, [item[0] for item in self.observer.requests])
        np.testing.assert_array_equal(result.requested_twist, self.request)

    def test_joint_velocity_fallback_uses_real_position_difference_and_resets_on_gap(self):
        model = NS(jacobian=lambda q: np.eye(6))
        q = np.array([0., -.0002, .0001, 0., 0., 0.])
        original = q.copy()
        self.assertFalse(self.observer.observe_joints('measured', 1., q, model))
        self.assertTrue(self.observer.observe_joints('measured', 1.01, q+.01*self.request, model))
        np.testing.assert_allclose(self.observer.observations['measured'][-1][1], self.request)
        np.testing.assert_array_equal(q, original)
        self.assertFalse(self.observer.observe_joints('measured', 1.2, q, model))
        self.assertFalse(self.observer.observations['measured'])
        self.assertTrue(self.observer.observe_joints('controller', 1.2, q, model, self.request))
        self.assertFalse(self.observer.observe_joints('controller', 1.21, q, model, [0.]*5))
        self.assertFalse(self.observer.observations['controller'])

    def test_reset_drops_old_capacity_and_out_of_order_samples_are_not_refreshed(self):
        self.assertTrue(self.warm().valid)
        previous = self.observer.observations['measured'][-1][0]
        self.assertFalse(self.observer.observe('measured', previous-.01, self.request))
        self.assertEqual(self.observer.observations['measured'][-1][0], previous)
        self.observer.reset()
        self.assertFalse(self.observer.feedback(1.).valid)
        self.observer.record_request(1., self.request, self.request)
        self.assertFalse(self.observer.feedback(1.).valid)
        self.assertEqual(self.observer.reason, 'servo_stale')


if __name__ == '__main__':
    unittest.main()
