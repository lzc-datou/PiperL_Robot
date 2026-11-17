"""Controller synchronization timing with fake services; no arm connection."""
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import yaml

from scripts.task.servo_tracking.core import load_config

try:
    from tests import test_servo_guard_recovery as recovery
    ROS = recovery.ROS
except ImportError:
    ROS = False


CONFIG = Path(__file__).resolve().parents[1]/'scripts/task/servo_tracking/config/config.yaml'


class ControllerSyncConfigTests(unittest.TestCase):
    def config(self, value=None):
        data = yaml.safe_load(CONFIG.read_text())
        data.pop('controller_sync_settle_time', None)
        if value is not None:
            data['controller_sync_settle_time'] = value
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml') as stream:
            yaml.safe_dump(data, stream)
            stream.flush()
            return load_config(stream.name)

    def test_default_and_inclusive_safe_range(self):
        self.assertEqual(self.config()['controller_sync_settle_time'], .08)
        for value in (.08, .2, 1.):
            self.assertEqual(self.config(value)['controller_sync_settle_time'], value)
        for value in (0., .0799, 1.001, -1., True, float('nan'), float('inf'), '0.08'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config(value)


@unittest.skipUnless(ROS, 'Use the ROS wrapper; every service is fake')
class ControllerSyncTimingTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('scripts.task.servo_tracking.guard.time.monotonic', return_value=100.)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)

    def syncing(self, settle=.08, resume=False):
        state = recovery.ServoGuardRecoveryTests.guard(self)
        state.config['controller_sync_settle_time'] = settle
        if resume:
            state.state = 'running'
            self.assertTrue(state.hold_service(NS(), NS()).success)
            self.assertFalse(state.enable(NS(data=True), NS()).success)
            state.gate.complete(); state.hold.complete(); state.tick()
        self.assertTrue(state.enable(NS(data=True), NS()).success)
        state.pause_servo.complete()
        self.assertEqual(state.state, 'syncing')
        return state

    def advance(self, state, now, actual=True, commanded=True):
        self.now.return_value, state.heartbeat = now, now
        if actual:
            state.feedback.stamp = now
        if commanded:
            state.commanded.stamp = now
        state.tick()

    def test_healthy_held_resume_uses_80ms_but_still_waits_for_reset_and_gate_ack(self):
        state = self.syncing(resume=True)
        self.advance(state, 100.01)
        self.advance(state, 100.07)
        self.assertEqual(state.state, 'syncing')
        self.advance(state, 100.09)
        self.assertEqual(state.state, 'resetting')
        self.assertTrue(all(not request.data for request, _ in state.gate.calls))
        state.pause_servo.complete()
        self.assertEqual(state.state, 'opening')
        self.assertTrue(state.gate.calls[-1][0].data)
        state.gate.complete()
        self.assertEqual(state.state, 'running')

    def test_longer_configured_window_is_honored(self):
        state = self.syncing(.2)
        self.advance(state, 100.01)
        self.advance(state, 100.09)
        self.assertEqual(state.state, 'syncing')
        self.advance(state, 100.21)
        self.assertEqual(state.state, 'resetting')

    def test_frozen_feedback_cannot_pass_by_elapsed_time_alone(self):
        for frozen in ('actual', 'commanded'):
            with self.subTest(frozen=frozen):
                self.now.return_value = 100.
                state = self.syncing()
                self.advance(state, 100.01)
                self.advance(state, 100.09, actual=frozen != 'actual', commanded=frozen != 'commanded')
                self.assertEqual(state.state, 'syncing')
                self.assertEqual(state.sync_samples, 1)
                self.advance(state, 100.11)
                self.assertEqual(state.state, 'resetting')

    def test_controller_mismatch_or_actual_motion_restarts_full_window(self):
        for disturbance in ('mismatch', 'motion'):
            with self.subTest(disturbance=disturbance):
                self.now.return_value = 100.
                state = self.syncing()
                self.advance(state, 100.01)
                q = state.health.return_value.copy()
                changed = q.copy(); changed[0] += .004
                state.commanded.get.return_value = changed
                if disturbance == 'motion':
                    state.health.return_value = changed
                self.advance(state, 100.09)
                self.assertEqual(state.state, 'syncing')
                self.assertEqual(state.sync_samples, 0)
                state.commanded.get.return_value = state.health.return_value.copy()
                self.advance(state, 100.11)
                self.advance(state, 100.17)
                self.assertEqual(state.state, 'syncing')
                self.advance(state, 100.19)
                self.assertEqual(state.state, 'resetting')


if __name__ == '__main__':
    unittest.main()
