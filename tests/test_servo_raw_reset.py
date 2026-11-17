"""Raw measured-state reset is visualization, never calibration or motion authority."""
import json
from pathlib import Path
import time
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

try:
    from scripts.task.servo_tracking.node import Backend
    from scripts.task.servo_tracking.core import RobotModel, load_config
    ROS = True
except ImportError:
    ROS = False

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'
CONFIG = ROOT / 'scripts/task/servo_tracking/config/config.yaml'


@unittest.skipUnless(ROS, 'Use built ROS runtime wrapper; feedback/services are fake')
class ServoRawResetTests(unittest.TestCase):
    def state(self, joints):
        model = RobotModel(URDF.read_text(), tip='link6')
        config = load_config(CONFIG)
        raw = np.asarray(joints, dtype=float)
        state = NS(model=model, config=config, feedback=Mock(), mode='real',
                   controller=Mock(antiwindup_state='inactive'),
                   target=None, approved_target=None, preview_q=None, revision=3,
                   valid_revision=-1, aligned_revision=-1, valid=False, active=False,
                   dragging=False, pending=None, start_pending=False, resume_future=None,
                   joint_epoch=4, joint_id='', joint_purpose='', joint_busy=False,
                   plan_ready=False, joint_wait=None, joint_future=None, guard_can_plan=False,
                   hold_future=None, hold_revision=-1, hold_requested_at=0.,
                   settle_since=0., settle_q=None, verification_gap_since=None,
                   guard_state='idle', guard_reason='', pause_revision=-1,
                   panel_seen=time.monotonic(), switched=True, control_mode='servo',
                   scale=.5, workspace_visible=False, marker_color='', ghost_revision=-1,
                   publish_twist=Mock(), publish_ghost=Mock(), marker=Mock(),
                   gate=Mock(), ik=Mock(), status=Mock(), server=Mock(), reason='', reachability='')
        state.feedback.get.return_value = raw
        state.server.get.return_value = None
        for name in ('raw_actual', 'actual', 'align', 'stop_tracking'):
            setattr(state, name, MethodType(getattr(Backend, name), state))
        state.begin_tracking = Mock()
        state.gate.service_is_ready.return_value = True
        state.ik.service_is_ready.return_value = True
        return state, raw

    def assert_no_motion(self, state):
        self.assertFalse(state.active)
        self.assertTrue(all(not call.args[0].data for call in state.gate.call_async.call_args_list))
        state.ik.call_async.assert_not_called()
        if state.publish_twist.called:
            np.testing.assert_array_equal(state.publish_twist.call_args.args[0], np.zeros(6))

    def test_raw_actual_preserves_out_of_range_feedback_while_control_actual_rejects_it(self):
        state, raw = self.state([0., -.04, 0., 0., 0., 0.])
        before = raw.copy()
        q, pose = Backend.raw_actual(state)
        np.testing.assert_array_equal(q, before)
        np.testing.assert_array_equal(raw, before)
        np.testing.assert_allclose(pose, state.model.pose(before))
        self.assertTrue(np.isfinite(pose).all())
        with self.assertRaisesRegex(ValueError, 'joint2'):
            Backend.actual(state)

    def test_raw_actual_rejects_stale_incomplete_and_nonfinite_feedback(self):
        state, _ = self.state([0.] * 6)
        state.feedback.get.side_effect = ValueError('实测反馈已过期')
        with self.assertRaisesRegex(ValueError, '已过期'):
            Backend.raw_actual(state)
        state.feedback.get.side_effect = None
        for raw in ([0.] * 5, [float('nan')] * 6, [float('inf')] * 6):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                state.feedback.get.return_value = np.asarray(raw)
                Backend.raw_actual(state)

    def test_reset_outside_feedback_tolerance_succeeds_visually_but_cannot_start(self):
        state, raw = self.state([0., -.04, 0., 0., 0., 0.])
        response = Backend.reset_service(state, None, NS())
        self.assertTrue(response.success)
        self.assertFalse(state.valid)
        np.testing.assert_allclose(state.target, state.model.pose(raw))
        np.testing.assert_array_equal(state.publish_ghost.call_args.args[0], raw)
        self.assertIn('joint2', state.reason + state.reachability)
        self.assertIn('越界', state.reason + state.reachability)
        target = state.target.copy()
        response = Backend.start(state, None, NS())
        self.assertFalse(response.success)
        np.testing.assert_array_equal(state.target, target)
        state.begin_tracking.assert_not_called()
        self.assert_no_motion(state)

    def test_reset_zero_edge_feedback_keeps_raw_hold_without_ik_authorizing_motion(self):
        state, raw = self.state([0., -.0005, .0002, 0., 0., 0.])
        before = raw.copy()
        response = Backend.reset_service(state, None, NS())
        self.assertTrue(response.success)
        self.assertTrue(state.valid)
        self.assertEqual(state.aligned_revision, state.revision)
        self.assertEqual(state.valid_revision, state.revision)
        np.testing.assert_array_equal(raw, before)
        np.testing.assert_array_equal(state.preview_q, before)
        np.testing.assert_array_equal(state.publish_ghost.call_args.args[0], before)
        np.testing.assert_array_equal(state.approved_target, state.model.pose(before))
        with self.assertRaisesRegex(ValueError, 'joint2'):
            state.model.check_joints(raw)  # Command/target bounds remain strict.
        Backend.validation_tick(state)
        self.assert_no_motion(state)

    def test_zero_edge_tolerance_applies_only_to_real_feedback(self):
        state, raw = self.state([0., -.0005, .0002, 0., 0., 0.])
        actual_q, _ = Backend.actual(state)
        np.testing.assert_array_equal(actual_q, raw)
        state.mode = 'sim'
        with self.assertRaisesRegex(ValueError, 'joint2'):
            Backend.actual(state)
        # Even in strict sim mode, resetting the visualization still succeeds.
        response = Backend.reset_service(state, None, NS())
        self.assertTrue(response.success)
        self.assertFalse(state.valid)
        np.testing.assert_array_equal(state.preview_q, raw)
        self.assert_no_motion(state)

    def test_reset_inside_model_is_a_hold_and_never_sends_enable(self):
        state, raw = self.state([0., 1.4, -1.2, .3, 1., 0.])
        response = Backend.reset_service(state, None, NS())
        self.assertTrue(response.success)
        self.assertTrue(state.valid)
        np.testing.assert_array_equal(state.target, state.model.pose(raw))
        Backend.validation_tick(state)
        self.assert_no_motion(state)

    def test_status_uses_raw_fk_errors_even_when_feedback_is_out_of_model_bounds(self):
        state, raw = self.state([0., -.04, 0., 0., 0., 0.])
        Backend.reset_service(state, None, NS())
        state.target = state.target.copy()
        state.target[0] += .01
        Backend.publish_status(state)
        status = json.loads(state.status.publish.call_args.args[0].data)
        self.assertAlmostEqual(status['position_error_mm'], 10., places=8)
        self.assertAlmostEqual(status['orientation_error_deg'], 0., places=8)
        self.assertNotEqual(status['singularity_level'], 'unknown')
        self.assertIn('joint2', status['state'] + status['reachability'])
        self.assertFalse(status['feedback_accepted'])
        self.assertIn('joint2', status['feedback_error'])
        self.assertEqual([bound['joint'] for bound in status['feedback_bounds']], ['joint2'])
        self.assertEqual(status['feedback_bounds'][0]['value'], raw[1])
        self.assert_no_motion(state)

    def test_status_reports_tolerated_raw_bound_offsets_without_hiding_them(self):
        state, raw = self.state([0., -.0005, .0002, 0., 0., 0.])
        Backend.reset_service(state, None, NS())
        Backend.publish_status(state)
        status = json.loads(state.status.publish.call_args.args[0].data)
        self.assertTrue(status['feedback_accepted'])
        self.assertEqual(status['feedback_error'], '')
        self.assertEqual([bound['joint'] for bound in status['feedback_bounds']], ['joint2', 'joint3'])
        self.assertEqual([bound['value'] for bound in status['feedback_bounds']], list(raw[1:3]))
        self.assertAlmostEqual(status['position_error_mm'], 0.)
        self.assertAlmostEqual(status['orientation_error_deg'], 0.)
        self.assert_no_motion(state)


if __name__ == '__main__':
    unittest.main()
