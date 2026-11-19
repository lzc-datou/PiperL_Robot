"""Owned offscreen Qt windows only; no ROS, evdev or robot dependencies."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import unittest
from PyQt5 import QtCore, QtWidgets
from scripts.tools.gamepad_control.app import GamepadWindow
from scripts.tools.gamepad_control.preview import PreviewBridge


class UiBridge(PreviewBridge):
    def __init__(self):
        super().__init__()
        self.selections = []
        self.requests = []
        self.speeds = []
        self.selection_callback = None

    def select_control(self, control_mode, joint_index, callback=None):
        self.selections.append((control_mode, joint_index))
        self.selection_callback = callback
        return object()

    def request(self, name, callback=None):
        self.requests.append(name)
        return object()

    def set_speed(self, value):
        self.speeds.append(value)

    def complete_selection(self, success=True, message='已接受'):
        callback, self.selection_callback = self.selection_callback, None
        if callback:
            callback(success, message)


class GamepadUiModeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.bridge = UiBridge()
        self.window = GamepadWindow('sim', bridge=self.bridge)
        self.window.timer.stop()
        self.window.resize(1000, 760)
        self.window.show()
        self.refresh()

    def tearDown(self):
        self.window.allow_close = True
        self.window.close()
        self.application.processEvents()

    def refresh(self, **status):
        self.bridge.status.update(status)
        self.window.refresh()
        self.application.processEvents()

    def assert_selection_enabled(self, enabled):
        for button in (*self.window.mode_buttons.values(), *self.window.joint_buttons):
            self.assertEqual(button.isEnabled(), enabled, button.text())

    def test_initial_single_axis_j1_and_single_joint_readout(self):
        self.assertEqual(self.window.control_mode, 'single_axis')
        self.assertEqual(self.window.selected_joint, 1)
        self.assertTrue(self.window.mode_buttons['single_axis'].isChecked())
        self.assertEqual([button.isChecked() for button in self.window.joint_buttons],
                         [True, False, False, False, False, False])
        self.refresh(selected_joint=3)
        self.assertIn('J3', self.window.joint_value.text())
        self.assertIn('-68.8°', self.window.joint_value.text())
        self.assertIn('-169.6°', self.window.joint_range.text())
        self.assertIn('0.0°', self.window.joint_range.text())
        self.assertIn('17.2°/s', self.window.speed_label.text())
        self.assertNotIn('mm/s', self.window.speed_label.text())
        self.assertIn('选轴', self.window.mapping.text())
        self.assertIn('停稳', self.window.mapping.toolTip())

    def test_selection_waits_for_backend_snapshot_after_success_response(self):
        self.window.mode_buttons['cartesian'].click()
        self.assertEqual(self.bridge.selections, [('cartesian', 1)])
        self.assertEqual(self.window.control_mode, 'single_axis')
        self.assertTrue(self.window.mode_buttons['single_axis'].isChecked())
        self.assert_selection_enabled(False)
        self.bridge.complete_selection()
        self.refresh()
        self.assertEqual(self.window.control_mode, 'single_axis')
        self.assertFalse(self.window.mode_buttons['cartesian'].isChecked())
        self.refresh(control_mode='cartesian', selection_revision=2)
        self.assertTrue(self.window.mode_buttons['cartesian'].isChecked())
        self.assertTrue(self.window.joint_panel.isHidden())
        self.refresh(control_mode='single_axis', selection_revision=3)
        self.window.joint_buttons[4].click()
        self.assertEqual(self.bridge.selections[-1], ('single_axis', 5))
        self.assertEqual(self.window.selected_joint, 1)
        self.bridge.complete_selection()
        self.refresh(selected_joint=5, selection_revision=4)
        self.assertEqual(self.window.selected_joint, 5)
        self.assertTrue(self.window.joint_buttons[4].isChecked())

    def test_rejected_selection_keeps_authoritative_mode_and_reports_reason(self):
        self.window.mode_buttons['cartesian'].click()
        self.bridge.complete_selection(False, '请先停稳')
        self.refresh()
        self.assertEqual(self.window.control_mode, 'single_axis')
        self.assertIn('请先停稳', self.window.hint.text())

    def test_lb_b_neutral_and_phase_gate_selection_and_speed(self):
        self.assert_selection_enabled(True)
        self.assertTrue(self.window.speed.isEnabled())
        for key in ('lb', 'b'):
            with self.subTest(button=key):
                self.bridge.input[key] = True
                self.refresh()
                self.assert_selection_enabled(False)
                self.assertFalse(self.window.speed.isEnabled())
                self.window.speed.setValue(35)
                self.assertEqual(self.bridge.speeds, [])
                self.bridge.input[key] = False
                self.refresh()
        self.bridge.input['axes'] = (.4, 0., 0., 0.)
        self.refresh()
        self.assert_selection_enabled(False)
        self.assertTrue(self.window.speed.isEnabled())
        self.bridge.input['axes'] = (0.,) * 4
        self.refresh(selection_ready=False)
        self.assert_selection_enabled(False)
        self.refresh(selection_ready=True)
        for phase in ('ACTIVE', 'PREPARING', 'STOPPING', 'LOCKED', 'STOP_UNKNOWN'):
            with self.subTest(phase=phase):
                self.refresh(state=phase)
                self.assert_selection_enabled(False)
                self.assertFalse(self.window.speed.isEnabled())
        self.refresh(state='ARMED')
        self.assert_selection_enabled(True)
        self.assertTrue(self.window.speed.isEnabled())

    def test_cartesian_pose_is_readonly_and_fine_rotation_is_visible(self):
        self.refresh(control_mode='cartesian', speed_scale=.20)
        self.assertIsInstance(self.window.pose, QtWidgets.QLabel)
        self.assertIn('仅显示', self.window.pose.text())
        self.assertIn('mm/s', self.window.speed_label.text())
        self.assertIn('°/s', self.window.speed_label.text())
        self.assertIn('35%', self.window.speed_label.toolTip())
        self.assertIn('Rx/Ry', self.window.cartesian_mapping.text())
        self.assertIn('base_link', self.window.cartesian_mapping.text())

    def test_primary_action_changes_with_phase_and_stop_stays_visible(self):
        expected = {'WAITING': (True, False, False), 'ARMED': (False, True, False),
                    'ACTIVE': (False, True, False), 'PREPARING': (False, True, False),
                    'STOPPING': (False, True, False), 'LOCKED': (False, False, True),
                    'STOP_UNKNOWN': (False, False, True)}
        for phase, visible in expected.items():
            with self.subTest(phase=phase):
                self.refresh(state=phase)
                self.assertEqual(tuple(button.isVisible() for button in
                                       (self.window.start, self.window.end, self.window.recover)), visible)
                self.assertTrue(self.window.stop.isVisible())
        self.refresh(state='LOCKED', stop_confirmed=False)
        self.assertFalse(self.window.recover.isEnabled())
        self.refresh(stop_confirmed=True)
        self.assertTrue(self.window.recover.isEnabled())

    def test_large_fonts_and_small_screen_scrolling_keep_stop_fixed(self):
        self.assertGreaterEqual(self.window.status.font().pixelSize(), 32)
        self.assertGreaterEqual(self.window.hint.font().pixelSize(), 22)
        self.assertGreaterEqual(self.window.stop.font().pixelSize(), 22)
        self.window.resize(self.window.minimumWidth(), self.window.minimumHeight())
        self.window.details_button.setChecked(True)
        self.application.processEvents()
        self.assertGreater(self.window.body_scroll.verticalScrollBar().maximum(), 0)
        self.assertFalse(self.window.body_scroll.isAncestorOf(self.window.stop))
        for position in (0, self.window.body_scroll.verticalScrollBar().maximum()):
            self.window.body_scroll.verticalScrollBar().setValue(position)
            self.application.processEvents()
            self.assertTrue(self.window.stop.isVisible())
            rect = QtCore.QRect(self.window.stop.mapTo(self.window, QtCore.QPoint()), self.window.stop.size())
            self.assertTrue(self.window.rect().contains(rect), (self.window.size(), rect))


if __name__ == '__main__':
    unittest.main()
