"""Owned offscreen Qt windows only; no ROS, evdev or robot dependencies."""
import os
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

import unittest
from types import SimpleNamespace
from unittest.mock import patch
from PyQt5 import QtCore, QtTest, QtWidgets
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

    def test_single_axis_tiles_show_all_angles_and_confirmed_selection(self):
        self.assertEqual(self.window.control_mode, 'single_axis')
        self.assertEqual(self.window.selected_joint, 1)
        self.assertTrue(self.window.mode_buttons['single_axis'].isChecked())
        self.assertEqual([button.isChecked() for button in self.window.joint_buttons],
                         [True, False, False, False, False, False])
        self.refresh(selected_joint=3)
        self.assertEqual([label.text() for label in self.window.joint_values],
                         ['0.000', '80.214', '-68.755', '17.189', '57.296', '0.000'])
        for index, tile in enumerate(self.window.joint_buttons):
            self.assertTrue(tile.isAncestorOf(self.window.joint_values[index]))
            self.assertEqual(tile.isChecked(), index == 2)
        self.assertIn('17.2°/s', self.window.speed_label.toolTip())
        self.assertNotIn('mm/s', self.window.speed_label.toolTip())
        self.assertTrue(self.window.mapping.single)

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
        tile = self.window.joint_buttons[4]
        value = self.window.joint_values[4]
        point = value.mapTo(tile, value.rect().center())
        target = self.window.childAt(tile.mapTo(self.window, point))
        self.assertIs(target, tile)
        QtTest.QTest.mouseClick(target, QtCore.Qt.LeftButton, pos=point)
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
        self.assertEqual([v.text() for v in self.window.pose_values],
                         ['250.0', '0.0', '300.0', '0.0', '0.0', '0.0'])
        page = self.window.motion_panel.currentWidget()
        self.assertFalse(page.findChildren(QtWidgets.QAbstractButton))
        self.assertTrue(all(page.isAncestorOf(v) for v in self.window.pose_values))
        self.assertIn('base_link', self.window.frame_label.text())
        self.assertFalse(self.window.mapping.single)
        tip = self.window.speed_label.toolTip()
        for unit in ('mm/s', '°/s', '35%'):
            self.assertIn(unit, tip)
        self.bridge.status['feedback']['age_s'] = 1.0
        self.refresh()
        self.assertEqual(self.window.feedback_state.text(), '反馈中断')
        self.assertTrue(all(v.text() == '—' for v in self.window.pose_values + self.window.joint_values))

    def test_home_speed_is_separate_from_the_jog_setting(self):
        self.refresh(state='HOME_ARMING', home_hold_progress=.56)
        self.assertEqual(self.window.speed_label.text(), '10%')
        self.assertIn('56%', self.window.status.text())
        self.assertTrue(self.window.jog_slider.isHidden())
        self.assertIn('Y + LB', self.window.hint.text())
        self.refresh(state='ARMED')
        self.assertEqual(self.window.speed_label.text(), '20%')
        self.assertTrue(self.window.jog_slider.isVisible())
        self.assertEqual(self.bridge.speeds, [])

    def test_signed_angles_fit_resize_and_long_notice_keeps_stop_in_view(self):
        self.refresh(selected_joint=3)
        for width in (600, 900, 1100, 1280):
            self.window.resize(width, 580)
            self.application.processEvents()
            for value in self.window.joint_values:
                self.assertGreaterEqual(value.contentsRect().width(),
                                        value.fontMetrics().horizontalAdvance('-180.000') - 1)
        self.window.resize(600, 480)
        self.refresh(state='LOCKED', reason='输入反馈中断，请检查控制器连接。' * 40)
        self.assertGreater(self.window.body_scroll.verticalScrollBar().maximum(), 0)
        self.assertEqual(self.window.body_scroll.horizontalScrollBar().maximum(), 0)
        stop = self.window.stop
        rect = QtCore.QRect(stop.mapTo(self.window, QtCore.QPoint()), stop.size())
        self.assertTrue(self.window.rect().contains(rect))

    def test_demo_recovery_and_mode_speeds_are_local(self):
        demo = GamepadWindow('demo')
        try:
            demo.speed.setValue(35)
            demo.mode_buttons['cartesian'].click()
            self.assertEqual(demo.speed.value(), 20)
            demo.mode_buttons['single_axis'].click()
            self.assertEqual(demo.speed.value(), 35)
            demo.start.click()
            demo.stop.click()
            self.assertEqual(demo.state, 'LOCKED')
            self.assertTrue(demo.recover.isEnabled())
            demo.recover.click()
            self.assertEqual(demo.state, 'ARMED')
        finally:
            demo.close()

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

    def test_diagnostic_updates_preserve_selection_and_both_scrollbars(self):
        self.window.resize(720, 580)
        self.window.details_button.setChecked(True)
        table = self.window.diag
        with patch('time.monotonic', return_value=100.) as clock:
            self.refresh(state='ARMED', input_age_s=.012)
            age = table.cells['输入延迟（秒）']
            self.assertEqual(age.text(), '0.012')
            row = table.rowCount() - 3
            table.setCurrentCell(row, 1)
            inner, outer = table.verticalScrollBar(), self.window.body_scroll.verticalScrollBar()
            for scrollbar in (inner, outer):
                self.assertGreater(scrollbar.maximum(), 10)
                scrollbar.setValue(scrollbar.maximum() - 10)
            position = inner.value(), outer.value()
            self.bridge.status['feedback']['joints_rad'][2] = None
            for tick in range(1, 202):
                clock.return_value = 100. + .05 * tick
                self.refresh(input_age_s=tick * .001, neutral_seen=bool(tick % 2))
                self.assertEqual((inner.value(), outer.value()), position)
                self.assertEqual(table.currentRow(), row)
                self.assertTrue(table.item(row, 1).isSelected())
                if tick == 1:
                    self.assertEqual(age.text(), '0.012')
            self.assertGreater(float(age.text()), .19)
            self.assertIn('—', table.cells['关节角（rad）'].text())
            self.window.details_button.setChecked(False)
            clock.return_value += 1.
            last_age = age.text()
            self.refresh(input_age_s=1.)
            self.assertEqual(age.text(), last_age)
            self.window.details_button.setChecked(True)
            self.refresh()
            self.assertEqual(age.text(), '1.000')

    def test_footer_geometry_and_full_status_in_compact_and_wide_windows(self):
        for width in (600, 1280):
            with self.subTest(width=width):
                self.window.resize(width, 620)
                self.refresh(state='ARMED', neutral_seen=True)
                # Let the scroll area's queued layout finish changing orientation.
                QtTest.QTest.qWait(20)
                watched = (self.window.body_scroll, self.window.work_card, self.window.controls,
                           self.window.status_area, self.window.stop)
                geometry = [w.geometry() for w in watched]
                self.refresh(neutral_seen=False)
                self.assertEqual(self.window.status.text(), '等待回中')
                self.assertIn('摇杆和方向键回中', self.window.hint.text())
                self.assertEqual([w.geometry() for w in watched], geometry)
                reason = '反馈连接中断，请检查连接。' * 40
                self.refresh(state='LOCKED', reason=reason, stop_confirmed=False)
                self.assertIn('等待停稳确认', self.window.hint.text())
                self.assertEqual([w.geometry() for w in watched], geometry)
                QtTest.QTest.mouseClick(self.window.status_area, QtCore.Qt.LeftButton)
                dialog = self.window.status_area.dialog
                self.assertTrue(dialog.isVisible())
                self.assertFalse(dialog.isModal())
                self.assertIn(reason, self.window.status_area.detail_text.toPlainText())
                QtTest.QTest.mouseClick(self.window.stop, QtCore.Qt.LeftButton)
                self.assertEqual(self.bridge.requests[-1], 'stop')
                dialog.close()
                self.refresh(state='HOME_ARMING', home_hold_progress=.35)
                self.assertEqual([w.geometry() for w in watched], geometry)

    def test_mode_button_hover_leave_press_and_keyboard_follow_confirmed_mode(self):
        single, cartesian = self.window.mode_buttons.values()
        self.assertEqual(self.window.mapping.title.text(), '手柄映射')
        self.assertEqual(single.size(), cartesian.size())
        self.assertEqual(cartesian.cursor().shape(), QtCore.Qt.PointingHandCursor)
        def fill(button):
            return button.grab().toImage().pixelColor(12, button.height() // 2)
        QtTest.QTest.mouseMove(self.window.stop)
        plain = fill(cartesian)
        QtTest.QTest.mouseMove(cartesian)
        hover = fill(cartesian)
        self.assertNotEqual(hover, plain)
        QtTest.QTest.mousePress(cartesian, QtCore.Qt.LeftButton)
        self.assertNotEqual(fill(cartesian), hover)
        self.assertTrue(single.isChecked())
        QtTest.QTest.mouseRelease(cartesian, QtCore.Qt.LeftButton, pos=QtCore.QPoint(-5, -5))
        QtTest.QTest.mouseMove(self.window.stop)
        self.assertEqual(fill(cartesian), plain)
        self.assertEqual(self.bridge.selections, [])
        cartesian.setFocus()
        QtTest.QTest.keyClick(cartesian, QtCore.Qt.Key_Space)
        self.assertEqual(self.bridge.selections, [('cartesian', 1)])
        self.assertTrue(single.isChecked())
        self.assertFalse(cartesian.isChecked())
        self.assertEqual(cartesian.cursor().shape(), QtCore.Qt.ArrowCursor)
        self.bridge.complete_selection()
        self.refresh(control_mode='cartesian')
        self.assertTrue(cartesian.isChecked())
        self.assertEqual(self.window.mapping.title.text(), '手柄映射')

    def test_raw_input_disconnect_clears_values_and_reports_error(self):
        frame = dict(self.bridge.input, device={'name': '测试手柄', 'path': '测试路径', 'connection': '蓝牙'},
                     guid='测试设备', link_verified=True)
        adapter = SimpleNamespace(latest=lambda: frame, error='', stop=lambda: None)
        window = GamepadWindow('input', input_adapter=adapter)
        try:
            window.show()
            window.timer.stop()
            window.refresh()
            self.application.processEvents()
            for line in (window.status, window.hint):
                self.assertGreaterEqual(line.height(), line.fontMetrics().height())
            self.assertEqual(window.diag.cells['GUID'].text(), '测试设备')
            self.assertEqual(window.diag.cells['连接方式'].text(), '蓝牙')
            self.assertIn('蓝牙已连接', window.device.text())
            self.assertEqual(window.diag.cells['连接失联验证'].text(), 'True')
            frame = None
            adapter.error = '设备已断开'
            window.next_diagnostic_update = 0.
            window.refresh()
            self.assertEqual(window.diag.cells['GUID'].text(), '—')
            self.assertEqual(window.diag.cells['摇杆'].text(), '—')
            self.assertEqual(window.diag.cells['输入错误'].text(), adapter.error)
        finally:
            window.close()

    def test_real_window_searches_then_keeps_bluetooth_online_across_lb_edges(self):
        frame = self.bridge.input
        self.bridge.input = None
        self.bridge.spin = lambda: None
        window = GamepadWindow('real', bridge=self.bridge, device_selector='bluetooth')
        window.timer.stop()
        try:
            window.refresh()
            self.assertEqual(window.device.text(), '正在查找蓝牙手柄')
            self.assertFalse(window.start.isEnabled())
            self.bridge.input = frame
            self.bridge.status.update(state='WAITING', servo_switched=True)
            with patch('time.monotonic', return_value=10.):
                for held in (False, True, True, False):
                    frame.update(lb=held, received_at=10., connection_type='蓝牙', device_name='Xbox')
                    window.refresh()
                    self.assertEqual(window.device.text(), '蓝牙已连接 · Xbox')
                    self.assertTrue(window.start.isEnabled())
                frame['connected'] = False
                window.refresh()
                self.assertIn('输入中断', window.device.text())
                self.assertFalse(window.start.isEnabled())
        finally:
            window.allow_close = True
            window.close()

    def test_stack_exit_reason_survives_control_refresh(self):
        self.window.report_stack_exit(1, 'logs/stack.log')
        self.refresh(state='ARMED')
        self.assertEqual(self.window.status.text(), '后端已退出')
        self.assertIn('logs/stack.log', self.window.hint.text())

    def test_large_fonts_and_small_screen_scrolling_keep_stop_fixed(self):
        self.assertGreaterEqual(self.window.status.font().pixelSize(), 18)
        self.assertGreaterEqual(self.window.hint.font().pixelSize(), 18)
        self.assertGreaterEqual(self.window.stop.font().pixelSize(), 22)
        self.window.resize(self.window.minimumWidth(), self.window.minimumHeight())
        self.window.details_button.setChecked(True)
        self.application.processEvents()
        self.assertGreater(self.window.body_scroll.verticalScrollBar().maximum(), 0)
        self.assertEqual(self.window.body_scroll.horizontalScrollBar().maximum(), 0)
        self.assertFalse(self.window.body_scroll.isAncestorOf(self.window.stop))
        for position in (0, self.window.body_scroll.verticalScrollBar().maximum()):
            self.window.body_scroll.verticalScrollBar().setValue(position)
            self.application.processEvents()
            self.assertTrue(self.window.stop.isVisible())
            rect = QtCore.QRect(self.window.stop.mapTo(self.window, QtCore.QPoint()), self.window.stop.size())
            self.assertTrue(self.window.rect().contains(rect), (self.window.size(), rect))


if __name__ == '__main__':
    unittest.main()
