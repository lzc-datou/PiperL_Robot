"""Regressions for terminal exit and Xbox-named evdev discovery; no ROS or CAN."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

from scripts.tools.gamepad_control.input_adapter import EvdevInput


ROOT = Path(__file__).resolve().parents[1]
WINDOW_PROCESS = r'''
import sys
from PyQt5 import QtCore, QtWidgets
from scripts.tools.gamepad_control.app import GamepadWindow
from scripts.tools.gamepad_control.input_adapter import EvdevInput
from scripts.tools.gamepad_control.run import _exec_window

app = QtWidgets.QApplication([])
adapter = None
bridge = None
mode, action = sys.argv[1:]
if mode == 'input':
    adapter = EvdevInput()
    adapter.devices = lambda path=None: ([], '测试：没有手柄')
    adapter.start()
elif mode == 'sim':
    class Bridge:
        status = {'state': 'ACTIVE', 'servo_switched': True}
        guard = {'state': 'running', 'stop_confirmed': False, 'stop_generation': 0}
        input = None
        last_reply = ''
        def spin(self): pass
        def heartbeat(self): pass
        def request(self, name, callback=None):
            assert name == 'stop'
            print('STOP_REQUESTED', flush=True)
            self.status = {'state': 'STOPPING', 'stop_generation_required': 1}
            def confirm():
                self.guard = {'state': 'idle', 'stop_confirmed': True, 'stop_generation': 1}
                print('STOP_CONFIRMED', flush=True)
            QtCore.QTimer.singleShot(250, confirm)
            return object()
    bridge = Bridge()
window = GamepadWindow(mode, input_adapter=adapter, bridge=bridge)
if mode == 'input':
    assert not any(button.isEnabled() for button in (window.start, window.end, window.recover, window.stop))
QtCore.QTimer.singleShot(150, lambda: print('READY', flush=True))
if action == 'close':
    QtCore.QTimer.singleShot(300, window.close)
try:
    code = _exec_window(app, window)
finally:
    if adapter:
        adapter.stop()
        assert not adapter._thread.is_alive()
print('CLOSED', flush=True)
raise SystemExit(code)
'''


class GamepadWindowLifecycleTests(unittest.TestCase):
    def run_window(self, mode, action):
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
        process = subprocess.Popen(
            [sys.executable, '-u', '-c', WINDOW_PROCESS, mode, action], cwd=ROOT,
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            ready, _, _ = select.select([process.stdout], [], [], 8.)
            self.assertTrue(ready, 'Qt window did not start')
            self.assertEqual(process.stdout.readline().strip(), 'READY')
            if action != 'close':
                process.send_signal(getattr(signal, action))
            output, errors = process.communicate(timeout=6.)
            self.assertEqual(process.returncode, 0, errors)
            self.assertNotIn('KeyboardInterrupt', errors)
            self.assertIn('CLOSED', output)
            return output
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()

    def test_no_device_window_closes_and_reader_stops(self):
        self.run_window('input', 'close')

    def test_ctrl_c_closes_demo_and_input_without_qt_exception(self):
        for mode in ('demo', 'input'):
            with self.subTest(mode=mode):
                self.run_window(mode, 'SIGINT')

    def test_sigterm_closes_input_without_qt_exception(self):
        self.run_window('input', 'SIGTERM')

    def test_terminal_signal_preserves_guarded_close(self):
        output = self.run_window('sim', 'SIGINT')
        self.assertLess(output.index('STOP_REQUESTED'), output.index('STOP_CONFIRMED'))
        self.assertLess(output.index('STOP_CONFIRMED'), output.index('CLOSED'))


class GamepadDiscoveryTests(unittest.TestCase):
    def test_xbox_name_is_usable_but_multiple_devices_require_selection(self):
        adapter = EvdevInput({'name_hint': 'flydigi'})
        xbox = {'name': 'Microsoft X-Box 360 pad', 'path': '/dev/input/event17'}
        self.assertEqual(adapter._choose_device([xbox]), xbox)
        with self.assertRaisesRegex(ValueError, '--device'):
            adapter._choose_device([xbox, dict(xbox, path='/dev/input/event18')])

    def test_native_sysfs_bitmap_does_not_lose_zero_words(self):
        adapter = EvdevInput()
        # Recorded kernel capability formatting for the connected XInput pad.
        with patch.object(Path, 'read_text', side_effect=['3003f\n', '7cdb000000000000 0 0 0 0\n']):
            self.assertTrue(adapter._supports_profile('/dev/input/event17'))

    def test_permission_error_is_distinct_from_no_device(self):
        adapter = EvdevInput()
        with patch('glob.glob', return_value=['/dev/input/event17']), \
                patch.object(adapter, '_supports_profile', return_value=True), \
                patch('os.open', side_effect=PermissionError('denied')):
            devices, reason = adapter.devices()
        self.assertEqual(devices, [])
        self.assertIn('无读取权限', reason)
        with patch('glob.glob', return_value=[]):
            devices, reason = adapter.devices()
        self.assertIn('系统未发现兼容手柄', reason)

    def test_reconnect_keeps_explicit_device_and_stop_interrupts_wait(self):
        adapter = EvdevInput()
        with patch.object(adapter, 'devices', return_value=([], '设备尚未连接')) as scan:
            adapter.start('/dev/input/by-id/selected-event-joystick')
            deadline = time.monotonic() + 1.
            while scan.call_count < 2 and time.monotonic() < deadline:
                time.sleep(.01)
            started = time.monotonic()
            adapter.stop()
            self.assertLess(time.monotonic()-started, .5)
            self.assertFalse(adapter._thread.is_alive())
            self.assertGreaterEqual(scan.call_count, 2)
            self.assertTrue(all(call.args == ('/dev/input/by-id/selected-event-joystick',)
                                for call in scan.call_args_list))


if __name__ == '__main__':
    unittest.main()
