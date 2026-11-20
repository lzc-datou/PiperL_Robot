"""Regressions for terminal exit and Xbox-named evdev discovery; no ROS or CAN."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from scripts.tools.gamepad_control import input_adapter as ev
from scripts.tools.gamepad_control.input_adapter import EvdevInput
from scripts.tools.gamepad_control.settings import load_input_profile


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
    def test_bluetooth_filter_mapping_and_renumbered_reconnect(self):
        adapter = EvdevInput(load_input_profile())
        pad = {'bus': 5, 'vendor': 0x045e, 'product': 0x02e0, 'version': 0x0903,
               'name': 'Xbox Wireless Controller', 'guid': 'wireless', 'path': '/dev/input/event18'}
        usb = dict(pad, bus=3, guid='receiver', path='/dev/input/event17')
        bits = 0x3ff << 0x130
        mapping, report = adapter._device_layout(pad, bits)
        self.assertEqual(mapping, {'lb': 0x134, 'b': 0x131, 'home': 0x133})
        self.assertEqual(report, (1, 16))
        for device, mask in ((usb, bits), (dict(pad, version=0x0500), bits), (pad, 0x7cdb << 0x130)):
            self.assertEqual(adapter._device_layout(device, mask), (adapter.key_codes, None))
        with patch('glob.glob', return_value=[usb['path'], pad['path']]), \
                patch.object(adapter, '_supports_profile', return_value=True), \
                patch('os.open', side_effect=[17, 18, 17, 18]), patch('os.close'), \
                patch.object(ev, '_device_info', side_effect=[usb, pad, usb, pad]), \
                patch.object(ev, '_absinfo', return_value=(0, -1, 1, 0, 0, 0)), \
                patch('fcntl.ioctl', side_effect=lambda fd, request, buf, mutate: buf.__setitem__(
                      slice(None), bits.to_bytes(len(buf), 'little'))):
            devices, error = adapter.devices('bluetooth')
            wired, wired_error = adapter.devices('usb')
        self.assertEqual(wired_error, '')
        self.assertEqual([item['guid'] for item in wired], ['receiver'])
        self.assertEqual(wired[0]['buttons'], adapter.key_codes)
        self.assertEqual(error, '')
        self.assertEqual([item['guid'] for item in devices], ['wireless'])
        self.assertEqual(devices[0]['buttons'], mapping)
        adapter.device = pad
        reconnected = dict(pad, path='/dev/input/event23')
        self.assertEqual(adapter._choose_device([usb, reconnected]), reconnected)
        self.assertIsNone(adapter._choose_device([usb]))

    def test_only_actual_control_activity_renews_bluetooth_hold(self):
        event_r, event_w = os.pipe2(os.O_NONBLOCK)
        raw_r, raw_w = os.pipe2(os.O_NONBLOCK)
        try:
            controls = {(ev.EV_KEY, ev.BTN_WEST), (ev.EV_ABS, ev.ABS_X)}
            def check(expected):
                self.assertEqual(ev._wait_for_input(event_r, raw_r, controls, (1, 16)), expected)
            os.write(event_w, ev.INPUT_EVENT.pack(0, 0, ev.EV_KEY, ev.BTN_WEST, 2))
            check(False)
            os.write(raw_w, bytes([4, 80]))  # battery report
            check(False)
            os.write(event_w, ev.INPUT_EVENT.pack(0, 0, ev.EV_ABS, ev.ABS_X, 1000))
            check(True)
            os.write(raw_w, bytes([1]) + bytes(15))
            check(True)
        finally:
            for fd in (event_r, event_w, raw_r, raw_w):
                os.close(fd)

    def test_bluetooth_without_kernel_link_or_fresh_report_needs_lb_release(self):
        adapter = EvdevInput(load_input_profile())
        keys = {'lb': 0x134, 'b': 0x131, 'home': 0x133}
        dev = dict(bus=5, path='/test/pad', name='Xbox Wireless Controller', guid='one',
                   buttons=keys, input_report=(1, 16))
        clock = [1.]
        index = [-1]
        # Per tick: fresh report, pressed physical key.
        samples = ([(True, 0x131), (True, 0x133)] + [(True, 0x134)] * 12 +
                   [(False, 0x134)] * 10 + [(True, 0x134), (True, None), (True, 0x134)])
        observed = []
        def wait(*args):
            if index[0] >= 0:
                observed.append(adapter.latest())
            index[0] += 1
            clock[0] += .02
            if index[0] == len(samples):
                adapter.running = False
                index[0] -= 1
            return samples[index[0]][0]
        def pressed(fd, codes):
            return {code: samples[index[0]][1] == code for code in codes}
        adapter.running = True
        with patch.object(adapter, 'devices', return_value=([dev], '')), patch('os.open', return_value=12), \
                patch('os.close'), patch.object(ev, '_input_sysfs_dir', return_value=Path('/virtual/input')), \
                patch.object(ev, '_open_report_stream', return_value=None), \
                patch.object(ev, '_device_info', return_value=dev), patch.object(ev, '_wait_for_input', side_effect=wait), \
                patch.object(ev, '_pressed', side_effect=pressed), patch('time.monotonic', side_effect=lambda: clock[0]), \
                patch.object(ev, '_absinfo', return_value=(0, -100, 100, 0, 0, 0)):
            adapter._read_loop('bluetooth')
        self.assertEqual([(f['lb'], f['b'], f['home']) for f in observed[:3]],
                         [(False, True, False), (False, False, True), (True, False, False)])
        self.assertTrue(observed[13]['lb'])
        self.assertIsNone(observed[-4])
        self.assertIsNone(observed[-3])  # new report, still holding LB
        self.assertFalse(observed[-2]['lb'])
        self.assertTrue(observed[-1]['lb'])

    def test_late_bluetooth_pad_can_hold_without_events_and_disconnects_on_link_loss(self):
        adapter = EvdevInput(load_input_profile(), link_verified=True, mapping_verified=True)
        keys = {'lb': 0x134, 'b': 0x131, 'home': 0x133}
        pad = dict(bus=ev.BUS_BLUETOOTH, path='/dev/input/event18', guid='one', name='Xbox',
                   buttons=keys, input_report=(1, 16))
        # One 20 ms sample per tick. The kernel only emits events on changes.
        clock = 1.
        ticks = 0
        frames = []
        def wait(*args):
            nonlocal ticks, clock
            if ticks:
                frames.append(adapter.latest())
            ticks += 1
            clock += .02
            if ticks == 310:
                link.rename(link.with_name('old-link'))
            return ticks in (2, 105, 210, 211)

        def pressed(fd, codes):
            return {code: code == keys['lb'] and ticks >= 2 and ticks != 210 for code in codes}

        def axes(fd, code):
            value = 70 if code == ev.ABS_Y and ticks >= 105 else 0
            return (value, -100, 100, 0, 0, 0)

        def retry(delay):
            if frames:  # reader has already invalidated the frame after link loss
                self.assertIsNone(adapter.latest())
                self.assertIn('手柄断开', adapter.error)
                adapter.running = False

        with tempfile.TemporaryDirectory() as tmp:
            link = Path(tmp)/'hci0:256'
            link.mkdir()
            (link/'uevent').write_text('DEVTYPE=link\n')
            input_dir = link/'0005:045E:02E0.0008/input/input39'
            input_dir.mkdir(parents=True)
            with patch.object(adapter, 'devices', side_effect=[([], '等待蓝牙手柄'), ([pad], '')]) as scan, \
                    patch.object(adapter._stop_event, 'wait', side_effect=retry), \
                    patch('os.open', return_value=12), patch('os.close'), \
                    patch.object(ev, '_input_sysfs_dir', return_value=input_dir), \
                    patch.object(ev, '_open_report_stream') as open_raw, \
                    patch.object(ev, '_device_info', return_value=pad), \
                    patch.object(ev, '_wait_for_input', side_effect=wait), \
                    patch.object(ev, '_pressed', side_effect=pressed), patch.object(ev, '_absinfo', side_effect=axes), \
                    patch('time.monotonic', side_effect=lambda: clock):
                adapter.running = True
                adapter._read_loop('bluetooth')
        self.assertEqual([c.args for c in scan.call_args_list], [('bluetooth',), ('bluetooth',)])
        open_raw.assert_not_called()
        self.assertTrue(all(f and f['connected'] and f['link_verified'] for f in frames))
        self.assertTrue(all(f['lb'] for f in frames[1:209]))
        self.assertEqual(frames[103]['axes'][1], 0.)
        self.assertAlmostEqual(frames[208]['axes'][1], .7)
        self.assertFalse(frames[209]['lb'])
        self.assertTrue(frames[-1]['lb'])

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
