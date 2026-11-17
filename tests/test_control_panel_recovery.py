"""Regression of controller stop/reset/mode/enable transitions with fake CAN."""
import itertools
import socket
import struct
import time
import unittest
from unittest.mock import Mock, patch

from scripts.tools.control_panel.core import CanFeedback, CAN_FRAME
from scripts.tools.control_panel.recovery import State
from scripts.tools.control_panel.backend import Backend
from tests.test_control_panel import wait_for


class Controller:
    def __init__(self):
        self.stopped = True
        self.mode = False
        self.enabled = False
        self.sent = []
        self.mode_count = self.enable_count = 0
        self.block_enable = self.block_mode = False
        self.readers = []

    def send_frame(self, identifier, payload):
        self.sent.append((identifier, payload))
        if identifier == 0x150:
            self.stopped = payload[0] == 1
            self.mode = self.enabled = False
            self.mode_count = self.enable_count = 0
        elif identifier == 0x151 and not self.stopped:
            self.mode_count += 1
            self.mode = self.mode_count >= 2 and not self.block_mode
        elif identifier == 0x471 and not self.stopped:
            self.enable_count += 1
            self.enabled = self.mode and self.enable_count >= 2 and not self.block_enable

    def reader(self, channel):
        owner = self
        class Reader:
            closed = False
            ids = itertools.cycle([0x2A1, *range(0x261, 0x267)])
            def receive(self):
                time.sleep(0.001)
                identifier = next(self.ids)
                if identifier == 0x2A1:
                    data = [int(owner.mode), int(owner.stopped), int(owner.mode), 0, 0, 0, 0, 0]
                else:
                    data = [0, 0, 0, 0, 0, 0x40 if owner.enabled else 0, 0, 0]
                return identifier, bytes(data)
            def close(self):
                self.closed = True
        reader = Reader()
        self.readers.append(reader)
        return reader

    def close(self):
        pass


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        for name, value in [('CanTransport', lambda channel: self.controller),
                            ('CanFeedback', self.controller.reader)]:
            guard = patch('scripts.tools.control_panel.backend.'+name, value)
            guard.start()
            self.addCleanup(guard.stop)
        guard = patch('socket.socket', side_effect=AssertionError('real CAN forbidden'))
        guard.start()
        self.addCleanup(guard.stop)
        self.backend = Backend('can0', discover=False)
        self.addCleanup(self.backend.close)
        self.backend.refresh_interfaces({'can0': (True, 1000000, 'ERROR-ACTIVE')})

    def run_action(self, action):
        self.backend.submit(action)
        wait_for(lambda: not self.backend.busy, 6)

    def test_stop_enable_is_rejected_until_explicit_reset_then_home_recovers(self):
        self.run_action('enable')
        self.assertEqual(self.controller.sent, [])
        self.assertTrue(any('仍在急停' in m for m in self.backend.events.queue))
        self.run_action('reset')
        self.run_action('enable')
        self.assertTrue(self.controller.mode)
        self.assertTrue(self.controller.enabled)
        self.assertGreaterEqual(self.controller.enable_count, 2)
        self.assertFalse(any(identifier in (0x155, 0x156, 0x157) for identifier, _ in self.controller.sent))
        self.run_action('home')
        self.assertEqual(self.controller.sent[-3:], [(i, bytes(8)) for i in (0x155, 0x156, 0x157)])
        self.assertEqual(sum(identifier == 0x150 for identifier, _ in self.controller.sent), 1)
        self.assertTrue(all(reader.closed for reader in self.controller.readers))

    def test_home_rejects_disabled_motors_without_auto_enable_or_reset(self):
        self.controller.stopped = False
        self.run_action('home')
        self.assertEqual(self.controller.sent, [])
        self.assertTrue(any('未使能' in m for m in self.backend.events.queue))

    def test_enable_timeout_reports_missing_enable_and_never_sends_target(self):
        self.controller.stopped = False
        self.controller.block_enable = True
        with patch('scripts.tools.control_panel.recovery.ENABLE_TIMEOUT', 0.35):
            self.run_action('enable')
        self.assertTrue(any('恢复超时' in m and '未使能' in m for m in self.backend.events.queue))
        self.assertFalse(any(i in (0x150, 0x155, 0x156, 0x157) for i, _ in self.controller.sent))
        self.assertTrue(all(reader.closed for reader in self.controller.readers))

    def test_home_mode_failure_never_sends_joint_targets(self):
        self.controller.stopped = False
        self.controller.enabled = True
        self.controller.block_mode = True
        with patch('scripts.tools.control_panel.recovery.RESPONSE_TIMEOUT', 0.3):
            self.run_action('home')
        self.assertTrue(any('模式未就绪' in m for m in self.backend.events.queue))
        self.assertTrue(all(i == 0x151 for i, _ in self.controller.sent))

    def test_emergency_stop_interrupts_enable_retries(self):
        self.controller.stopped = False
        self.controller.block_enable = True
        self.backend.submit('enable')
        wait_for(lambda: self.controller.enable_count >= 1)
        self.backend.emergency_stop()
        wait_for(lambda: any(i == 0x150 for i, _ in self.controller.sent))
        wait_for(lambda: not self.backend.busy)
        self.assertEqual(self.controller.sent[-1], (0x150, bytes.fromhex('0100000000000000')))
        self.assertTrue(all(reader.closed for reader in self.controller.readers))

    def test_port_switch_interrupts_feedback_wait_and_future_retries(self):
        self.controller.stopped = False
        self.controller.block_enable = True
        self.backend.submit('enable')
        wait_for(lambda: self.controller.enable_count >= 1)
        self.backend.select('can1')
        sent = list(self.controller.sent)
        wait_for(lambda: all(reader.closed for reader in self.controller.readers))
        self.assertEqual(self.controller.sent, sent)

    def test_absent_feedback_sends_no_blind_mode_or_enable(self):
        reader = Mock()
        reader.receive.side_effect = lambda: (time.sleep(0.005), None)[1]
        with patch('scripts.tools.control_panel.backend.CanFeedback', return_value=reader), \
             patch('scripts.tools.control_panel.recovery.RESPONSE_TIMEOUT', 0.05):
            self.run_action('enable')
        self.assertEqual(self.controller.sent, [])
        self.assertTrue(any('0x2A1' in m for m in self.backend.events.queue))
        reader.close.assert_called_once()

    def test_stop_interrupts_reset_settling_wait(self):
        self.backend.submit('reset')
        wait_for(lambda: self.controller.sent)
        start = time.monotonic()
        self.backend.emergency_stop()
        wait_for(lambda: not self.backend.busy)
        wait_for(lambda: self.controller.stopped)
        self.assertLess(time.monotonic() - start, 0.5)
        self.assertEqual(self.controller.sent[-1][1][0], 1)
        self.assertFalse(any('请再使能' in m for m in self.backend.events.queue))


class FeedbackTests(unittest.TestCase):
    def test_kernel_filter_receives_status_as_data_instead_of_error_frames(self):
        sock = Mock()
        reader = CanFeedback('fake', socket_factory=lambda *args: sock)
        self.addCleanup(reader.close)
        _, option, packed_filters = sock.setsockopt.call_args.args
        self.assertEqual(option, socket.CAN_RAW_FILTER)
        filters = list(struct.iter_unpack('=II', packed_filters))

        # Linux selects the error receive list when CAN_ERR_FLAG is in the
        # mask. A fake recv that ignores setsockopt cannot catch this bug.
        def kernel_delivers(identifier):
            return any(not mask & socket.CAN_ERR_FLAG and
                       identifier & mask == expected & mask
                       for expected, mask in filters)

        for identifier in (0x2A1, *range(0x261, 0x267)):
            self.assertTrue(kernel_delivers(identifier))
            self.assertFalse(kernel_delivers(identifier | socket.CAN_EFF_FLAG))
            self.assertFalse(kernel_delivers(identifier | socket.CAN_RTR_FLAG))
        self.assertFalse(kernel_delivers(0x251))
        sock.send.assert_not_called()

    def test_filters_short_and_flagged_frames_and_closes_failed_bind(self):
        sock = Mock()
        reader = CanFeedback('fake', socket_factory=lambda *args: sock)
        sock.send.assert_not_called()
        frames = [b'bad', CAN_FRAME.pack(0x800002A1, 8, bytes(8)),
                  CAN_FRAME.pack(0x2A1, 7, bytes(8)), socket.timeout(),
                  CAN_FRAME.pack(0x2A1, 8, bytes(8))]
        sock.recv.side_effect = frames
        for _ in range(4):
            self.assertIsNone(reader.receive())
        self.assertEqual(reader.receive(), (0x2A1, bytes(8)))
        reader.close()
        sock.close.assert_called_once()
        sock = Mock()
        sock.bind.side_effect = OSError('missing interface')
        with self.assertRaises(OSError):
            CanFeedback('fake', socket_factory=lambda *args: sock)
        sock.close.assert_called_once()

    def test_expired_and_partial_motor_feedback_cannot_confirm_enable(self):
        state = State()
        with patch('scripts.tools.control_panel.recovery.time.monotonic', return_value=1.0):
            state.update((0x2A1, bytes([1, 0, 1, 0, 0, 0, 0, 0])))
            for identifier in range(0x261, 0x266):
                state.update((identifier, bytes([0, 0, 0, 0, 0, 0x40, 0, 0])))
            self.assertFalse(state.enabled())
            state.update((0x266, bytes([0, 0, 0, 0, 0, 0x40, 0, 0])))
            self.assertTrue(state.enabled())
        with patch('scripts.tools.control_panel.recovery.time.monotonic', return_value=2.0):
            self.assertFalse(state.enabled())
            self.assertIsNone(state.status_data())


if __name__ == '__main__':
    unittest.main()
