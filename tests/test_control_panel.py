"""Raw CAN panel tests; no real interfaces or control frames are used."""
import struct
import threading
import time
import unittest
from unittest.mock import patch
from scripts.tools.control_panel.core import CanTransport, command_frame, activate_interface
from scripts.tools.control_panel.backend import Backend


def wait_for(predicate, seconds=2):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError('operation timed out')


class HealthyFeedback:
    def __init__(self, channel):
        self.channel = channel
        self.index = 0
        self.closed = False

    def receive(self):
        time.sleep(0.001)
        self.index = (self.index + 1) % 7
        if self.index == 0:
            return 0x2A1, bytes([1, 0, 1, 0, 0, 0, 0, 0])
        return 0x260+self.index, bytes([0, 0, 0, 0, 0, 0x40, 0, 0])

    def close(self):
        self.closed = True


class ProtocolTests(unittest.TestCase):
    def test_active_interface_is_reused_without_configuration(self):
        with patch('scripts.tools.control_panel.core.inspect_interface', return_value=(True, 1000000, 'ERROR-ACTIVE')), \
             patch('subprocess.Popen', side_effect=AssertionError('must not reconfigure')):
            activate_interface('fake_can')

    def test_wrong_bitrate_and_bus_off_never_reset_a_live_interface(self):
        for status in ((True, 500000, 'ERROR-ACTIVE'), (True, 1000000, 'BUS-OFF')):
            with patch('scripts.tools.control_panel.core.inspect_interface', return_value=status), \
                 patch('subprocess.Popen', side_effect=AssertionError('must not reconfigure')):
                with self.assertRaises(RuntimeError):
                    activate_interface('fake_can')

    def test_exact_user_frames(self):
        for name, expected in [('stop', (0x150, '0100000000000000')),
                               ('reset', (0x150, '0200000000000000')),
                               ('enable', (0x471, '0702000000000000'))]:
            identifier, length, payload = struct.unpack('=IB3x8s', command_frame(name))
            self.assertEqual((identifier, length, payload.hex()), (expected[0], 8, expected[1]))
        with self.assertRaises(KeyError):
            command_frame('home')


    def test_transport_sends_only_one_exact_frame_per_click(self):
        class FakeSocket:
            def __init__(self): self.sent=[]; self.closed=False
            def setsockopt(self, *args): pass
            def settimeout(self, *args): pass
            def bind(self, address): self.address=address
            def send(self, frame): self.sent.append(frame); return len(frame)
            def close(self): self.closed=True
            def recv(self, *args): raise AssertionError('must not read feedback')
        sock=FakeSocket()
        transport=CanTransport('fake_can', socket_factory=lambda *args: sock)
        self.assertEqual(sock.sent, [])
        for action in ('enable','stop','reset'):
            transport.send(action)
        self.assertEqual(sock.sent, [command_frame(a) for a in ('enable','stop','reset')])
        transport.close()
        self.assertTrue(sock.closed)

    def test_socket_closed_after_failed_bind(self):
        from unittest.mock import Mock
        sock=Mock()
        sock.bind.side_effect=OSError('failed bind')
        with self.assertRaises(OSError):
            CanTransport('fake_can', socket_factory=lambda *args: sock)
        sock.close.assert_called_once()


class BackendTests(unittest.TestCase):
    def setUp(self):
        guard=patch('socket.socket', side_effect=AssertionError('real sockets forbidden'))
        guard.start(); self.addCleanup(guard.stop)
        self.backend=Backend('fake_can', demo=True, discover=False)
        self.addCleanup(self.backend.close)
        self.backend.submit('connect')
        wait_for(lambda: self.backend.connected and not self.backend.busy)

    def test_demo_reset_and_enable_without_real_feedback(self):
        for action in ('reset','enable'):
            self.backend.submit(action)
            wait_for(lambda: not self.backend.busy)
        messages=[]
        while not self.backend.events.empty(): messages.append(self.backend.events.get_nowait())
        self.assertTrue(any('150#0200000000000000' in m for m in messages))
        self.assertTrue(any('471#0702000000000000' in m for m in messages))
        self.assertFalse(hasattr(self.backend, 'bridge'))
        self.assertFalse(hasattr(self.backend, 'feedback'))

    def test_stop_bypasses_pending_work_and_invalidates_older_click(self):
        entered,release=threading.Event(),threading.Event()
        original=self.backend.operate
        def blocked(action,epoch):
            entered.set(); release.wait(2)
            original(action,epoch)
        self.backend.operate=blocked
        self.backend.submit('enable')
        self.assertTrue(entered.wait(1))
        self.backend.emergency_stop()
        wait_for(lambda: any('150#01' in m for m in list(self.backend.events.queue)))
        release.set()
        wait_for(lambda: not self.backend.busy)
        messages=list(self.backend.events.queue)
        self.assertFalse(any('471#' in m for m in messages))
        self.backend.operate=original
        self.backend.submit('enable')
        wait_for(lambda: not self.backend.busy)
        self.assertTrue(any('471#' in m for m in list(self.backend.events.queue)))


class DiscoveryAndMotionTests(unittest.TestCase):
    healthy = (True, 1000000, 'ERROR-ACTIVE')

    def setUp(self):
        from unittest.mock import Mock
        self.socket_guard = patch('socket.socket', side_effect=AssertionError('real socket forbidden'))
        self.socket_guard.start()
        self.addCleanup(self.socket_guard.stop)
        self.factory = patch('scripts.tools.control_panel.backend.CanTransport', side_effect=lambda port: Mock(port=port))
        self.transport_factory = self.factory.start()
        self.addCleanup(self.factory.stop)
        feedback = patch('scripts.tools.control_panel.backend.CanFeedback', HealthyFeedback)
        feedback.start()
        self.addCleanup(feedback.stop)
        self.activation = patch('scripts.tools.control_panel.backend.activate_interface', side_effect=AssertionError('no activation expected'))
        self.activation.start()
        self.addCleanup(self.activation.stop)

    def backend(self, channel=None):
        backend = Backend(channel, discover=False)
        self.addCleanup(backend.close)
        return backend

    def test_single_active_port_auto_attaches_without_any_command(self):
        backend = self.backend()
        backend.refresh_interfaces({'can0': self.healthy})
        self.assertEqual(backend.channel, 'can0')
        self.assertTrue(backend.connected)
        self.transport_factory.assert_called_once_with('can0')
        backend.transport.send_frame.assert_not_called()
        backend.refresh_interfaces({'can0': self.healthy})
        self.transport_factory.assert_called_once()

    def test_multiple_ports_require_selection_and_missing_saved_port_never_falls_back(self):
        backend = self.backend()
        backend.refresh_interfaces({'can0': self.healthy, 'can1': self.healthy})
        self.assertIsNone(backend.channel)
        backend.refresh_interfaces({'can1': self.healthy})
        self.assertIsNone(backend.channel)
        backend.select('can1')
        backend.refresh_interfaces({'can1': self.healthy})
        self.assertTrue(backend.connected)
        saved = self.backend('can9')
        saved.refresh_interfaces({'can0': self.healthy})
        self.assertEqual(saved.channel, 'can9')
        self.assertFalse(saved.connected)

    def test_down_bad_rate_bus_off_do_not_auto_attach_or_reconfigure(self):
        for status in ((False, 1000000, 'STOPPED'), (True, 500000, 'ERROR-ACTIVE'),
                       (True, 1000000, 'BUS-OFF'), (True, 1000000, 'ERROR-PASSIVE')):
            backend = self.backend('can0')
            backend.refresh_interfaces({'can0': status})
            self.assertFalse(backend.connected)
            self.assertEqual(backend.snapshot()['can_connect'], not status[0])
        self.transport_factory.assert_not_called()

    def test_home_sends_exact_order_and_no_enable_or_calibration(self):
        backend = self.backend('can0')
        backend.refresh_interfaces({'can0': self.healthy})
        backend.submit('home')
        wait_for(lambda: not backend.busy)
        actual = [(call.args[0], call.args[1].hex()) for call in backend.transport.send_frame.call_args_list]
        self.assertEqual(actual, [(0x151, '01010a0000000000'),
                                  (0x155, '0000000000000000'),
                                  (0x156, '0000000000000000'),
                                  (0x157, '0000000000000000')])
        self.assertIn('回零指令已发送', list(backend.events.queue))

    def test_stop_cancels_remaining_home_frames(self):
        backend = self.backend('can0')
        backend.refresh_interfaces({'can0': self.healthy})
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        original = backend.send_frame
        def blocked(identifier, payload, epoch):
            if identifier == 0x155:
                entered.set()
                release.wait(2)
            return original(identifier, payload, epoch)
        backend.send_frame = blocked
        backend.submit('home')
        self.assertTrue(entered.wait(1))
        backend.emergency_stop()
        wait_for(lambda: any('150#01' in m for m in list(backend.events.queue)))
        release.set()
        wait_for(lambda: not backend.busy)
        self.assertEqual([c.args[0] for c in backend.transport.send_frame.call_args_list], [0x151, 0x150])
        self.assertNotIn('回零指令已发送', list(backend.events.queue))

    def test_switch_cancels_pending_home_and_stop_without_sending_on_new_port(self):
        backend = self.backend('can0')
        ports = {'can0': self.healthy, 'can1': self.healthy}
        backend.refresh_interfaces(ports)
        old = backend.transport
        old_epoch = backend.epoch
        backend.select('can1')
        backend.refresh_interfaces(ports)
        old.close.assert_called_once()
        with self.assertRaises(InterruptedError):
            backend.operate('home', old_epoch)
        backend._stop(old_epoch)
        old.send_frame.assert_not_called()
        backend.transport.send_frame.assert_not_called()
        backend.submit('enable')
        wait_for(lambda: not backend.busy)
        self.assertEqual(backend.transport.send_frame.call_args.args[0], 0x471)

    def test_unplug_discards_queued_operation_and_does_not_replay_on_reconnect(self):
        backend = self.backend('can0')
        backend.refresh_interfaces({'can0': self.healthy})
        epoch, old = backend.epoch, backend.transport
        backend.refresh_interfaces({})
        self.assertFalse(backend.connected)
        old.close.assert_called_once()
        backend.refresh_interfaces({'can0': self.healthy})
        with self.assertRaises(InterruptedError):
            backend.operate('enable', epoch)
        backend.transport.send_frame.assert_not_called()

    def test_port_enumeration_filters_non_can(self):
        import json
        from types import SimpleNamespace
        from scripts.tools.control_panel.core import list_interfaces
        links = [{'ifname': 'eth0', 'flags': ['UP']},
                 {'ifname': 'can7', 'ifindex': 77, 'flags': ['UP'], 'linkinfo': {'info_kind': 'can',
                  'info_data': {'state': 'ERROR-ACTIVE', 'bittiming': {'bitrate': 1000000}}}}]
        with patch('subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=json.dumps(links))):
            self.assertEqual(list_interfaces(), {'can7': (*self.healthy, 77)})

    def test_same_name_replug_rebinds_socket_and_cancels_old_commands(self):
        backend = self.backend('can0')
        backend.refresh_interfaces({'can0': (*self.healthy, 19)})
        old, epoch = backend.transport, backend.epoch
        backend.refresh_interfaces({'can0': (*self.healthy, 23)})
        self.assertTrue(backend.connected)
        self.assertEqual(backend.interface_index, 23)
        old.close.assert_called_once()
        self.assertIsNot(backend.transport, old)
        with self.assertRaises(InterruptedError):
            backend.operate('enable', epoch)
        backend.transport.send_frame.assert_not_called()

    def test_scan_started_before_connection_cannot_disconnect_new_socket(self):
        backend = self.backend('can0')
        down = {'can0': (False, 1000000, 'STOPPED', 19)}
        backend.refresh_interfaces(down)
        scan_revision = backend.link_revision
        # A manual activation finishes while the previous scan is in flight.
        backend.refresh_interfaces({'can0': (*self.healthy, 19)})
        transport = backend.transport
        backend.refresh_interfaces(down, scan_revision)
        self.assertTrue(backend.connected)
        self.assertIs(backend.transport, transport)
        transport.close.assert_not_called()

    def test_unselected_port_change_does_not_repeat_selected_port_error(self):
        backend = self.backend('can0')
        down = (False, 1000000, 'STOPPED')
        backend.refresh_interfaces({'can0': down, 'can1': self.healthy})
        backend.refresh_interfaces({'can0': down})
        messages = list(backend.events.queue)
        self.assertEqual(sum('未启用' in m for m in messages), 1)


if __name__ == '__main__':
    unittest.main()
