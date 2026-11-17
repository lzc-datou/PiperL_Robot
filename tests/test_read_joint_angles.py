"""Joint monitor tests use in-memory sockets and clocks, never real CAN."""
from contextlib import redirect_stderr, redirect_stdout
import importlib
import io
import math
import os
import socket
import struct
import unittest
from unittest.mock import patch


def position_frame(identifier, first, second, *, length=8):
    """Build wire bytes independently of the reader's protocol constants."""
    return struct.pack("=IB3x8s", identifier, length, struct.pack(">ii", first, second))


def speed_frame(identifier, speed, *, length=8):
    return struct.pack('=IB3x8s', identifier, length, struct.pack('>h', speed)+b'\0'*6)


def table_rows(text):
    return [[cell.strip() for cell in line.split('|')[1:-1]]
            for line in text.splitlines() if line.startswith('| joint')]


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


class ReceiveOnlySocket:
    """A finite queue with timeout-driven time; every send method is forbidden."""
    def __init__(self, clock, events=(), *, fail_at=None):
        self.clock = clock
        self.events = list(events)
        self.fail_at = fail_at
        self.calls = []
        self.timeout = None
        self.closed = False
        self.recv_calls = 0

    def setsockopt(self, *args):
        self.calls.append(("setsockopt", args))
        if self.fail_at == "setsockopt":
            raise OSError("synthetic filter failure")

    def bind(self, address):
        self.calls.append(("bind", address))
        if self.fail_at == "bind":
            raise OSError("synthetic bind failure")

    def settimeout(self, timeout):
        self.calls.append(("settimeout", timeout))
        self.timeout = timeout

    def recv(self, size):
        self.calls.append(("recv", size))
        self.recv_calls += 1
        if self.events:
            self.clock.now += 0.02
            event = self.events.pop(0)
            if isinstance(event, BaseException):
                raise event
            return event
        # Advancing by the requested timeout models an idle receive without
        # sleeping or consulting an operating-system network interface.
        self.clock.now += self.timeout
        raise socket.timeout("synthetic idle receive")

    def send(self, *args, **kwargs):
        raise AssertionError("a read-only monitor must never send")

    sendall = send
    sendto = send
    sendmsg = send

    def close(self):
        self.calls.append(("close",))
        self.closed = True


class TerminalOutput(io.StringIO):
    def isatty(self):
        return True

    def fileno(self):
        return 1


class JointAngleReaderTests(unittest.TestCase):
    def setUp(self):
        # Guard the real default factory even when testing argument handling or
        # imports, so a future regression cannot contact hardware from tests.
        guard = patch("socket.socket", side_effect=AssertionError("real sockets forbidden"))
        self.real_socket = guard.start()
        self.addCleanup(guard.stop)
        self.tool = importlib.import_module("scripts.tools.monitoring.read_joint_angles")

    def run_monitor(self, events=(), *, seconds=0.35, fail_at=None, output=None, **kwargs):
        clock = FakeClock()
        transport = ReceiveOnlySocket(clock, events, fail_at=fail_at)
        factory_calls = []

        def factory(*args, **factory_kwargs):
            factory_calls.append((args, factory_kwargs))
            return transport

        output = io.StringIO() if output is None else output
        self.tool.monitor("configured_can", seconds=seconds, socket_factory=factory,
                          clock=clock, output=output, **kwargs)
        self.real_socket.assert_not_called()
        return transport, factory_calls, output.getvalue(), clock

    def test_signed_big_endian_millidegrees_and_all_three_joint_pairs(self):
        cases = (
            (0x2A5, 0, 180000, -90125, (180.0, -90.125)),
            (0x2A6, 2, -1, 12345, (-0.001, 12.345)),
            (0x2A7, 4, -2147483648, 2147483647, (-2147483.648, 2147483.647)),
        )
        feedback = self.tool.JointFeedback()
        for identifier, first, raw_a, raw_b, expected in cases:
            with self.subTest(identifier=hex(identifier)):
                frame = position_frame(identifier, raw_a, raw_b)
                self.assertEqual(self.tool.decode_positions(frame), (first, expected))
                feedback.update(frame, received_at=10.0 + first)
                self.assertEqual(tuple(feedback.degrees[first:first + 2]), expected)
                self.assertEqual(feedback.received_at[first:first + 2], [10.0 + first] * 2)
        self.assertEqual(feedback.degrees, [180.0, -90.125, -0.001, 12.345,
                                           -2147483.648, 2147483.647])

    def test_six_raw_motor_speeds_keep_sign_zero_and_wire_scaling(self):
        feedback = self.tool.JointFeedback()
        for i, raw in enumerate((-32768, -289, 0, 161, 1234, 32767)):
            frame = speed_frame(0x251+i, raw)
            self.assertEqual(self.tool.decode_speed(frame), (i, raw*.001))
            feedback.update(frame, received_at=10+i)
            self.assertEqual(feedback.speeds[i], raw*.001)
            self.assertEqual(feedback.speed_received_at[i], 10+i)
        self.assertEqual(feedback.received_at, [None]*6)
        invalid = [b'', speed_frame(0x251, 1)[:-1], speed_frame(0x251, 1)+b'\0',
                   speed_frame(0x251, 1, length=7), speed_frame(0x250, 1), speed_frame(0x257, 1)]
        invalid += [speed_frame(0x251 | flag, 1) for flag in (0x80000000, 0x40000000, 0x20000000)]
        for frame in invalid:
            self.assertIsNone(self.tool.decode_speed(frame))
            feedback.update(frame, received_at=99)
        self.assertEqual(feedback.speed_received_at, [10, 11, 12, 13, 14, 15])

    def test_position_and_motor_speed_have_independent_freshness(self):
        feedback = self.tool.JointFeedback()
        feedback.update(position_frame(0x2A5, 1000, 2000), received_at=10)
        feedback.update(speed_frame(0x251, -289), received_at=10.6)
        feedback.update(speed_frame(0x253, 0), received_at=10.6)
        def rows(now):
            return table_rows(feedback.format_table(now, channel='fake', hz=10, stale_after=.5))
        table = rows(10.6)
        self.assertEqual(table[0][3:], ['-0.289', '600.0*', '0.0'])
        self.assertEqual(table[1][3:], ['--', '600.0*', '--'])
        self.assertEqual(table[2][3:], ['0.000', '--', '0.0'])
        feedback.update(position_frame(0x2A5, 1000, 2000), received_at=11.2)
        self.assertEqual(rows(11.2)[0][3:], ['-0.289', '0.0', '600.0*'])

    def test_monitor_receives_speed_without_sending_or_requiring_position(self):
        transport, _, output, _ = self.run_monitor([speed_frame(0x256, -289)])
        self.assertIn('Motor(rad/s)', output)
        self.assertIn('-0.289', output)
        self.assertIn('Pos age(ms)', output)
        self.assertTrue(transport.closed)

    def test_terminal_restores_screen_and_cursor_on_exit_and_interrupt(self):
        for error in (None, KeyboardInterrupt(), OSError('receive failure')):
            output = TerminalOutput()
            clock = FakeClock()
            transport = ReceiveOnlySocket(clock, [] if error is None else [error])
            with patch.object(self.tool.os, 'get_terminal_size', return_value=os.terminal_size((80, 24))):
                def run():
                    self.tool.monitor('can0', seconds=.25, output=output, clock=clock,
                                      socket_factory=lambda *a: transport)
                if error is None:
                    run()
                else:
                    with self.assertRaises(type(error)):
                        run()
            text = output.getvalue()
            self.assertEqual(text.count('\033[?1049h'), 1)
            self.assertTrue(text.endswith('\033[?25h\033[?1049l'))
            self.assertIn('\033[2K| joint6', text)
            self.assertNotIn('resize terminal', text)
            self.assertTrue(transport.closed)

    def test_terminal_resize_does_not_wrap_or_hide_status_columns(self):
        output = TerminalOutput()
        display = self.tool.MonitorDisplay(output)
        text = self.tool.JointFeedback().format_table(10, channel='can0', hz=10, stale_after=.5)
        self.assertLessEqual(max(map(len, text.splitlines())), 79)
        self.assertEqual(len(text.splitlines()), 14)
        display.start()
        for size in ((40, 24), (80, 8)):
            output.seek(0); output.truncate()
            with patch.object(self.tool.os, 'get_terminal_size', return_value=os.terminal_size(size)):
                display.draw(text)
            self.assertIn('resize terminal', output.getvalue())
            self.assertNotIn('joint6', output.getvalue())
        output.seek(0); output.truncate()
        with patch.object(self.tool.os, 'get_terminal_size', return_value=os.terminal_size((80, 24))):
            display.draw(text)
        self.assertIn('joint6', output.getvalue())
        self.assertNotIn('resize terminal', output.getvalue())
        display.close()

    def test_invalid_flags_identifiers_lengths_and_truncated_frames_are_ignored(self):
        valid = position_frame(0x2A5, 90000, -90000)
        invalid = [position_frame(0x2A5 | flag, 1, 2)
                   for flag in (0x80000000, 0x40000000, 0x20000000)]
        invalid += [position_frame(identifier, 1, 2)
                    for identifier in (0x2A1, 0x2A4, 0x2A8, 0x150, 0x12A5)]
        invalid += [position_frame(0x2A5, 1, 2, length=length)
                    for length in (0, 7, 9, 15)]
        invalid += [b"", valid[:-1], valid + b"\0"]
        feedback = self.tool.JointFeedback()
        feedback.update(valid, received_at=1.0)
        for frame in invalid:
            with self.subTest(frame=frame.hex()):
                self.assertIsNone(self.tool.decode_positions(frame))
                feedback.update(frame, received_at=99.0)
                self.assertEqual(feedback.degrees, [90.0, -90.0, None, None, None, None])
                self.assertEqual(feedback.received_at, [1.0, 1.0, None, None, None, None])

    def test_table_converts_degrees_to_radians_for_six_axes(self):
        feedback = self.tool.JointFeedback()
        for identifier, values in ((0x2A5, (180000, -90000)),
                                   (0x2A6, (45000, -1000)),
                                   (0x2A7, (0, 12345))):
            feedback.update(position_frame(identifier, *values), received_at=10.0)
        table = feedback.format_table(10.1, channel="test_can", hz=10.0, stale_after=0.5)
        rows = table_rows(table)
        self.assertEqual(len(rows), 6)
        for index, (row, degrees) in enumerate(zip(rows, (180, -90, 45, -1, 0, 12.345)), 1):
            with self.subTest(joint=index):
                self.assertEqual(row[0], f"joint{index}")
                self.assertAlmostEqual(float(row[1]), math.radians(degrees), places=6)
                self.assertEqual(float(row[2]), degrees)
                self.assertEqual(row[3], '--')
                self.assertEqual(row[4], "100.0")

    def test_missing_pairs_and_stale_last_values_are_distinct(self):
        feedback = self.tool.JointFeedback()
        feedback.update(position_frame(0x2A5, 90000, -90000), received_at=10.0)
        boundary = feedback.format_table(10.5, channel="test_can", hz=10, stale_after=0.5)
        boundary_rows = table_rows(boundary)
        self.assertTrue(all(row[4] == "500.0" for row in boundary_rows[:2]))
        self.assertTrue(all(row[4:] == ['--', '--'] and row.count("--") == 5
                            for row in boundary_rows[2:]))
        stale = feedback.format_table(10.6, channel="test_can", hz=10, stale_after=0.5)
        stale_rows = table_rows(stale)
        self.assertEqual(stale_rows[0][2:4], ["90.000", "--"])
        self.assertEqual(stale_rows[0][4:], ["600.0*", "--"])
        self.assertEqual(stale_rows[1][2], "-90.000")
        # A fresh different pair must not make old joint values look fresh.
        feedback.update(position_frame(0x2A6, 1000, 2000), received_at=10.6)
        mixed = feedback.format_table(10.6, channel="test_can", hz=10, stale_after=0.5)
        mixed_rows = table_rows(mixed)
        self.assertEqual(mixed_rows[0][4], "600.0*")
        self.assertEqual(mixed_rows[2][4], "0.0")
        self.assertEqual(mixed_rows[4][4:], ['--', '--'])

    def test_monitor_defaults_only_receive_and_display_feedback(self):
        events = [position_frame(identifier, first, second)
                  for identifier, first, second in ((0x2A5, 90000, -45000),
                                                     (0x2A6, 0, 1000),
                                                     (0x2A7, -1000, 180000))]
        transport, factories, output, _ = self.run_monitor(events)
        self.assertEqual(factories, [((socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW), {})])
        self.assertIn(("bind", ("configured_can",)), transport.calls)
        self.assertGreaterEqual(transport.recv_calls, len(events))
        self.assertTrue(transport.closed)
        self.assertEqual(transport.calls[-1], ("close",))
        self.assertIn("10 Hz  |  read-only", output)
        self.assertIn("1.570796", output)
        self.assertIn("180.000", output)
        self.assertIn("--: waiting for feedback", output)
        self.assertNotIn("\033[", output)
        options = [call[1] for call in transport.calls if call[0] == "setsockopt"]
        self.assertEqual(len(options), 1)
        self.assertEqual(options[0][:2], (socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER))
        filters = list(struct.iter_unpack("=II", options[0][2]))
        self.assertEqual([identifier for identifier, _ in filters], [0x2A5, 0x2A6, 0x2A7, 0x251, 0x252, 0x253, 0x254, 0x255, 0x256])
        for _, mask in filters:
            self.assertEqual(mask & 0xC00007FF, 0xC00007FF)
        self.assertTrue(all(call[1] == 16 for call in transport.calls if call[0] == "recv"))

    def test_idle_monitor_keeps_refreshing_and_exits_after_requested_duration(self):
        transport, _, output, clock = self.run_monitor(seconds=0.35)
        self.assertGreaterEqual(output.count("Piper-L  |"), 3)
        self.assertGreaterEqual(output.count("--: waiting for feedback"), 3)
        self.assertGreater(transport.recv_calls, 0)
        self.assertGreaterEqual(clock.now, 100.35)
        self.assertLess(clock.now, 100.36)
        self.assertTrue(transport.closed)

    def test_monitor_marks_feedback_stale_while_no_more_frames_arrive(self):
        transport, _, output, _ = self.run_monitor(
            [position_frame(0x2A5, 1234, -5678)], seconds=0.35, stale_after=0.1)
        self.assertIn("1.234", output)
        self.assertIn("-5.678", output)
        self.assertIn("STALE", output)
        self.assertTrue(transport.closed)

    def test_monitor_closes_socket_on_receive_errors_and_ctrl_c(self):
        for error in (OSError("synthetic receive failure"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                clock = FakeClock()
                transport = ReceiveOnlySocket(clock, [error])
                with self.assertRaises(type(error)):
                    self.tool.monitor("fake", socket_factory=lambda *args: transport,
                                      clock=clock, output=io.StringIO())
                self.assertTrue(transport.closed)
                self.assertEqual(transport.calls[-1], ("close",))

    def test_monitor_closes_socket_if_filter_or_bind_fails(self):
        for fail_at in ("setsockopt", "bind"):
            with self.subTest(fail_at=fail_at):
                clock = FakeClock()
                transport = ReceiveOnlySocket(clock, fail_at=fail_at)
                with self.assertRaises(OSError):
                    self.tool.monitor("fake", socket_factory=lambda *args: transport,
                                      clock=clock, output=io.StringIO())
                self.assertTrue(transport.closed)
                self.assertEqual(transport.recv_calls, 0)

    def test_main_rejects_bad_numbers_before_monitor_or_configuration(self):
        invalid = [[option, value]
                   for option in ("--hz", "--stale-after", "--seconds")
                   for value in ("0", "-1", "nan", "inf", "-inf", "not-a-number")]
        invalid.extend((["--hz", "100.1"], ["--send"]))
        for argv in invalid:
            with self.subTest(argv=argv), redirect_stderr(io.StringIO()), \
                    patch.object(self.tool, "load_driver_config") as load, \
                    patch.object(self.tool, "monitor") as monitor:
                with self.assertRaises(SystemExit) as error:
                    self.tool.main(argv)
                self.assertEqual(error.exception.code, 2)
                load.assert_not_called()
                monitor.assert_not_called()
                self.real_socket.assert_not_called()

    def test_main_rejects_can_override_mismatch_before_opening_socket(self):
        with patch.object(self.tool, "load_driver_config", return_value={"can_port": "configured_can"}), \
                patch.object(self.tool, "monitor") as monitor, redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                self.tool.main(["--can", "some_other_can"])
        self.assertEqual(error.exception.code, 2)
        monitor.assert_not_called()
        self.real_socket.assert_not_called()

    def test_main_uses_shared_channel_and_passes_valid_display_options(self):
        with patch.object(self.tool, "load_driver_config", return_value={"can_port": "configured_can"}), \
                patch.object(self.tool, "monitor") as monitor:
            result = self.tool.main(["--can", "configured_can", "--hz", "100",
                                     "--stale-after", "0.2", "--seconds", "2"])
        self.assertEqual(result, 0)
        monitor.assert_called_once_with("configured_can", hz=100.0, stale_after=0.2, seconds=2.0)
        self.real_socket.assert_not_called()

    def test_main_ctrl_c_is_clean_and_transport_error_returns_failure(self):
        with patch.object(self.tool, "load_driver_config", return_value={"can_port": "configured_can"}):
            with patch.object(self.tool, "monitor", side_effect=KeyboardInterrupt()), \
                    redirect_stdout(io.StringIO()) as output:
                self.assertEqual(self.tool.main([]), 0)
            self.assertIn("Stopped reading", output.getvalue())
            with patch.object(self.tool, "monitor", side_effect=OSError("no fake interface")), \
                    redirect_stderr(io.StringIO()) as errors:
                self.assertEqual(self.tool.main([]), 1)
            self.assertIn("Cannot read joint feedback", errors.getvalue())
            self.real_socket.assert_not_called()

    def test_help_is_safe_without_configuration_or_socket(self):
        with patch.object(self.tool, "load_driver_config") as load, \
                patch.object(self.tool, "monitor") as monitor, redirect_stdout(io.StringIO()) as output:
            with self.assertRaises(SystemExit) as error:
                self.tool.main(["--help"])
        self.assertEqual(error.exception.code, 0)
        self.assertIn("--stale-after", output.getvalue())
        self.assertIn("--seconds", output.getvalue())
        load.assert_not_called()
        monitor.assert_not_called()
        self.real_socket.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
