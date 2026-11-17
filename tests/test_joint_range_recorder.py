"""Observed-range recording tests use temporary files and fake CAN sockets only."""
import csv
import importlib
import io
import json
import math
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch

from tests.test_read_joint_angles import FakeClock, ReceiveOnlySocket, position_frame


class JointRangeRecorderTests(unittest.TestCase):
    def setUp(self):
        guard = patch("socket.socket", side_effect=AssertionError("real sockets forbidden"))
        self.real_socket = guard.start()
        self.addCleanup(guard.stop)
        self.module = importlib.import_module("scripts.tools.monitoring.joint_range_recorder")
        self.reader = importlib.import_module("scripts.tools.monitoring.read_joint_angles")
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name) / "new_recording"

    def recorder(self):
        return self.module.JointRangeRecorder(self.directory, "fake_can")

    def add_frame(self, recorder, identifier, first, second, *, received_at=10.0):
        frame = position_frame(identifier, first, second)
        decoded = self.reader.decode_positions(frame)
        self.assertIsNotNone(decoded)
        recorder.update(*decoded, received_at, frame.hex())
        return frame

    def read_files(self):
        summary = json.loads((self.directory / "summary.json").read_text(encoding="utf-8"))
        with (self.directory / "ranges.csv").open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        return summary, rows

    def run_monitor(self, events=(), *, seconds=0.3, **kwargs):
        clock = FakeClock()
        transport = ReceiveOnlySocket(clock, events)
        calls = []

        def factory(*args, **factory_kwargs):
            calls.append((args, factory_kwargs))
            return transport

        output = io.StringIO()
        self.reader.monitor("fake_can", hz=1, seconds=seconds,
                            socket_factory=factory, clock=clock, output=output,
                            record_dir=self.directory, **kwargs)
        self.real_socket.assert_not_called()
        return transport, calls, output.getvalue(), clock

    def test_all_pairs_accumulate_every_frame_in_degrees_and_radians_without_clipping(self):
        recorder = self.recorder()
        frames = ((0x2A5, -155000, -1989), (0x2A6, -176000, -128000),
                  (0x2A7, -90555, -171000), (0x2A5, 156000, 196000),
                  (0x2A6, 1924, 128000), (0x2A7, 90555, 171000),
                  (0x2A5, 149610, 1000), (0x2A6, -1000, 2000),
                  (0x2A7, 3000, 0))
        for identifier, first, second in frames:
            self.add_frame(recorder, identifier, first, second)
        snapshot = recorder.snapshot(10.1, 0.5)
        self.assertEqual(snapshot["valid_frames"], 9)
        expected = ((-155, 156, 149.61), (-1.989, 196, 1),
                    (-176, 1.924, -1), (-128, 128, 2),
                    (-90.555, 90.555, 3), (-171, 171, 0))
        for index, (joint, values) in enumerate(zip(snapshot["joints"], expected), 1):
            with self.subTest(joint=index):
                self.assertEqual(joint["name"], f"joint{index}")
                self.assertEqual(joint["samples"], 3)
                self.assertEqual(joint["feedback_status"], "OK")
                for key, degrees in zip(("observed_min", "observed_max", "current"), values):
                    self.assertEqual(joint[key]["deg"], degrees)
                    self.assertAlmostEqual(joint[key]["rad"], math.radians(degrees), places=12)

    def test_no_feedback_keeps_unknown_values_and_unconfirmed_limits(self):
        summary = self.recorder().snapshot(100.0, 0.5)
        self.assertEqual(summary["scope"], "observed_ranges_only")
        self.assertEqual(summary["valid_frames"], 0)
        self.assertEqual(summary["timezone"], "Asia/Shanghai")
        self.assertIn("not confirmed mechanical", summary["limitation"])
        self.assertIn("not an atomic", summary["limitation"])
        for joint in summary["joints"]:
            self.assertEqual(joint["samples"], 0)
            for key in ("current", "observed_min", "observed_max", "age_seconds"):
                self.assertIsNone(joint[key])
            self.assertEqual(joint["feedback_status"], "WAITING")
            self.assertIs(joint["mechanical_limit_confirmed"], False)

    def test_freshness_is_per_pair_and_stale_values_remain_historical_observations(self):
        recorder = self.recorder()
        self.add_frame(recorder, 0x2A5, 90000, -45000, received_at=10.0)
        self.add_frame(recorder, 0x2A6, 1000, 2000, received_at=10.5)
        boundary = recorder.snapshot(10.5, 0.5)
        self.assertEqual(boundary["joints"][0]["feedback_status"], "OK")
        summary = recorder.snapshot(10.6, 0.5)
        self.assertEqual([joint["feedback_status"] for joint in summary["joints"]],
                         ["STALE", "STALE", "OK", "OK", "WAITING", "WAITING"])
        self.assertAlmostEqual(summary["joints"][0]["age_seconds"], 0.6)
        self.assertAlmostEqual(summary["joints"][2]["age_seconds"], 0.1)
        self.assertEqual(summary["joints"][0]["current"]["deg"], 90.0)
        self.assertEqual(summary["joints"][0]["observed_min"]["deg"], 90.0)
        self.assertTrue(all(joint["mechanical_limit_confirmed"] is False
                            for joint in summary["joints"]))

    def test_extrema_keep_their_original_time_and_raw_frame_after_further_updates(self):
        recorder = self.recorder()
        with patch.object(self.module, "timestamp", side_effect=("first", "second", "tie")):
            first = self.add_frame(recorder, 0x2A5, -1000, 2000)
            second = self.add_frame(recorder, 0x2A5, 3000, -4000)
            self.add_frame(recorder, 0x2A5, -1000, -4000)
        low = recorder.joints[0]["observed_min"]
        high = recorder.joints[0]["observed_max"]
        self.assertEqual((low["captured_at"], low["raw_frame_hex"]), ("first", first.hex()))
        self.assertEqual((high["captured_at"], high["raw_frame_hex"]), ("second", second.hex()))
        self.assertEqual(recorder.joints[1]["observed_min"]["captured_at"], "second")
        self.assertEqual(recorder.joints[1]["current"]["captured_at"], "tie")

    def test_json_and_csv_agree_including_unknowns_counts_times_and_units(self):
        recorder = self.recorder()
        self.add_frame(recorder, 0x2A5, -155000, 196000)
        self.add_frame(recorder, 0x2A5, 156000, -1989, received_at=10.2)
        self.add_frame(recorder, 0x2A7, 90555, 0, received_at=10.2)
        recorder.save(10.3, 0.5, "stopped", force=True)
        summary, rows = self.read_files()
        self.assertEqual(summary["status"], "stopped")
        self.assertEqual(len(rows), 6)
        for row, joint in zip(rows, summary["joints"]):
            with self.subTest(joint=joint["name"]):
                self.assertEqual(row["joint"], joint["name"])
                self.assertEqual(int(row["samples"]), joint["samples"])
                self.assertEqual(row["feedback_status"], joint["feedback_status"])
                self.assertEqual(row["mechanical_limit_confirmed"], "False")
                self.assertIs(joint["mechanical_limit_confirmed"], False)
                for prefix, key in (("observed_min", "observed_min"),
                                    ("observed_max", "observed_max"), ("current", "current")):
                    for unit in ("deg", "rad"):
                        actual = row[f"{prefix}_{unit}"]
                        if joint[key] is None:
                            self.assertEqual(actual, "")
                        else:
                            self.assertAlmostEqual(float(actual), joint[key][unit], places=12)
                for column, key in (("min_at", "observed_min"), ("max_at", "observed_max")):
                    self.assertEqual(row[column], "" if joint[key] is None else joint[key]["captured_at"])
                if joint["age_seconds"] is None:
                    self.assertEqual(row["age_seconds"], "")
                else:
                    self.assertAlmostEqual(float(row["age_seconds"]), joint["age_seconds"])
        self.assertFalse(list(self.directory.glob("*.tmp")))

    def test_existing_directory_is_never_reused_or_overwritten(self):
        self.directory.mkdir()
        existing = self.directory / "summary.json"
        existing.write_text("previous recording", encoding="utf-8")
        with self.assertRaises(FileExistsError):
            self.recorder()
        self.assertEqual(existing.read_text(encoding="utf-8"), "previous recording")
        self.assertEqual(list(self.directory.iterdir()), [existing])
        self.real_socket.assert_not_called()

    def test_forced_final_save_keeps_last_frames_between_periodic_checkpoints(self):
        recorder = self.recorder()
        recorder.save(10.0, 0.5)
        self.add_frame(recorder, 0x2A5, 1234, -5678, received_at=10.1)
        recorder.save(10.2, 0.5)
        self.assertEqual(self.read_files()[0]["valid_frames"], 0)
        recorder.save(10.3, 0.5, "interrupted", force=True)
        final = self.read_files()[0]
        self.assertEqual(final["status"], "interrupted")
        self.assertEqual(final["valid_frames"], 1)
        self.assertEqual(final["joints"][0]["current"]["deg"], 1.234)

    def test_monitor_records_extrema_between_display_ticks_using_only_receive_operations(self):
        frames = [position_frame(identifier, first, second) for identifier, first, second in
                  ((0x2A5, -155000, 196000), (0x2A5, 156000, -1989),
                   (0x2A5, 0, 0), (0x2A6, -176000, 128000), (0x2A7, 90555, -171000))]
        transport, factories, output, _ = self.run_monitor(frames)
        summary, _ = self.read_files()
        self.assertEqual(factories, [((socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW), {})])
        self.assertEqual(summary["valid_frames"], 5)
        self.assertEqual(summary["status"], "stopped")
        self.assertEqual(summary["joints"][0]["observed_min"]["deg"], -155)
        self.assertEqual(summary["joints"][0]["observed_max"]["deg"], 156)
        self.assertEqual(summary["joints"][0]["current"]["deg"], 0)
        self.assertEqual(summary["joints"][0]["samples"], 3)
        self.assertIn("Observed ranges (not confirmed limits)", output)
        self.assertIn(str(self.directory), output)
        self.assertTrue(transport.closed)
        self.assertEqual(transport.calls[-1], ("close",))
        self.assertTrue(all(call[0] in ("setsockopt", "bind", "settimeout", "recv", "close")
                            for call in transport.calls))

    def test_monitor_ignores_invalid_frames_when_recording(self):
        valid = position_frame(0x2A5, 1234, -5678)
        events = [position_frame(0x150, 90000, 90000),
                  position_frame(0x2A5 | 0x80000000, 90000, 90000),
                  position_frame(0x2A5 | 0x40000000, 90000, 90000),
                  position_frame(0x2A5 | 0x20000000, 90000, 90000),
                  position_frame(0x2A5, 90000, 90000, length=7), valid[:-1], valid]
        transport, _, _, _ = self.run_monitor(events)
        summary, _ = self.read_files()
        self.assertEqual(summary["valid_frames"], 1)
        self.assertEqual(summary["joints"][0]["samples"], 1)
        self.assertEqual(summary["joints"][0]["current"]["deg"], 1.234)
        self.assertEqual(summary["joints"][2]["samples"], 0)
        self.assertTrue(transport.closed)

    def test_idle_monitor_final_save_records_waiting_without_fabricated_limits(self):
        transport, _, _, _ = self.run_monitor()
        summary, rows = self.read_files()
        self.assertEqual(summary["status"], "stopped")
        self.assertEqual(summary["valid_frames"], 0)
        self.assertTrue(all(joint["feedback_status"] == "WAITING" for joint in summary["joints"]))
        self.assertTrue(all(row["observed_min_deg"] == row["observed_max_deg"] == "" for row in rows))
        self.assertTrue(transport.closed)

    def test_monitor_finalizes_after_keyboard_interrupt_and_receive_error(self):
        for error, status in ((KeyboardInterrupt(), "interrupted"),
                              (OSError("synthetic receive failure"), "error")):
            with self.subTest(status=status):
                self.directory = self.directory.parent / status
                clock = FakeClock()
                transport = ReceiveOnlySocket(clock, [position_frame(0x2A7, 90555, -171000), error])
                with self.assertRaises(type(error)):
                    self.reader.monitor("fake_can", socket_factory=lambda *args: transport,
                                        clock=clock, output=io.StringIO(), record_dir=self.directory)
                summary, rows = self.read_files()
                self.assertEqual(summary["status"], status)
                self.assertEqual(summary["valid_frames"], 1)
                self.assertEqual(summary["joints"][4]["observed_max"]["deg"], 90.555)
                self.assertEqual(rows[5]["observed_min_deg"], "-171.0")
                self.assertTrue(transport.closed)
                self.assertEqual(transport.calls[-1], ("close",))
                self.real_socket.assert_not_called()

    def test_monitor_existing_recording_is_not_overwritten_and_socket_closes(self):
        self.directory.mkdir()
        sentinel = self.directory / "summary.json"
        sentinel.write_text("original", encoding="utf-8")
        clock = FakeClock()
        transport = ReceiveOnlySocket(clock)
        with self.assertRaises(FileExistsError):
            self.reader.monitor("fake_can", socket_factory=lambda *args: transport,
                                clock=clock, output=io.StringIO(), record_dir=self.directory)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "original")
        self.assertEqual(transport.recv_calls, 0)
        self.assertTrue(transport.closed)
        self.real_socket.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
