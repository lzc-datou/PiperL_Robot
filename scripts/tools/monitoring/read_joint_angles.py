#!/usr/bin/env python3
"""Passively display Piper-L joint positions and raw motor speeds.

Receive SocketCAN feedback with optional read-only MoveIt FK.
Never initialize an SDK driver or send control commands.
"""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import socket
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.piperl.deployment import load_driver_config

CAN_FRAME = struct.Struct("=IB3x8s")
POSITION_IDS = (0x2A5, 0x2A6, 0x2A7)
SPEED_IDS = tuple(range(0x251, 0x257))
CAN_FLAGS = 0xE0000000  # Extended, remote-request and error frames are not positions.
FILTER_MASK = 0xC00007FF  # Match standard CAN identifier and EFF/RTR flags.


def decode_positions(frame):
    """Return (first joint index, pair in degrees) for a valid position frame."""
    if len(frame) != CAN_FRAME.size:
        return None
    identifier, length, payload = CAN_FRAME.unpack(frame)
    if identifier & CAN_FLAGS or identifier not in POSITION_IDS or length != 8:
        return None
    # Piper feedback is two signed big-endian int32 values in 0.001 degrees.
    values = tuple(value / 1000.0 for value in struct.unpack(">ii", payload))
    return 2 * (identifier - POSITION_IDS[0]), values


def decode_speed(frame):
    """Return (joint index, raw motor rad/s), using installed SDK wire units."""
    if len(frame) != CAN_FRAME.size:
        return None
    identifier, length, payload = CAN_FRAME.unpack(frame)
    if identifier & CAN_FLAGS or identifier not in SPEED_IDS or length != 8:
        return None
    return identifier-SPEED_IDS[0], struct.unpack('>h', payload[:2])[0]*0.001


class JointFeedback:
    def __init__(self):
        self.degrees = [None] * 6
        self.received_at = [None] * 6
        self.speeds = [None] * 6
        self.speed_received_at = [None] * 6

    def update(self, frame, received_at):
        decoded = decode_positions(frame)
        if decoded is not None:
            first, values = decoded
            self.degrees[first:first + 2] = values
            self.received_at[first:first + 2] = [received_at, received_at]
        speed = decode_speed(frame)
        if speed is not None:
            index, value = speed
            self.speeds[index] = value
            self.speed_received_at[index] = received_at

    def format_table(self, now, *, channel, hz, stale_after):
        def age(timestamp):
            if timestamp is None:
                return '--'
            elapsed = max(0.0, now-timestamp)
            return f'{elapsed*1000:.1f}' + ('*' if elapsed > stale_after else '')

        headers = ['Joint', 'Angle(rad)', 'Angle(deg)', 'Motor(rad/s)', 'Pos age(ms)', 'Vel age(ms)']
        rows = []
        for index, degrees in enumerate(self.degrees):
            speed = self.speeds[index]
            rows.append([
                f'joint{index+1}',
                '--' if degrees is None else f'{math.radians(degrees):.6f}',
                '--' if degrees is None else f'{degrees:.3f}',
                '--' if speed is None else f'{speed:.3f}',
                age(self.received_at[index]), age(self.speed_received_at[index]),
            ])
        widths = [max(minimum, len(header), *(len(row[i]) for row in rows))
                  for i, (header, minimum) in enumerate(zip(headers, (6, 10, 10, 12, 8, 8)))]
        border = '+' + '+'.join('-'*(width+2) for width in widths) + '+'
        def row_text(row, header=False):
            cells = []
            for i, (value, width) in enumerate(zip(row, widths)):
                alignment = '^' if header else ('<' if i == 0 else '>')
                cells.append(f' {value:{alignment}{width}} ')
            return '|'+'|'.join(cells)+'|'
        lines = [
            f'Piper-L  |  {channel}  |  Motor monitor',
            datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S')
                + f'  |  {hz:g} Hz  |  read-only  |  Ctrl+C exit',
            '', border, row_text(headers, header=True), border,
            *(row_text(row) for row in rows), border,
        ]
        notices = []
        if any(value.endswith('*') for row in rows for value in row[4:]):
            notices.append(f'* STALE: last reading, older than {stale_after:g}s')
        if any('--' in row[4:] for row in rows):
            notices.append('--: waiting for feedback')
        if notices:
            lines.append(' | '.join(notices))
        return "\n".join(lines)


class MonitorDisplay:
    """Use a separate terminal screen; preserve ordinary text when redirected."""
    def __init__(self, output):
        self.output = output
        self.tty = output.isatty()
        self.active = False

    def start(self):
        if self.tty:
            self.active = True
            self.output.write('\033[?1049h\033[?25l\033[2J\033[H')
            self.output.flush()

    def draw(self, text):
        if not self.tty:
            self.output.write(text+'\n\n')
        else:
            try:
                columns, height = os.get_terminal_size(self.output.fileno())
            except (AttributeError, OSError, ValueError):
                columns, height = 80, 24
            lines = text.splitlines()
            required_columns = max(map(len, lines), default=0)+1
            required_height = len(lines)+1
            if columns < required_columns or height < required_height:
                # Do not wrap rows or silently hide status columns on small screens.
                lines = [f'Need {required_columns}x{required_height}; resize terminal.']
                lines = [line[:max(0, columns-1)] for line in lines[:max(0, height-1)]]
            self.output.write('\033[H'+'\r\n'.join('\033[2K'+line for line in lines)+'\033[J')
        self.output.flush()

    def close(self):
        if self.active:
            self.output.write('\033[?25h\033[?1049l')
            self.output.flush()
            self.active = False


def monitor(channel, *, hz=10.0, stale_after=0.5, seconds=None,
            socket_factory=None, clock=None, output=None, record_dir=None,
            points=True, frame="base_link", tcp="tcp_link", snapshot_dir=None):
    """Loop over received frames; injectable transport/clock support offline tests."""
    interactive_points = points and socket_factory is None and clock is None and output is None
    panel = None
    socket_factory = socket.socket if socket_factory is None else socket_factory
    clock = time if clock is None else clock
    output = sys.stdout if output is None else output
    feedback = JointFeedback()
    interval = 1.0 / hz
    display = MonitorDisplay(output)
    recorder = None
    status = "stopped"
    transport = socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        filters = b"".join(struct.pack("=II", identifier, FILTER_MASK) for identifier in POSITION_IDS+SPEED_IDS)
        transport.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        transport.bind((channel,))
        if record_dir is not None:
            from scripts.tools.monitoring.joint_range_recorder import JointRangeRecorder
            recorder = JointRangeRecorder(record_dir, channel)
        if interactive_points:
            from scripts.tools.monitoring.point_panel import PointPanel
            panel = PointPanel(ROOT, frame, tcp, snapshot_dir)
        display.start()
        started = clock.monotonic()
        deadline = None if seconds is None else started + seconds
        next_display = started
        while True:
            now = clock.monotonic()
            finished = deadline is not None and now >= deadline
            if now >= next_display or finished:
                table = feedback.format_table(now, channel=channel, hz=hz, stale_after=stale_after)
                if recorder is not None:
                    recorder.save(now, stale_after)
                    table += "\n\n" + recorder.format_ranges() + f"\nSaving: {recorder.directory}"
                if panel is not None:
                    table += panel.render(feedback, now, stale_after)
                display.draw(table)
                next_display = now + interval
            if finished:
                return
            wait = min(next_display - now, 0.1)
            if deadline is not None:
                wait = min(wait, deadline - now)
            transport.settimeout(max(wait, 0.000001))
            try:
                frame = transport.recv(CAN_FRAME.size)
            except socket.timeout:
                continue
            received_at = clock.monotonic()
            feedback.update(frame, received_at)
            if recorder is not None:
                decoded = decode_positions(frame)
                if decoded is not None:
                    recorder.update(*decoded, received_at, frame.hex())
    except KeyboardInterrupt:
        status = "interrupted"
        raise
    except BaseException:
        status = "error"
        raise
    finally:
        try:
            display.close()
        finally:
            transport.close()
            if panel is not None:
                panel.close()
            if recorder is not None:
                recorder.save(clock.monotonic(), stale_after, status, force=True)


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", help="Optional equality check against config/baseline.json")
    parser.add_argument("--hz", type=positive_number, default=10.0,
                        help="Display refresh rate, at most 100 Hz (default: 10)")
    parser.add_argument("--stale-after", type=positive_number, default=0.5,
                        help="Mark old feedback STALE after this many seconds (default: 0.5)")
    parser.add_argument("--seconds", type=positive_number,
                        help="Exit after this duration; default: run until Ctrl+C")
    parser.add_argument("--record-dir", type=Path,
                        help="Create a new directory and continuously save observed ranges in JSON/CSV")
    parser.add_argument("--no-points", action="store_true", help="Only show the original joint table")
    parser.add_argument("--frame", default="base_link", help="FK reference frame")
    parser.add_argument("--tcp", default="tcp_link", help="FK end-effector link")
    parser.add_argument("--snapshot-dir", type=Path, help="Point snapshot output root (default: outputs/points)")
    args = parser.parse_args(argv)
    if args.hz > 100:
        parser.error("--hz must be at most 100")
    try:
        channel = load_driver_config()["can_port"]
        if args.can is not None and args.can != channel:
            parser.error("--can differs from baseline.json; edit the shared configuration instead")
        options = {} if args.record_dir is None else {"record_dir": args.record_dir}
        if args.no_points: options["points"] = False
        if args.frame != "base_link": options["frame"] = args.frame
        if args.tcp != "tcp_link": options["tcp"] = args.tcp
        if args.snapshot_dir is not None: options["snapshot_dir"] = args.snapshot_dir
        monitor(channel, hz=args.hz, stale_after=args.stale_after, seconds=args.seconds, **options)
    except KeyboardInterrupt:
        print("\nStopped reading. No control commands were sent.")
    except (OSError, ValueError) as error:
        print(f"Cannot read joint feedback: {error}", file=sys.stderr)
        print("Check that the configured CAN interface exists, is UP, and is accessible.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
