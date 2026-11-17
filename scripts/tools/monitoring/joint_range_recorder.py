"""Save observed joint ranges only; no transport or robot control interfaces."""
import csv
from datetime import datetime
import io
import json
import math
from pathlib import Path
from zoneinfo import ZoneInfo


def timestamp():
    return datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="milliseconds")


class JointRangeRecorder:
    """Accumulate every valid received joint pair, without clipping to model limits."""

    def __init__(self, directory, channel):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.channel = channel
        self.started_at = timestamp()
        self.joints = [dict(name=f"joint{i}", samples=0, current=None,
                            observed_min=None, observed_max=None, received_at=None)
                       for i in range(1, 7)]
        self.valid_frames = 0
        self.last_checkpoint = None

    def update(self, first, degrees, received_at, frame_hex):
        captured_at = timestamp()
        for offset, value in enumerate(degrees):
            joint = self.joints[first + offset]
            point = dict(deg=value, rad=math.radians(value), captured_at=captured_at,
                         raw_frame_hex=frame_hex)
            joint["samples"] += 1
            joint["current"] = point
            joint["received_at"] = received_at
            for key, compare in (("observed_min", lambda a, b: a < b),
                                 ("observed_max", lambda a, b: a > b)):
                if joint[key] is None or compare(value, joint[key]["deg"]):
                    joint[key] = point.copy()
        self.valid_frames += 1

    def snapshot(self, now, stale_after, status="recording"):
        joints = []
        for joint in self.joints:
            received = joint["received_at"]
            age = None if received is None else max(0.0, now - received)
            joints.append({key: value for key, value in joint.items() if key != "received_at"})
            joints[-1].update(age_seconds=age, feedback_status=("WAITING" if age is None else
                              "OK" if age <= stale_after else "STALE"),
                              mechanical_limit_confirmed=False)
        return dict(schema_version=1, scope="observed_ranges_only", channel=self.channel,
                    timezone="Asia/Shanghai", started_at=self.started_at, updated_at=timestamp(),
                    status=status, valid_frames=self.valid_frames, joints=joints,
                    limitation="Observed extrema are not confirmed mechanical or safe operating limits. "
                               "Three joint-pair frames are not an atomic six-axis sample. "
                               "No model limits or zero offsets are changed.")

    def save(self, now, stale_after, status="recording", *, force=False):
        if not force and self.last_checkpoint is not None and now - self.last_checkpoint < 1.0:
            return
        summary = self.snapshot(now, stale_after, status)
        stream = io.StringIO(newline="")
        fields = ["joint", "observed_min_deg", "observed_max_deg", "observed_min_rad",
                  "observed_max_rad", "current_deg", "current_rad", "min_at", "max_at",
                  "samples", "feedback_status", "age_seconds", "mechanical_limit_confirmed"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for joint in summary["joints"]:
            low, high, current = (joint[key] or {} for key in ("observed_min", "observed_max", "current"))
            writer.writerow(dict(joint=joint["name"], observed_min_deg=low.get("deg"),
                                 observed_max_deg=high.get("deg"), observed_min_rad=low.get("rad"),
                                 observed_max_rad=high.get("rad"), current_deg=current.get("deg"),
                                 current_rad=current.get("rad"), min_at=low.get("captured_at"),
                                 max_at=high.get("captured_at"), samples=joint["samples"],
                                 feedback_status=joint["feedback_status"], age_seconds=joint["age_seconds"],
                                 mechanical_limit_confirmed=False))
        for name, content in (("summary.json", json.dumps(summary, ensure_ascii=False, indent=2) + "\n"),
                              ("ranges.csv", stream.getvalue())):
            temporary = self.directory / (name + ".tmp")
            temporary.write_text(content, encoding="utf-8")
            temporary.replace(self.directory / name)
        self.last_checkpoint = now

    def format_ranges(self):
        lines = ["Observed ranges (not confirmed limits):", "Joint       min(deg)     max(deg)     min(rad)     max(rad)"]
        for joint in self.joints:
            low, high = joint["observed_min"], joint["observed_max"]
            if low is None:
                lines.append(f"{joint['name']}           --           --           --           --")
            else:
                lines.append(f"{joint['name']} {low['deg']:12.3f} {high['deg']:12.3f} "
                             f"{low['rad']:12.6f} {high['rad']:12.6f}")
        return "\n".join(lines)
