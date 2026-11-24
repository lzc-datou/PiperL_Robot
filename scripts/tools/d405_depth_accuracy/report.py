"""Write per-corner samples, per-group statistics and reproducible run metadata."""

import csv
from datetime import datetime
from pathlib import Path
import time

import numpy as np
import yaml

from .geometry import CORNERS, METHODS


QUALITY_FIELDS = (
    'depth_points', 'depth_inliers', 'depth_inlier_ratio', 'depth_plane_rms_mm',
    'depth_span_fraction', 'pnp_candidates', 'pnp_reprojection_rms_px', 'pnp_second_rms_px',
)
AXES = ('x', 'y', 'z')
DETAIL_FIELDS = (
    'group', 'sample', 'rgb_stamp_ns', 'depth_stamp_ns', 'depth_skew_ms', 'pose_gap_ms',
    'corner', 'u_px', 'v_px', *(f'ref_{axis}_mm' for axis in AXES),
    *(field for method in METHODS for field in (
        f'{method}_status', *(f'{method}_{axis}_mm' for axis in AXES),
        *(f'{method}_delta_{axis}_mm' for axis in AXES), f'{method}_error_mm',
    )), *QUALITY_FIELDS,
)
SUMMARY_FIELDS = (
    'group', 'group_status', 'corner', 'method', 'population', 'frames', 'valid_frames',
    'valid_fraction', *(f'mean_delta_{axis}_mm' for axis in AXES),
    'error_rmse_mm', 'error_p95_mm', 'jitter_rms_mm',
)


def probe_statistics(path, reference_mm):
    path = Path(path)
    if not path.exists():
        return {}
    grouped = {corner: [] for corner in CORNERS}
    with path.open(encoding='utf-8-sig', newline='') as file:
        rows = csv.DictReader(file)
        if not rows.fieldnames or not {'corner', 'sample', 'x_mm', 'y_mm', 'z_mm'} <= set(rows.fieldnames):
            raise ValueError(f'{path.name} 缺少触测列')
        for row in rows:
            point = np.array([float(row[f'{axis}_mm']) for axis in AXES])
            if row['corner'] not in grouped or not np.isfinite(point).all():
                raise ValueError(f'{path.name}:{rows.line_num} 的角点编号或坐标无效')
            grouped[row['corner']].append(point)
    result = {}
    for index, (corner, points) in enumerate(grouped.items()):
        if points:
            points = np.asarray(points)
            mean = np.mean(points, axis=0)
            jitter = np.sqrt(np.mean(np.sum((points - mean) ** 2, axis=1)))
            result[corner] = dict(samples=len(points), mean_xyz_mm=mean.tolist(),
                                  mean_minus_reference_xyz_mm=(mean - reference_mm[index]).tolist(),
                                  jitter_rms_mm=float(jitter) if len(points) > 1 else None)
    return result


def detail_rows(frame, reference_mm, group, sample):
    common = dict(group=group, sample=sample, rgb_stamp_ns=frame.rgb_stamp_ns,
                  depth_stamp_ns=frame.depth_stamp_ns, pose_gap_ms=frame.pose_gap_ms)
    if frame.depth_stamp_ns is not None:
        common['depth_skew_ms'] = (frame.depth_stamp_ns - frame.rgb_stamp_ns) / 1e6
    for estimate in frame.estimates.values():
        common.update(estimate.metrics)
    for index, corner in enumerate(CORNERS):
        reference = reference_mm[index]
        row = dict(common, corner=corner)
        row.update(zip((f'ref_{axis}_mm' for axis in AXES), reference))
        if frame.pixels is not None:
            row['u_px'], row['v_px'] = frame.pixels[index]
        for method, estimate in frame.estimates.items():
            row[f'{method}_status'] = estimate.status
            if estimate.points_m is not None:
                point_mm = estimate.points_m[index] * 1000
                delta = point_mm - reference
                row.update(zip((f'{method}_{axis}_mm' for axis in AXES), point_mm))
                row.update(zip((f'{method}_delta_{axis}_mm' for axis in AXES), delta))
                row[f'{method}_error_mm'] = np.linalg.norm(delta)
        yield row


def summarize(group, status, frames, reference_mm):
    for index, corner in enumerate(CORNERS):
        for method in METHODS:
            for population in ('all', 'paired'):
                positions = []
                for frame in frames:
                    estimate = frame.estimates[method]
                    if estimate.points_m is None:
                        continue
                    if population == 'paired' and any(e.points_m is None for e in frame.estimates.values()):
                        continue
                    positions.append(estimate.points_m[index] * 1000)
                count = len(positions)
                row = dict(group=group, group_status=status, corner=corner, method=method,
                           population=population, frames=len(frames), valid_frames=count,
                           valid_fraction=count / len(frames) if frames else None)
                if count:
                    positions = np.asarray(positions)
                    deltas = positions - reference_mm[index]
                    errors = np.linalg.norm(deltas, axis=1)
                    row.update(zip((f'mean_delta_{axis}_mm' for axis in AXES), np.mean(deltas, axis=0)))
                    row['error_rmse_mm'] = np.sqrt(np.mean(errors ** 2))
                    row['error_p95_mm'] = np.percentile(errors, 95)
                if count > 1:
                    residuals = positions - np.mean(positions, axis=0)
                    row['jitter_rms_mm'] = np.sqrt(np.mean(np.sum(residuals ** 2, axis=1)))
                yield row


class RunReport:
    def __init__(self, root, metadata, reference_mm, capture_frames, timeout_s):
        now = datetime.now().astimezone()
        self.directory = Path(root) / now.strftime('%Y%m%d_%H%M%S_%f_id100')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.reference_mm = reference_mm
        self.capture_frames = capture_frames
        self.timeout_s = timeout_s
        self.metadata = dict(metadata, started_at=now.isoformat(), camera_streams={}, groups=[])
        self.frames = []
        self.started_at = None
        self.annotation_saved = False
        self.details_file = (self.directory / 'corners.csv').open('w', newline='', encoding='utf-8-sig')
        self.details = csv.DictWriter(self.details_file, fieldnames=DETAIL_FIELDS)
        self.details.writeheader()
        self.details_file.flush()
        with (self.directory / 'summary.csv').open('w', newline='', encoding='utf-8-sig') as file:
            csv.DictWriter(file, fieldnames=SUMMARY_FIELDS).writeheader()
        self.write_metadata()

    @property
    def group(self):
        return len(self.metadata['groups']) + 1

    def write_metadata(self):
        path = self.directory / 'run.yaml'
        temporary = path.with_suffix('.tmp')
        temporary.write_text(yaml.safe_dump(self.metadata, allow_unicode=True, sort_keys=False), encoding='utf-8')
        temporary.replace(path)

    def set_camera_stream(self, name, camera):
        self.metadata['camera_streams'][name] = camera
        self.write_metadata()

    def start(self):
        if self.started_at is not None:
            return '当前组正在采集'
        self.started_at = time.monotonic()
        return f'开始采集第 {self.group} 组'

    def record(self, frame, received_at):
        if self.started_at is None or received_at < self.started_at:
            return None
        self.frames.append(frame)
        self.details.writerows(detail_rows(frame, self.reference_mm, self.group, len(self.frames)))
        self.details_file.flush()
        if len(self.frames) >= self.capture_frames:
            return self.finish('complete')
        return None

    def check_timeout(self):
        if self.started_at is not None and time.monotonic() - self.started_at > self.timeout_s:
            return self.finish('timeout')
        return None

    def finish(self, status):
        if self.started_at is None:
            return None
        count, group = len(self.frames), self.group
        with (self.directory / 'summary.csv').open('a', newline='', encoding='utf-8') as file:
            writer = csv.DictWriter(file, fieldnames=SUMMARY_FIELDS)
            writer.writerows(summarize(group, status, self.frames, self.reference_mm))
        stamps = [frame.rgb_stamp_ns for frame in self.frames if frame.rgb_stamp_ns > 0]
        intervals = np.diff(stamps)
        intervals = intervals[intervals > 0]
        stream_fps = float(1e9 / np.median(intervals)) if len(intervals) else None
        self.metadata['groups'].append(dict(group=group, status=status, frames=count,
                                            observed_rgb_fps=stream_fps))
        self.write_metadata()
        self.frames.clear()
        self.started_at = None
        return f'第 {group} 组已保存，{count}/{self.capture_frames} 帧（{status}）'

    def close(self):
        try:
            self.finish('interrupted')
            self.metadata['ended_at'] = datetime.now().astimezone().isoformat()
            self.write_metadata()
        finally:
            self.details_file.close()
