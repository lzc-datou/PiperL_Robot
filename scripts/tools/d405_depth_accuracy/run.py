"""Compare ArUco 100 corners from D405 and RGB PnP with probe references."""

import argparse
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import yaml

from scripts.tools.handeye_calibration.quality import checked_transform

from .geometry import CORNERS, METHODS, Detector
from .report import RunReport, probe_statistics


TOOL = Path(__file__).resolve().parent
ROOT = TOOL.parents[2]
FEEDBACK_END_FRAME = 'tcp_link'


def mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} 必须为键值映射')
    return value


def yaml_mapping(path):
    return mapping(yaml.safe_load(path.read_text(encoding='utf-8-sig')), path.name)


def positive(value, name):
    if type(value) not in (int, float) or not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} 必须填写正的有限数')
    return float(value)


def vector(value, length, name):
    if not isinstance(value, list) or len(value) != length or any(type(x) not in (int, float) for x in value):
        raise ValueError(f'{name} 需要 {length} 个数值')
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f'{name} 含无效数值')
    return result


def read_config(path, reference_path):
    config = yaml_mapping(path)
    reference = yaml_mapping(reference_path)
    if config.get('marker_id') != 100 or reference.get('marker_id') != 100:
        raise ValueError('本工具的参考文件和实验配置均应使用 ArUco 100')
    if reference.get('units') != 'mm' or not isinstance(reference.get('frame_id'), str) or not reference['frame_id']:
        raise ValueError('参考坐标需填写基座 frame_id，单位填写 mm')
    corners = mapping(reference.get('corners'), 'corners')
    if set(corners) != set(CORNERS):
        raise ValueError('参考点必须包含 TL、TR、BR、BL 四角')
    reference_mm = np.stack([vector(corners[corner], 3, corner) for corner in CORNERS])
    if len(np.unique(reference_mm, axis=0)) != 4:
        raise ValueError('四个参考角点必须是不同的 XYZ')
    dictionary = config.get('dictionary')
    if not isinstance(dictionary, str) or not dictionary.startswith('DICT_'):
        raise ValueError('dictionary 须填写生成 ID 100 时使用的 OpenCV ArUco 字典名')
    for key in ('camera_namespace', 'robot_pose_topic'):
        if not isinstance(config.get(key), str) or not config[key].startswith('/') or config[key].endswith('/'):
            raise ValueError(f'{key} 须填写绝对 ROS 路径')
    for key in ('marker_size_mm', 'capture_timeout_s', 'display_hz', 'max_rgb_depth_skew_ms',
                'max_pose_gap_ms', 'input_wait_ms', 'pnp_max_reprojection_px'):
        positive(config.get(key), key)
    if config['max_pose_gap_ms'] > 1000 or config['max_rgb_depth_skew_ms'] > config['input_wait_ms']:
        raise ValueError('位姿插值最大间隔须 ≤ 1000 ms，RGB-D 时间差须 ≤ 等待时限')
    if type(config.get('capture_frames')) is not int or config['capture_frames'] < 1:
        raise ValueError('capture_frames 须填写正整数')
    depth = mapping(config.get('depth'), 'depth')
    for key in ('min_mm', 'max_mm', 'inlier_threshold_mm', 'min_inlier_ratio', 'min_span_fraction'):
        positive(depth.get(key), f'depth.{key}')
    inset = depth.get('inset_fraction')
    if (depth['min_mm'] >= depth['max_mm'] or depth['min_inlier_ratio'] > 1 or depth['min_span_fraction'] > 1
            or type(inset) not in (int, float) or not 0 <= inset < 1):
        raise ValueError('depth 的量程、比例或内缩配置无效')
    for key, minimum in (('pixel_stride', 1), ('min_inliers', 3)):
        if type(depth.get(key)) is not int or depth[key] < minimum:
            raise ValueError(f'depth.{key} 须填写不小于 {minimum} 的整数')
    if 'scale_m_per_unit' not in depth:
        raise ValueError('depth.scale_m_per_unit 缺失，可填写 null 使用 ROS 标准单位')
    if depth['scale_m_per_unit'] is not None:
        positive(depth['scale_m_per_unit'], 'depth.scale_m_per_unit')
    return config, reference, reference_mm


def read_handeye(directory, base_frame):
    directory = directory.resolve()
    session_bytes = (directory / 'session.json').read_bytes()
    matrix_bytes = (directory / 'T_end_camera.json').read_bytes()
    session = mapping(json.loads(session_bytes), 'session.json')
    end_frame, camera_frame = session.get('end_frame'), session.get('camera_frame')
    if (session.get('mode') != 'eye_in_hand' or session.get('base_frame') != base_frame
            or end_frame != FEEDBACK_END_FRAME
            or not isinstance(camera_frame, str) or not camera_frame.endswith('_optical_frame')
            or session.get('transform') != f'{end_frame}_T_{camera_frame}'):
        raise ValueError(f'手眼标定应为 eye_in_hand，使用 {base_frame}、{FEEDBACK_END_FRAME} '
                         '与彩色光学系，并记录末端_T_相机方向')
    end_from_camera = checked_transform(json.loads(matrix_bytes))
    return dict(directory=str(directory), base_frame=base_frame, camera_frame=camera_frame,
                end_frame=end_frame,
                matrix_sha256=hashlib.sha256(matrix_bytes).hexdigest(),
                session_sha256=hashlib.sha256(session_bytes).hexdigest(),
                end_T_camera=end_from_camera)


@contextmanager
def terminal_keys():
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)

        def read():
            return os.read(fd, 64).decode(errors='ignore').lower() if select.select([fd], [], [], 0)[0] else ''

        yield read
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def display(report, frame, message):
    def xyz(point):
        return ' '.join(f'{item:9.3f}' for item in point)

    lines = [f'ArUco 100  |  {report.metadata["reference"]["frame_id"]}  |  mm',
             's：采一组   q / Ctrl+C：退出', f'输出：{report.directory}']
    if message:
        lines.append(message)
    if report.started_at is not None:
        lines.append(f'第 {report.group} 组  {len(report.frames)} / {report.capture_frames} 帧')
    if frame is None:
        lines.append('等待 RGB 图像。请核对相机话题和 ROS Domain。')
    else:
        depth, pnp = (frame.estimates[method] for method in METHODS)
        lines.append(f'D405 {depth.status}   PnP {pnp.status}')
        lines.append('角点  测针参考 XYZ                   D405 XYZ                        PnP XYZ                         误差 D405 / PnP')
        for index, corner in enumerate(CORNERS):
            reference = report.reference_mm[index]
            columns, errors = [], []
            for estimate in (depth, pnp):
                if estimate.points_m is None:
                    columns.append(f'{"--":>29}')
                    errors.append('    --')
                else:
                    point = estimate.points_m[index] * 1000
                    columns.append(xyz(point))
                    errors.append(f'{np.linalg.norm(point - reference):6.3f}')
            lines.append(f'{corner:>3}  {xyz(reference)}  {columns[0]}  {columns[1]}   {errors[0]} / {errors[1]}')
    prefix = '\033[H\033[J' if sys.stdout.isatty() and os.environ.get('TERM') != 'dumb' else ''
    print(prefix + '\n'.join(lines), flush=True)


def preview(config, calibration, detector, report_metadata, reference_mm):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    from .observer import Observer

    rclpy.init(args=[])
    node, report = None, None
    try:
        report = RunReport(ROOT / 'outputs' / TOOL.name, report_metadata, reference_mm,
                           config['capture_frames'], config['capture_timeout_s'])
        node = Observer(config, calibration, detector, report)
        last_display, received, frame, message = 0.0, 0.0, None, ''
        with terminal_keys() as read_keys:
            while rclpy.ok():
                keys = read_keys()
                if 'q' in keys:
                    break
                if 's' in keys:
                    message = report.start()
                rclpy.spin_once(node, timeout_sec=0.005)
                # Consume at most one queued observation per spin so feedback and TF can catch up.
                measurement = node.next_frame()
                if measurement is not None:
                    frame, received = measurement
                    message = report.record(frame, received) or message
                message = report.check_timeout() or message
                now = time.monotonic()
                if now - last_display >= 1 / config['display_hz']:
                    display(report, frame if now - received < 0.5 else None, message)
                    last_display = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if report is not None:
                report.close()
                print(f'结果目录：{report.directory}', flush=True)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--handeye-dir', required=True, type=Path, help='本次手眼标定结果目录')
    parser.add_argument('--config', type=Path, default=TOOL / 'config/experiment.yaml')
    parser.add_argument('--reference', type=Path, default=TOOL / 'config/reference_points.yaml')
    parser.add_argument('--check', action='store_true', help='离线核对输入及 OpenCV ArUco 依赖')
    args = parser.parse_args()
    try:
        config, reference, reference_mm = read_config(args.config, args.reference)
        calibration = read_handeye(args.handeye_dir, reference['frame_id'])
        probe = probe_statistics(args.reference.with_name('probe_samples.csv'), reference_mm)

        import cv2

        detector = Detector(cv2, config['dictionary'], config['marker_id'])
        if args.check:
            print(f'输入检查通过  |  ArUco 100  |  {config["dictionary"]}  |  {config["marker_size_mm"]:g} mm')
            print(f'基座 {reference["frame_id"]}，彩色光学系 {calibration["camera_frame"]}')
            print(f'手眼标定：{calibration["directory"]}')
            print(f'T_end_camera（将彩色光学系中的点转换到 {calibration["end_frame"]}）')
            print(calibration['end_T_camera'])
            return
        if not sys.stdin.isatty():
            raise ValueError('请在 Ubuntu 的交互终端中运行，以使用 s / q 采集或退出')
        saved_calibration = {key: value.tolist() if isinstance(value, np.ndarray) else value
                             for key, value in calibration.items()}
        metadata = dict(schema_version=1, config=config, reference=reference,
                        reference_file=str(args.reference.resolve()), probe_repeatability=probe,
                        handeye=saved_calibration, opencv_version=cv2.__version__,
                        ros_domain_id=os.environ.get('ROS_DOMAIN_ID'))
        preview(config, calibration, detector, metadata, reference_mm)
    except (AttributeError, ImportError, KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
        parser.exit(2, f'd405_depth_accuracy: {error}\n')


if __name__ == '__main__':
    main()
