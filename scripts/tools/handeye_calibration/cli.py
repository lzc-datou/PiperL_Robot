"""Launch official camera/detector/calibrator with project-specific parameters."""
import argparse
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime

import yaml

TOOL = Path(__file__).resolve().parent
ROOT = TOOL.parents[2]


def config():
    return yaml.safe_load((TOOL / 'config/config.yaml').read_text())


def marker_size(cfg):
    size = cfg.get('marker_size_m')
    if not isinstance(size, (int, float)) or isinstance(size, bool) or not 0.005 <= size <= 0.3:
        raise SystemExit('请先实测 ArUco 黑色方块外边长，在 config/config.yaml 填写 marker_size_m（米，不含白边）。')
    return float(size)


def ros_exec(args):
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable, '/opt/ros/jazzy/bin/ros2', *args])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check', 'camera', 'marker', 'view', 'calibrate', 'board', 'offline-test'])
    args = parser.parse_args()
    cfg = config()
    name = cfg['camera_name']
    camera_frame = f'{name}_color_optical_frame'
    topic = f"/{cfg['camera_namespace']}/{name}/color"
    if args.command == 'check':
        for module in ('rclpy', 'cv2', 'tf_transformations', 'scipy', 'handeye_calibration_ros.agx_arm_calibration'):
            importlib.import_module(module)
            print('OK import', module)
        for package in ('aruco_ros', 'realsense2_camera', 'realsense2_description', 'handeye_calibration_ros'):
            subprocess.run([sys.executable, '/opt/ros/jazzy/bin/ros2', 'pkg', 'prefix', package], check=True)
        print('依赖检查通过（未连接相机或 CAN，未检查实时数据）。')
        print('实测标记边长:', cfg['marker_size_m'], 'm；null 时禁止启动检测和采样。')
    elif args.command == 'camera':
        command = ['launch', 'realsense2_camera', 'rs_launch.py',
                   f'camera_namespace:={cfg["camera_namespace"]}', f'camera_name:={name}',
                   'enable_color:=true', 'enable_depth:=true', 'enable_infra1:=false',
                   'enable_infra2:=false', 'enable_gyro:=false', 'enable_accel:=false',
                   f'depth_module.color_profile:={cfg["color_profile"]}',
                   f'depth_module.depth_profile:={cfg["color_profile"]}',
                   'depth_module.color_format:=RGB8', 'depth_module.enable_auto_exposure:=true']
        if cfg['camera_serial']:
            command.append(f'serial_no:=_{cfg["camera_serial"]}')
        ros_exec(command)
    elif args.command == 'marker':
        size = marker_size(cfg)
        ros_exec(['run', 'aruco_ros', 'single', '--ros-args', '-r', '__node:=aruco_single',
                  '-p', f'marker_id:={cfg["marker_id"]}', '-p', f'marker_size:={size}',
                  '-p', 'image_is_rectified:=false', '-p', f'camera_frame:={camera_frame}',
                  '-p', f'reference_frame:={camera_frame}', '-p', 'marker_frame:=handeye_marker',
                  '-r', f'/image:={topic}/image_raw', '-r', f'/camera_info:={topic}/camera_info'])
    elif args.command == 'view':
        ros_exec(['run', 'rqt_image_view', 'rqt_image_view', '/aruco_single/result'])
    elif args.command == 'calibrate':
        marker_size(cfg)
        if not sys.stdin.isatty():
            raise SystemExit('请在交互终端中运行采样。')
        output = ROOT / 'outputs/handeye' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output.mkdir(parents=True)
        metadata = {**cfg, 'mode': 'eye_in_hand', 'camera_frame': camera_frame,
                    'transform': f"{cfg['end_frame']}_T_{camera_frame}",
                    'convention': 'p_end = R_end_camera @ p_camera + t_end_camera; meters; quaternion xyzw',
                    'dependencies': json.loads((TOOL / 'config/dependencies.json').read_text())}
        (output / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        print('结果目录:', output, flush=True)
        # Reuse the upstream solver and interactive sampling; add read-only input checks and sample export.
        from collector import run
        run(cfg, output, camera_frame)
    elif args.command == 'board':
        from board import generate
        generate(TOOL / 'assets', cfg['marker_id'])
    elif args.command == 'offline-test':
        from offline_test import run
        run()


if __name__ == '__main__':
    main()
