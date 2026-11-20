"""Launch official camera/detector/calibrator with project-specific parameters."""
import argparse
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import signal
import threading
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import yaml

TOOL = Path(__file__).resolve().parent
ROOT = TOOL.parents[2]
PREVIEW_ROOT = ROOT / 'outputs/handeye_preview'
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.task.common.process_cleanup import stop_process_group


def config():
    return yaml.safe_load((TOOL / 'config/config.yaml').read_text())


def marker_size(cfg):
    size = cfg.get('marker_size_m')
    if not isinstance(size, (int, float)) or isinstance(size, bool) or not 0.005 <= size <= 0.3:
        raise SystemExit('请先实测 ArUco 黑色方块外边长，在 config/config.yaml 填写 marker_size_m（米，不含白边）。')
    return float(size)


def target_settings(cfg):
    kind = cfg.get('target', 'aruco')
    if kind == 'aruco':
        return kind, cfg['marker_pose_topic'], '/aruco_single/result'
    if kind == 'chessboard':
        return kind, '/handeye/chessboard/pose', '/handeye/chessboard/result'
    raise SystemExit('target 必须为 chessboard 或 aruco。')


def validate_target(cfg, kind):
    if kind == 'aruco':
        marker_size(cfg)
    else:
        from chessboard import board_settings
        return board_settings(cfg.get('chessboard'))


def ros_exec(args):
    sys.stdout.flush()
    os.execv(sys.executable, [sys.executable, '/opt/ros/jazzy/bin/ros2', *args])


def preview_commands(with_depth=False, tracking=False):
    # Call the existing CLI inside this environment, not run.sh's env -i again.
    commands = {name: [sys.executable, '-u', str(TOOL / 'cli.py'), name]
                for name in ('camera', 'marker', 'view')}
    if not with_depth:
        commands['camera'].append('--color-only')
    if tracking:
        commands['marker'].append('--tracking')
    return commands


def tracking_clock_options(directory):
    # rs_launch.py forwards typed YAML options to the sensor, including
    # dynamic SDK parameters absent from its list of launch arguments.
    options = directory / 'camera-options.yaml'
    options.write_text(yaml.safe_dump({'depth_module.global_time_enabled': False}))
    return options


def preview_preflight(cfg, kind):
    """Check GUI/dependencies and conflicting ROS endpoints before opening a device."""
    if not os.environ.get('DISPLAY'):
        raise SystemExit('preview 需要图形桌面或有效的 DISPLAY；无图形环境时请分别使用 camera、marker。')
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    for package in ('realsense2_camera', 'rqt_image_view', *(('aruco_ros',) if kind == 'aruco' else ())):
        try:
            get_package_prefix(package)
        except PackageNotFoundError:
            raise SystemExit(f'preview 缺少 ROS 包 {package}；请先运行本工具的 check 命令检查依赖。') from None
    camera = '/' + '/'.join(part.strip('/') for part in (cfg['camera_namespace'], cfg['camera_name'])
                            if part.strip('/'))
    _, pose_topic, result_topic = target_settings(cfg)
    detector = '/handeye_chessboard' if kind == 'chessboard' else '/aruco_single'
    context = rclpy.context.Context()
    rclpy.init(args=[], context=context)
    node, executor = None, None
    try:
        node = rclpy.create_node(f'handeye_preview_check_{os.getpid()}', context=context)
        # The global spin helper uses the default Context, not this private one.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        nodes = {'/' + '/'.join(part.strip('/') for part in (namespace, name) if part.strip('/'))
                 for name, namespace in node.get_node_names_and_namespaces()}
        conflicts = sorted(nodes.intersection((camera, detector)))
        for topic in (camera + '/color/image_raw', camera + '/color/camera_info', pose_topic, result_topic):
            if node.get_publishers_info_by_topic(topic):
                conflicts.append(topic)
        if conflicts:
            raise SystemExit(f'当前 ROS Domain {context.get_domain_id()} 已有相机或检测发布源：'
                             f'{"、".join(conflicts)}。请先退出原 camera/marker/preview 会话，再启动 preview。')
    finally:
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown(context=context)


def run_preview(cfg, with_depth=False, tracking=False):
    """Own exactly three process groups, their logs, and their bounded teardown."""
    kind, _, result_topic = target_settings(cfg)
    validate_target(cfg, kind)
    PREVIEW_ROOT.mkdir(parents=True, exist_ok=True)
    # Keep this inode after unlocking; deleting it would allow parallel locks.
    with (PREVIEW_ROOT / '.preview.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有 preview 会话运行，请先关闭原预览窗口或在原终端按 Ctrl+C。', flush=True)
            return 1
        preview_preflight(cfg, kind)
        started = datetime.now(ZoneInfo('Asia/Shanghai'))
        logs = PREVIEW_ROOT / started.strftime('%Y%m%d_%H%M%S_%f')
        logs.mkdir()
        commands = preview_commands(with_depth, tracking)
        if tracking:
            commands['camera'] += ['--camera-options', str(tracking_clock_options(logs))]
        metadata = {'started_at': started.isoformat(), 'pid': os.getpid(),
                    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                    'config': cfg, 'preview_topic': result_topic, 'processes': {}}
        (logs / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        processes, readers, files = {}, [], []
        reason, exit_code = None, 0

        def request_stop(signum, frame):
            nonlocal reason
            if reason is None:
                reason = signal.Signals(signum).name

        def relay(name, pipe, logfile):
            for line in pipe:
                logfile.write(line)
                logfile.flush()
                try:
                    print(f'[{name}] {line.rstrip()}', flush=True)
                except BrokenPipeError:
                    pass  # Continue persisting logs if a terminal pipe closes.

        handlers = {sig: signal.signal(sig, request_stop)
                    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        print(f'预览日志: {logs}\n结果话题: {result_topic}\n关闭预览窗口或按 Ctrl+C 结束本次三个程序。', flush=True)
        cleanup = {'reason': None, 'processes': {}, 'errors': []}
        try:
            for name, command in commands.items():
                if reason is not None:
                    break
                logfile = (logs / f'{name}.log').open('w', encoding='utf-8')
                files.append(logfile)
                process = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    encoding='utf-8', errors='replace', start_new_session=True,
                    env={**os.environ, 'PYTHONUNBUFFERED': '1'})
                processes[name] = process
                metadata['processes'][name] = {'pid': process.pid, 'command': command}
                (logs / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
                reader = threading.Thread(target=relay, args=(name, process.stdout, logfile), daemon=True)
                readers.append(reader)
                reader.start()
            while reason is None:
                for name, process in processes.items():
                    code = process.poll()
                    if code is not None:
                        reason = f'{name}_exited:{code}'
                        exit_code = 0 if name == 'view' and code == 0 else 1
                        print(f'{name} 已退出（{code}），正在结束本次预览。', flush=True)
                        break
                if reason is None:
                    time.sleep(0.05)
        except (OSError, RuntimeError) as error:
            reason, exit_code = 'startup_error', 1
            cleanup['errors'].append(str(error))
            print(f'预览启动失败: {error}', flush=True)
        finally:
            try:
                for name, process in reversed(list(processes.items())):
                    try:
                        report = stop_process_group(process)
                    except (OSError, RuntimeError) as error:
                        report = {'remaining_pids': [], 'errors': [str(error)]}
                    cleanup['processes'][name] = report
                    if report['remaining_pids'] or report['errors']:
                        exit_code = 1
                for reader in readers:
                    reader.join(timeout=2.)
                    if reader.is_alive():
                        cleanup['errors'].append('日志读取线程尚未结束，请检查清理报告中的残留进程。')
                        exit_code = 1
                for process in processes.values():
                    if process.stdout is not None:
                        process.stdout.close()
                for logfile in files:
                    logfile.close()
                cleanup['reason'] = reason or 'supervisor_error'
                (logs / 'cleanup.json').write_text(json.dumps(cleanup, ensure_ascii=False, indent=2) + '\n')
            finally:
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
        if exit_code:
            print(f'预览未正常结束，请检查 {logs}/cleanup.json 和对应日志。', flush=True)
        return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check', 'camera', 'marker', 'view', 'preview', 'calibrate', 'board',
                                          'offline-test', 'integration-test'])
    parser.add_argument('--color-only', action='store_true', help='camera 仅开彩色流，供棋盘跟踪使用')
    parser.add_argument('--camera-options', type=Path, help='传给 D405 驱动的 ROS 参数 YAML（camera）')
    parser.add_argument('--with-depth', action='store_true', help='preview 同时开启深度流')
    parser.add_argument('--tracking', action='store_true', help='跟踪模式使用相机硬件时钟和快速棋盘检测')
    args = parser.parse_args()
    if args.color_only and args.command != 'camera':
        parser.error('--color-only 仅适用于 camera')
    if args.camera_options and args.command != 'camera':
        parser.error('--camera-options 仅适用于 camera')
    if args.with_depth and args.command != 'preview':
        parser.error('--with-depth 仅适用于 preview')
    cfg = config()
    kind, pose_topic, preview_topic = target_settings(cfg)
    if args.tracking and (args.command not in ('preview', 'camera', 'marker') or kind != 'chessboard'):
        parser.error('--tracking 适用于棋盘格 preview、camera 或 marker')
    name = cfg['camera_name']
    camera_frame = f'{name}_color_optical_frame'
    topic = f"/{cfg['camera_namespace']}/{name}/color"
    if args.command == 'preview':
        try:
            code = run_preview(cfg, args.with_depth, args.tracking)
        except KeyboardInterrupt:
            raise SystemExit(0) from None
        except (OSError, RuntimeError, ImportError) as error:
            raise SystemExit(f'preview 启动检查失败: {error}') from error
        raise SystemExit(code)
    elif args.command == 'check':
        for module in ('rclpy', 'cv2', 'tf_transformations', 'scipy', 'handeye_calibration_ros.agx_arm_calibration'):
            importlib.import_module(module)
            print('OK import', module)
        for package in ('aruco_ros', 'realsense2_camera', 'realsense2_description', 'handeye_calibration_ros'):
            subprocess.run([sys.executable, '/opt/ros/jazzy/bin/ros2', 'pkg', 'prefix', package], check=True)
        print('依赖检查通过（未连接相机或 CAN，未检查实时数据）。')
        if kind == 'chessboard':
            pattern, size = validate_target(cfg, kind)
            for module in ('cv_bridge', 'message_filters'):
                importlib.import_module(module)
            print(f'棋盘格：{pattern[0]} × {pattern[1]} 内角点，每格 {size * 1000:g} mm。')
        else:
            print('ArUco 实测边长:', cfg['marker_size_m'], 'm；null 时禁止启动检测和采样。')
    elif args.command == 'camera':
        if args.tracking and not args.camera_options:
            directory = PREVIEW_ROOT / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f_camera')
            directory.mkdir(parents=True)
            args.camera_options = tracking_clock_options(directory)
        command = ['launch', 'realsense2_camera', 'rs_launch.py',
                   f'camera_namespace:={cfg["camera_namespace"]}', f'camera_name:={name}',
                   'enable_color:=true', 'enable_depth:=false' if args.color_only else 'enable_depth:=true',
                   'enable_infra1:=false',
                   'enable_infra2:=false', 'enable_gyro:=false', 'enable_accel:=false',
                   f'depth_module.color_profile:={cfg["color_profile"]}',
                   f'depth_module.depth_profile:={cfg["color_profile"]}',
                   'depth_module.color_format:=RGB8', 'depth_module.enable_auto_exposure:=true']
        if cfg['camera_serial']:
            command.append(f'serial_no:=_{cfg["camera_serial"]}')
        if args.camera_options:
            command.append(f'config_file:={args.camera_options.resolve(strict=True)}')
        ros_exec(command)
    elif args.command == 'marker':
        if kind == 'chessboard':
            from chessboard import run
            pattern, size = validate_target(cfg, kind)
            return run(topic, camera_frame, pose_topic, preview_topic, pattern, size, args.tracking, args.tracking)
        size = marker_size(cfg)
        ros_exec(['run', 'aruco_ros', 'single', '--ros-args', '-r', '__node:=aruco_single',
                  '-p', f'marker_id:={cfg["marker_id"]}', '-p', f'marker_size:={size}',
                  '-p', 'image_is_rectified:=false', '-p', f'camera_frame:={camera_frame}',
                  '-p', f'reference_frame:={camera_frame}', '-p', 'marker_frame:=handeye_marker',
                  '-r', f'/image:={topic}/image_raw', '-r', f'/camera_info:={topic}/camera_info'])
    elif args.command == 'view':
        ros_exec(['run', 'rqt_image_view', 'rqt_image_view', preview_topic])
    elif args.command == 'calibrate':
        validate_target(cfg, kind)
        cfg = {**cfg, 'target': kind, 'marker_pose_topic': pose_topic}
        if not sys.stdin.isatty():
            raise SystemExit('请在交互终端中运行采样。')
        output = ROOT / 'outputs/handeye' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output.mkdir(parents=True)
        metadata = {**cfg, 'mode': 'eye_in_hand', 'camera_frame': camera_frame,
                    'transform': f"{cfg['end_frame']}_T_{camera_frame}",
                    'convention': 'p_end = R_end_camera @ p_camera + t_end_camera; meters; quaternion xyzw',
                    'capture_schema_version': 2,
                    'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
                    'tool_sources_sha256': {file: hashlib.sha256((TOOL / file).read_bytes()).hexdigest()
                                           for file in ('cli.py', 'collector.py', 'chessboard.py', 'quality.py', 'feedback.py',
                                                        'config/config.yaml')},
                    'dependencies': json.loads((TOOL / 'config/dependencies.json').read_text())}
        (output / 'session.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
        print('结果目录:', output, flush=True)
        # Reuse the upstream solver; the project sampler persists inputs and checks stillness/source time.
        from collector import run
        run(cfg, output, camera_frame)
    elif args.command == 'board':
        from board import generate
        generate(TOOL / 'assets', cfg['marker_id'])
    elif args.command == 'offline-test':
        from offline_test import run
        run()
    elif args.command == 'integration-test':
        from integration_test import run
        run()


if __name__ == '__main__':
    main()
