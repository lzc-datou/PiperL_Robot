#!/usr/bin/env python3
"""Task entry: explicit mode selection, process ownership, and orderly shutdown."""
import argparse
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from scripts.task.servo_tracking.core import load_config
from scripts.task.common.process_cleanup import stop_process_group


def main(argv=None, *, visual=None):
    parser = argparse.ArgumentParser(description='RViz 六轴伺服；默认纯仿真，不自动使能实机')
    parser.add_argument('--mode', choices=['sim', 'real'], default='sim')
    parser.add_argument('--config', type=Path, default=Path(__file__).parent/'config/config.yaml')
    parser.add_argument('--control-config', type=Path, help='独立笛卡尔控制参数文件')
    parser.add_argument('--no-rviz', action='store_true', help='自动化仿真验证使用；实机仍要求面板心跳')
    args = parser.parse_args(argv)
    args.config = args.config.resolve()
    if args.control_config is not None:
        args.control_config = args.control_config.expanduser().resolve()
    config = load_config(args.config, args.control_config)
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from std_srvs.srv import Trigger, SetBool, Empty
    from std_msgs.msg import String
    import json
    for package in ('moveit_servo', 'piper_servo_tracking'):
        try:
            get_package_share_directory(package)
        except LookupError:
            parser.error(f'缺少 {package}；先安装依赖并运行 bash scripts/piperl/build.sh')
    locks = []
    names = [f'piper_servo_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock']
    if args.mode == 'real':
        names.append(f'piper_can_{os.getuid()}_{config["can_port"]}.lock')
    for name in names:
        lock = (Path('/tmp')/name).open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('已有伺服任务或该 CAN 的控制任务运行')
        locks.append(lock)
    rclpy.init()
    node = rclpy.create_node('servo_task_launcher')
    deadline = time.monotonic()+1.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if any(ns == '/piper_servo' for _, ns in node.get_node_names_and_namespaces()):
        parser.error('已有 /piper_servo 任务；请先退出旧任务')
    if args.mode == 'real':
        # Domains do not isolate a physical CAN device. Refuse existing official drivers even in other domains.
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                parts = path.read_bytes().split(b'\0')
                if any(Path(p.decode(errors='replace')).name in ('agx_arm_ctrl_single', 'agx_arm_ctrl_single_node.py') for p in parts if p):
                    parser.error('检测到已有实机驱动。请先停止原实机控制栈，再启动伺服实机模式')
            except (OSError, PermissionError):
                pass
    stop = node.create_client(Trigger, '/piper_servo/stop')
    driver_gate = node.create_client(SetBool, '/piper_servo/control_enable') if args.mode == 'real' else None
    driver_hold = node.create_client(Empty, '/piper_servo/emergency_stop') if args.mode == 'real' else None
    seen = {}
    guard_active = False
    def guard_status(msg):
        nonlocal guard_active
        seen['guard'] = time.monotonic()
        state = json.loads(msg.data)['state']
        guard_active = state.startswith('joint_') or state in ('pausing', 'syncing', 'resetting', 'opening', 'running', 'paused', 'holding')
    node.create_subscription(String, '/piper_servo/guard/status', guard_status, 1)
    node.create_subscription(String, '/piper_servo/status', lambda msg: seen.update(backend=time.monotonic()), 1)
    exiting = False
    fault_exit = False
    def request_exit(signum, frame):
        nonlocal exiting
        exiting = True
    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)
    # Multiple isolated simulations may start within the same second.
    run_name = f'{time.strftime("%Y%m%d_%H%M%S")}_{args.mode}_d{os.environ.get("ROS_DOMAIN_ID", "0")}_{os.getpid()}'
    logs = (ROOT/'outputs/tasks/visual_servo' if visual else ROOT/'outputs/servo_tracking')/run_name
    logs.mkdir(parents=True, exist_ok=True)
    print(f'模式：{args.mode}；日志：{logs}/stack.log', flush=True)
    command = [sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', 'piper_servo_tracking', 'task.launch.py',
               f'mode:={args.mode}', f'config:={args.config}',
               f'rviz:={str(not args.no_rviz).lower()}']
    if visual:
        command.extend([f'{key}:={value}' for key, value in visual.items()])
        command.append(f'visual_log_dir:={logs}')
    if args.control_config is not None:
        command.append(f'control_config:={args.control_config}')
    with (logs/'stack.log').open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True, cwd=ROOT)
        try:
            while process.poll() is None and not exiting:
                rclpy.spin_once(node, timeout_sec=.1)
                if any(time.monotonic()-value > .8 for value in seen.values()):
                    print('任务节点心跳中断，正在停止', flush=True)
                    fault_exit = True
                    exiting = True
        finally:
            # Independent fallback if the watchdog process itself died; never enable hardware here.
            if driver_gate is not None and driver_gate.service_is_ready():
                future = driver_gate.call_async(SetBool.Request(data=False))
                rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
            if guard_active and driver_hold is not None and driver_hold.service_is_ready():
                future = driver_hold.call_async(Empty.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=.5)
            if stop.service_is_ready():
                future = stop.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
                # Leave watchdog and driver alive briefly for the hold/gate-close requests.
                until = time.monotonic()+.8
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.05)
            report = stop_process_group(process)
            (logs/'cleanup.json').write_text(__import__('json').dumps(report, indent=2))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if fault_exit or (process.returncode and not exiting):
        print(f'任务退出，请检查 {logs}/stack.log', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
