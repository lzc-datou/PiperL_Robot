#!/usr/bin/env python3
"""Modes: offline demo, read-only evdev, isolated mock stack, guarded real stack."""
import argparse
from contextlib import ExitStack
import fcntl
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from scripts.tools.gamepad_control.settings import load_tool_config, load_input_profile, TOOL_CONFIG_PATH
from scripts.tools.gamepad_control.input_adapter import EvdevInput


def _exec_window(app, window):
    """Deliver terminal exit requests through Qt, including the guarded close path."""
    from PyQt5 import QtCore
    close_requested = False

    def request_close(signum, frame):
        nonlocal close_requested
        # Raising KeyboardInterrupt inside a PyQt slot can stop its event loop.
        # Let Qt process the normal close event on its next timer tick instead.
        close_requested = True

    def poll_close():
        nonlocal close_requested
        if close_requested:
            close_requested = False
            window.close()

    previous = {sig: signal.signal(sig, request_close)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    timer = QtCore.QTimer()
    timer.timeout.connect(poll_close)
    timer.start(50)
    window.show()
    try:
        return app.exec_()
    finally:
        timer.stop()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _acquire(mode,can_port):
    names=[f'piper_servo_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID","0")}.lock']
    if mode=='real': names.append(f'piper_can_{os.getuid()}_{can_port}.lock')
    handles=[]
    try:
        for name in names:
            handle=(Path('/tmp')/name).open('a')
            handles.append(handle)
            try: fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError: raise RuntimeError('已有伺服任务或该 CAN 的控制任务运行')
    except BaseException:
        for handle in handles:
            handle.close()
        raise
    return handles


def _preflight_ros(mode,can_port):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    ready=False
    try:
        node=rclpy.create_node('gamepad_launcher_preflight')
        deadline=time.monotonic()+1.2
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
        if any(ns=='/piper_servo' for _,ns in node.get_node_names_and_namespaces()):
            raise RuntimeError('当前 ROS domain 已有 /piper_servo 控制栈')
        if mode=='real':
            for path in Path('/proc').glob('[0-9]*/cmdline'):
                try:
                    parts=path.read_bytes().split(b'\0')
                    if any(Path(p.decode(errors='replace')).name in ('agx_arm_ctrl_single','agx_arm_ctrl_single_node.py') for p in parts if p):
                        raise RuntimeError('检测到已有官方实机驱动；拒绝占用共享 CAN')
                except (OSError,PermissionError): pass
        ready=True
    finally:
        try:
            if node is not None: node.destroy_node()
        finally:
            if not ready and rclpy.ok(): rclpy.shutdown()
    return rclpy


def _run_stack(args,parser):
    from PyQt5 import QtWidgets,QtCore
    from scripts.tools.gamepad_control.app import GamepadWindow
    from scripts.tools.gamepad_control.stack_supervisor import start_stack, stop_stack
    config_path=TOOL_CONFIG_PATH
    gp=load_tool_config(config_path)
    if args.mode=='real' and (args.input_source!='evdev' or not args.verified_link or not args.verified_mapping):
        parser.error('实机要求 evdev 输入，并分别完成连接断链验证和轴/按键映射校准；使用 --verified-link --verified-mapping 明确确认')
    if args.mode=='real' and args.input_source=='fixture': parser.error('实机模式禁止输入 fixture')
    if args.input_source=='fixture' and args.mode!='sim': parser.error('fixture 只允许纯仿真')
    if args.input_source=='evdev':
        adapter=EvdevInput(load_input_profile())
        candidates, error=adapter.devices(args.device)
        if not candidates: parser.error(error)
        try: adapter._choose_device(candidates)
        except ValueError as exc: parser.error(str(exc))
    command=[sys.executable,'/opt/ros/jazzy/bin/ros2','launch','piper_servo_tracking','task.launch.py',
        f'mode:={args.mode}',f'gamepad:=true',
        f'rviz:={str(getattr(args, "rviz", args.mode == "sim")).lower()}',
        f'gamepad_input_source:={args.input_source}',
        f'gamepad_link_verified:={str(args.verified_link).lower()}',
        f'gamepad_mapping_verified:={str(args.verified_mapping).lower()}',
        f'gamepad_config:={config_path}']
    # An omitted device uses the launch file's default/automatic selection.
    # ROS 2 rejects an explicitly empty CLI argument such as gamepad_device:=.
    if args.device:
        command.append(f'gamepad_device:={args.device}')
    with ExitStack() as lifetime:
        for handle in _acquire(args.mode,'can0'):
            lifetime.callback(handle.close)
        # Qt and ROS initialization can fail. Finish all UI setup before any
        # independent ROS processes exist, and register every cleanup at once.
        app=QtWidgets.QApplication(sys.argv[:1])
        rclpy_module=_preflight_ros(args.mode,'can0')
        lifetime.callback(lambda: rclpy_module.shutdown() if rclpy_module.ok() else None)
        from scripts.tools.gamepad_control.ros_bridge import init_bridge
        bridge=init_bridge()
        lifetime.callback(bridge.destroy_node)
        window=GamepadWindow(args.mode,bridge=bridge,config=gp)
        logs=ROOT/'outputs/gamepad_control'/f'{time.strftime("%Y%m%d_%H%M%S")}_{args.mode}_d{os.environ.get("ROS_DOMAIN_ID","0")}_{os.getpid()}'
        logs.mkdir(parents=True,exist_ok=True)
        logfile=lifetime.enter_context((logs/'stack.log').open('w',encoding='utf-8'))
        process, lifeline=start_stack(command, logfile, logs)
        lifetime.callback(stop_stack, process, lifeline, logs)
        monitor=QtCore.QTimer(window)
        def check_stack():
            if process.poll() is not None and not window.close_requested_at:
                window.status.setText(f'后端启动/运行失败（退出码 {process.returncode}）；请结束控制并检查日志')
                window.hint.setText(f'查看日志：{logs}/stack.log')
        monitor.timeout.connect(check_stack); monitor.start(250)
        return _exec_window(app, window)


def main():
    parser=argparse.ArgumentParser(description='PiPER-L 手柄控制：单轴与基座坐标系笛卡尔速度点动')
    parser.add_argument('--mode',choices=('demo','input','sim','real'),default='demo')
    parser.add_argument('--device',help='输入设备 /dev/input/eventN 或 by-id 路径；用于 input 和使用 evdev 的 sim/real')
    parser.add_argument('--list-devices', action='store_true', help='列出可读的兼容手柄设备并退出，不打开窗口或启动 ROS')
    parser.add_argument('--input-source',choices=('fixture','evdev'),default=None,help='sim 默认使用标记为 fixture 的虚拟中立输入')
    parser.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=None,
                        help='显示三维机械臂；sim 默认开启，--no-rviz 可关闭，real 默认关闭')
    parser.add_argument('--verified-link',action='store_true',help='仅在已实测当前连接模式断链检测后用于 real')
    parser.add_argument('--verified-mapping',action='store_true',help='仅在已校准当前手柄轴/按键方向后用于 real')
    args=parser.parse_args()
    if args.list_devices:
        devices, error = EvdevInput(load_input_profile()).devices(args.device)
        for device in devices:
            print(f"{device['path']}  {device['name']}  USB/输入ID {device['vendor']:04x}:{device['product']:04x}")
        if error:
            print(error, file=sys.stderr)
        return 0 if devices else 1
    if args.mode in ('sim','real') and args.input_source is None:
        args.input_source='evdev' if args.mode=='real' else 'fixture'
    if args.rviz is None:
        args.rviz = args.mode == 'sim'
    if args.mode in ('demo','input'):
        from PyQt5 import QtWidgets
        from scripts.tools.gamepad_control.app import GamepadWindow
        adapter=None
        app=QtWidgets.QApplication(sys.argv[:1])
        try:
            if args.mode=='input':
                adapter=EvdevInput(load_input_profile())
                adapter.start(args.device)
            window=GamepadWindow(args.mode,input_adapter=adapter)
            return _exec_window(app, window)
        finally:
            if adapter is not None:
                adapter.stop()
    try:
        return _run_stack(args,parser)
    except (RuntimeError,ImportError,FileNotFoundError) as exc:
        parser.error(str(exc))


if __name__=='__main__': raise SystemExit(main())
