#!/usr/bin/env python3
"""Modes: offline demo, read-only evdev, isolated mock stack, guarded real stack."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from scripts.tools.gamepad_control.core import validate_tool_config
from scripts.tools.gamepad_control.input_adapter import EvdevInput


def load_profile():
    import yaml
    path=Path(__file__).parent/'config/flydigi_direwolf2.yaml'
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _acquire(mode,can_port):
    names=[f'piper_servo_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID","0")}.lock']
    if mode=='real': names.append(f'piper_can_{os.getuid()}_{can_port}.lock')
    handles=[]
    for name in names:
        handle=(Path('/tmp')/name).open('a')
        try: fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('已有伺服任务或该 CAN 的控制任务运行')
        handles.append(handle)
    return handles


def _preflight_ros(mode,can_port):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('gamepad_launcher_preflight')
    deadline=time.monotonic()+1.2
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
    if any(ns=='/piper_servo' for _,ns in node.get_node_names_and_namespaces()):
        node.destroy_node(); rclpy.shutdown(); raise RuntimeError('当前 ROS domain 已有 /piper_servo 控制栈')
    if mode=='real':
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                parts=path.read_bytes().split(b'\0')
                if any(Path(p.decode(errors='replace')).name in ('agx_arm_ctrl_single','agx_arm_ctrl_single_node.py') for p in parts if p):
                    node.destroy_node(); rclpy.shutdown(); raise RuntimeError('检测到已有官方实机驱动；拒绝占用共享 CAN')
            except (OSError,PermissionError): pass
    node.destroy_node()
    return rclpy


def _run_stack(args,parser):
    import yaml
    from PyQt5 import QtWidgets,QtCore
    import rclpy
    from scripts.tools.gamepad_control.app import GamepadWindow
    from scripts.tools.gamepad_control.ros_bridge import init_bridge
    from scripts.task.common.process_cleanup import stop_process_group
    config_path=Path(__file__).parent/'config/config.yaml'
    gp=validate_tool_config(yaml.safe_load(config_path.read_text(encoding='utf-8')))
    if args.mode=='real' and (args.input_source!='evdev' or not args.verified_link or not args.verified_mapping):
        parser.error('实机要求 evdev 输入，并分别完成连接断链验证和轴/按键映射校准；使用 --verified-link --verified-mapping 明确确认')
    if args.mode=='real' and args.input_source=='fixture': parser.error('实机模式禁止输入 fixture')
    if args.input_source=='fixture' and args.mode!='sim': parser.error('fixture 只允许纯仿真')
    handles=_acquire(args.mode,'can0')
    rclpy_module=_preflight_ros(args.mode,'can0')
    logs=ROOT/'outputs/gamepad_control'/f'{time.strftime("%Y%m%d_%H%M%S")}_{args.mode}_d{os.environ.get("ROS_DOMAIN_ID","0")}_{os.getpid()}'
    logs.mkdir(parents=True,exist_ok=True)
    logfile=(logs/'stack.log').open('w',encoding='utf-8')
    command=[sys.executable,'/opt/ros/jazzy/bin/ros2','launch','piper_servo_tracking','task.launch.py',
        f'mode:={args.mode}',f'gamepad:=true','rviz:=false',f'gamepad_input_source:={args.input_source}',
        f'gamepad_device:={args.device or ""}',f'gamepad_link_verified:={str(args.verified_link).lower()}',
        f'gamepad_mapping_verified:={str(args.verified_mapping).lower()}',
        f'gamepad_config:={config_path}']
    process=subprocess.Popen(command,stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT)
    app=QtWidgets.QApplication(sys.argv[:1])
    bridge=init_bridge()
    window=GamepadWindow(args.mode,bridge=bridge)
    monitor=QtCore.QTimer(window)
    def check_stack():
        if process.poll() is not None and not window.close_requested_at:
            window.status.setText(f'后端启动/运行失败（退出码 {process.returncode}）；未发送运动指令')
            window.hint.setText(f'查看日志：{logs}/stack.log')
    monitor.timeout.connect(check_stack); monitor.start(250)
    window.show()
    exit_code=0
    try: exit_code=app.exec_()
    finally:
        # A close event first sends gamepad/stop and waits asynchronously for
        # guard evidence. The owned launch group remains alive while it drains.
        bridge.destroy_node()
        if rclpy_module.ok(): rclpy_module.shutdown()
        report=stop_process_group(process,timeouts=(6.,3.,2.))
        (logs/'cleanup.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        logfile.close()
    return exit_code


def main():
    parser=argparse.ArgumentParser(description='飞智冰原狼 2 · PiPER-L 固定六自由度速度控制')
    parser.add_argument('--mode',choices=('demo','input','sim','real'),default='demo')
    parser.add_argument('--device',help='输入设备 /dev/input/eventN；input 或 sim --input-source evdev 使用')
    parser.add_argument('--input-source',choices=('fixture','evdev'),default=None,help='sim 默认使用标记为 fixture 的虚拟中立输入')
    parser.add_argument('--verified-link',action='store_true',help='仅在已实测当前连接模式断链检测后用于 real')
    parser.add_argument('--verified-mapping',action='store_true',help='仅在已校准当前手柄轴/按键方向后用于 real')
    args=parser.parse_args()
    if args.mode in ('sim','real') and args.input_source is None:
        args.input_source='evdev' if args.mode=='real' else 'fixture'
    if args.mode in ('demo','input'):
        from PyQt5 import QtWidgets
        from scripts.tools.gamepad_control.app import GamepadWindow
        adapter=None
        if args.mode=='input':
            adapter=EvdevInput(load_profile())
            adapter.start(args.device)
        app=QtWidgets.QApplication(sys.argv[:1]); window=GamepadWindow(args.mode,input_adapter=adapter)
        window.show(); return app.exec_()
    try:
        return _run_stack(args,parser)
    except (RuntimeError,ImportError,FileNotFoundError) as exc:
        parser.error(str(exc))


if __name__=='__main__': raise SystemExit(main())
