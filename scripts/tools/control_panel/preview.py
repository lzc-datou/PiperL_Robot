#!/usr/bin/env python3
"""Capture owned CAN panel windows using static data, without any CAN backend."""
import argparse
import importlib.util
import os
from pathlib import Path
import queue
import sys

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PyQt5 import QtWidgets


class PreviewBackend:
    demo = True
    home_speed = 10

    def __init__(self, scenario):
        connected = scenario in ('connected', 'busy')
        self.events = queue.Queue()
        self.state = dict(interfaces={'can0': (connected, 1000000, 'ERROR-ACTIVE' if connected else 'STOPPED'),
                                     'can1': (False, 1000000, 'STOPPED')},
                          channel='can0', scanned=True, can_connect=not connected,
                          connected=connected, busy=scenario == 'busy')
        messages = {
            'normal': 'can0 未启用，请点击 CAN 连接',
            'connected': 'CAN 运动模式和六轴使能已确认',
            'busy': '正在恢复 CAN / MOVE J 模式，等待控制器正常与六轴使能反馈。请等待本次操作完成。',
            'error': ('使能确认失败：关节反馈超时。当前没有收到完整的控制器模式与六轴使能反馈，'
                      '请检查适配器、线缆和机械臂供电。确认旧控制任务已经结束，再检查现场状态；'
                      '不要通过反复点击回零或重置来确认连接。'),
        }
        self.events.put(messages[scenario])

    def snapshot(self):
        return dict(self.state)

    def select(self, channel):
        raise RuntimeError('Preview must not select a port')

    def submit(self, action):
        raise RuntimeError('Preview must not submit an operation')

    def emergency_stop(self):
        raise RuntimeError('Preview must not request a stop')

    def close(self):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', choices=('before', 'after'), default='after')
    parser.add_argument('--source', type=Path, help='Optional saved app.py for reproducing the baseline')
    parser.add_argument('--output-dir', type=Path,
                        default=ROOT/'outputs/ui_redesign/desktop_workstation/control_panel')
    args = parser.parse_args()
    if args.source:
        spec = importlib.util.spec_from_file_location('control_panel_preview_source', args.source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        Panel = module.Panel
    else:
        from scripts.tools.control_panel.app import Panel
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    output = args.output_dir/args.label
    output.mkdir(parents=True, exist_ok=True)
    for scenario in ('normal', 'connected', 'busy', 'error'):
        for width, height in ((900, 720), (720, 640)):
            panel = Panel(PreviewBackend(scenario))
            try:
                panel.timer.stop()
                panel.resize(width, height)
                panel.show()
                application.processEvents()
                path = output/f'{scenario}_{width}x{height}.png'
                if not panel.grab().save(str(path)):
                    raise RuntimeError(f'Cannot save preview: {path}')
                print(path)
            finally:
                panel.close()
                application.processEvents()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
