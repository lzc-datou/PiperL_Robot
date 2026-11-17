#!/usr/bin/env python3
"""Run the desktop panel; --demo never opens CAN or starts ROS."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description='PiPER-L 底层 CAN 五按钮面板')
    parser.add_argument('--demo', action='store_true', help='离线演示，不打开 CAN')
    parser.add_argument('--screenshot', type=Path, help='仅演示模式：保存界面截图后退出')
    parser.add_argument('--home-speed', type=int, default=10, help='回零速度比例 1–100，默认 10')
    args = parser.parse_args()
    if not 1 <= args.home_speed <= 100:
        parser.error('--home-speed 必须为 1–100')
    if args.screenshot and not args.demo:
        parser.error('--screenshot 必须与 --demo 一起使用')
    from PyQt5 import QtCore, QtGui, QtWidgets
    from scripts.tools.control_panel.backend import Backend
    from scripts.tools.control_panel.app import Panel
    application = QtWidgets.QApplication(sys.argv[:1])
    application.setFont(QtGui.QFont('Noto Sans CJK SC', 10))
    settings = None if args.demo else QtCore.QSettings('PiperL', 'RawCanPanel')
    channel = 'can0' if args.demo else (settings.value('last_port', '', type=str) or None)
    backend = Backend(channel, demo=args.demo, home_speed=args.home_speed)
    window = Panel(backend, settings)
    window.show()
    if args.screenshot:
        def capture():
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(args.screenshot)):
                raise RuntimeError('无法保存截图')
            window.close()
        QtCore.QTimer.singleShot(1800, capture)
    return application.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
