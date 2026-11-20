#!/usr/bin/env python3
"""Render labeled UI samples without ROS, evdev, drivers, or control requests."""
import os
import argparse
from math import radians
from pathlib import Path
import sys
import time
from types import SimpleNamespace

os.environ['QT_QPA_PLATFORM'] = 'offscreen'
ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from PyQt5 import QtWidgets
from scripts.tools.gamepad_control.app import GamepadWindow


class PreviewBridge:
    def __init__(self):
        self.status = {
            'mode': 'sim', 'state': 'WAITING', 'servo_switched': True,
            'reason': '界面预览，所有数据为示例', 'device_name': '示例手柄',
            'generation': 1, 'input_age_s': .012,
            'device': 'preview-only', 'connection_type': '示例',
            'link_verified': False, 'stop_confirmed': True, 'neutral_seen': True,
            'control_mode': 'single_axis', 'selected_joint': 1,
            'selection_revision': 1, 'selection_ready': True,
            'speed_scale': .20, 'speed_scales': {'single_axis': .20, 'cartesian': .20},
            'dpad_scale': .35,
            'speed_limits': {'linear_m_s': .30, 'angular_rad_s': 1.50, 'joint_rad_s': 1.50},
            'joint_limits_rad': {'lower': [-2.6, 0., -2.96, -1.75, -1.22, -2.09],
                                 'upper': [2.6, 3.14, 0., 1.75, 1.22, 2.09]},
            'feedback': {'xyz_m': [.25, 0., .30], 'rpy_rad': [0., 0., 0.],
                         'joints_rad': [0., 1.4, -1.2, .3, 1., 0.], 'age_s': 0.},
        }
        self.input = {
            'axes': (0.,)*4, 'dpad': (0,)*4, 'lb': False, 'b': False,
            'source': 'preview', 'device_name': '示例手柄', 'generation': 1,
            'connected': True, 'home': False,
        }
        self.guard = {'state': 'idle', 'gate_closed': True, 'stop_confirmed': True, 'stop_generation': 0}
        self.guard_received = 0.
        self.last_reply = '仅渲染界面，没有连接设备或运行 ROS。'

    def spin(self):
        self.input['received_at'] = time.monotonic()
        self.status['feedback']['source_stamp'] = time.time()

    def heartbeat(self):
        pass

    def select_control(self, control_mode, joint_index, callback=None):
        if callback: callback(False, '只读界面预览，不处理控制请求。')
        return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/generated_images/gamepad_control')
    args=parser.parse_args()
    app = QtWidgets.QApplication([])
    outputs = args.output_dir
    outputs.mkdir(parents=True, exist_ok=True)

    def capture(window, name):
        window.timer.stop()
        window.mode_label.setText('○  离线预览')
        window.resize(960 if window.mode=='input' else 1280, 680 if window.mode=='input' else 736)
        window.show()
        window.refresh()
        app.processEvents()
        if window.details_button.isChecked():
            window.body_scroll.ensureWidgetVisible(window.details_panel)
            app.processEvents()
        else:
            window.body_scroll.verticalScrollBar().setValue(0)
        path = outputs/name
        if not window.grab().save(str(path)):
            raise RuntimeError(f'无法保存预览：{path}')
        print(path)

    bridge = PreviewBridge()
    window = GamepadWindow('sim', bridge=bridge)
    try:
        capture(window, 'ready.png')
        bridge.status.update(state='ARMED', selected_joint=2, selection_revision=2)
        capture(window, 'single_axis.png')
        bridge.status.update(control_mode='cartesian', selection_revision=2)
        bridge.status['feedback'].update(xyz_m=[.2486, -.0324, .3068],
                                         rpy_rad=[radians(v) for v in (2.1, -14.3, 35.0)])
        capture(window, 'cartesian.png')
        bridge.status.update(neutral_seen=False, selection_ready=False)
        capture(window, 'waiting_neutral.png')
        bridge.status.update(neutral_seen=True, selection_ready=True)
        original_feedback=dict(bridge.status['feedback'])
        bridge.status['feedback'].update(xyz_m=[-.6255,.3189,.7254],rpy_rad=[-3.,1.57,3.1])
        capture(window, 'cartesian_values.png')
        bridge.status['feedback']=original_feedback
        bridge.status.update(control_mode='single_axis', selected_joint=3, selection_revision=3)
        window.details_button.setChecked(True)
        capture(window, 'details.png')
        window.diag.scrollToBottom()
        capture(window, 'details_scrolled.png')
        window.details_button.setChecked(False)
        bridge.status.update(state='ACTIVE', joint_bound_limited=True, selection_ready=False)
        bridge.guard.update(state='running', joint_bound_limited=True)
        bridge.input['lb'] = True
        capture(window, 'joint_limit.png')
        bridge.status.update(state='LOCKED', joint_bound_limited=False,
                             reason='手柄 B 停止锁定；停稳已确认')
        bridge.guard.update(state='fault', joint_bound_limited=False)
        bridge.input['lb'] = False
        capture(window, 'locked.png')
        bridge.status.update(state='HOME_ARMING', home_hold_progress=.56,
                             reason='保持 LB + Y 0.4 秒后回零 · 回零速度 10%')
        bridge.input.update(lb=True, home=True)
        capture(window, 'home_arming.png')
        bridge.status.update(state='HOMING',reason='正在执行回零路径')
        capture(window, 'homing.png')
        bridge.status.update(state='ARMED',control_mode='cartesian',neutral_seen=True,selection_ready=True)
        bridge.input.update(lb=False,home=False)
        window.resize(720, 580)
        window.show(); window.refresh(); app.processEvents()
        window.grab().save(str(outputs/'compact.png'))
        bridge.status.update(state='LOCKED',reason='机械臂反馈中断，当前停稳状态未知。请检查反馈连接，并在停稳后恢复控制。')
        bridge.status['feedback']['age_s']=1.
        capture(window, 'feedback_lost.png')
    finally:
        # Preview objects own no execution stack and cannot request a real stop.
        window.allow_close = True
        window.close()

    adapter = SimpleNamespace(error='', latest=lambda: {
        'axes': (0.,)*4, 'dpad': (0,)*4, 'lb': False, 'b': False,
        'device': {'name': '示例手柄', 'path': 'preview-only'},
        'guid': 'preview-only', 'generation': 1,
    }, stop=lambda: None)
    window = GamepadWindow('input', input_adapter=adapter)
    try:
        capture(window, 'input.png')
    finally:
        window.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
