"""Single-page PyQt5 gamepad window with nonblocking ROS status/control bridge."""
import time
from math import degrees
from PyQt5 import QtCore, QtWidgets
from .gamepad_widget import GamepadWidget
from .core import stop_confirmed_for_generation


class GamepadWindow(QtWidgets.QWidget):
    def __init__(self, mode='demo', input_adapter=None, bridge=None):
        super().__init__()
        self.mode, self.input_adapter, self.bridge = mode, input_adapter, bridge
        self.state='WAITING'; self.last_frame=None; self.close_requested_at=None; self.allow_close=False
        self.close_stop_requested=False
        self.close_stop_generation_required=None
        self.real_fallback_sent=False
        label={'demo':'离线演示 · 不连接机械臂','input':'只读输入 · 不连接机械臂',
               'sim':'纯仿真 · 官方 mock / MoveIt Servo','real':'实机 · 显式模式 / 手动使能'}[mode]
        self.setWindowTitle('PiPER-L · 手柄控制'+(' · '+label))
        self.resize(900,720); self.setMinimumSize(680,600)
        self.setStyleSheet('QWidget{background:#f8f9fb;color:#243244} QLabel#title{font-size:25px;font-weight:600} QLabel#status{font-size:20px;font-weight:600} QPushButton{font-size:16px;padding:12px;border-radius:8px;background:white;border:1px solid #dce2e9} QPushButton#stop{background:#bd3945;color:white} QSlider{min-height:26px}')
        root=QtWidgets.QVBoxLayout(self); root.setContentsMargins(24,18,24,18); root.setSpacing(10)
        root.addWidget(QtWidgets.QLabel('PiPER-L · 手柄控制',objectName='title'))
        top=QtWidgets.QHBoxLayout(); self.mode_label=QtWidgets.QLabel(label); self.device=QtWidgets.QLabel('等待手柄')
        top.addWidget(self.mode_label); top.addStretch(); top.addWidget(self.device); root.addLayout(top)
        self.status=QtWidgets.QLabel('等待手柄与反馈',objectName='status'); root.addWidget(self.status)
        self.hint=QtWidgets.QLabel('LB 持续许可，松开立即撤销；B 停止并锁定。先松开 LB、摇杆回中，再按住 LB。')
        root.addWidget(self.hint)
        row=QtWidgets.QHBoxLayout(); self.pad=GamepadWidget(); row.addWidget(self.pad,1)
        feedback=QtWidgets.QVBoxLayout(); feedback.addWidget(QtWidgets.QLabel('TCP 位置 · base_link'))
        self.xyz=QtWidgets.QLabel('X —   Y —   Z —'); self.xyz.setStyleSheet('font-size:22px;font-weight:600'); feedback.addWidget(self.xyz)
        self.feedback_state=QtWidgets.QLabel('反馈中断 / 停稳未知'); feedback.addWidget(self.feedback_state)
        self.pose=QtWidgets.QLabel('RPY —'); feedback.addWidget(self.pose)
        self.details_button=QtWidgets.QToolButton(); self.details_button.setText('设备与诊断'); self.details_button.setCheckable(True); self.details_button.setArrowType(QtCore.Qt.RightArrow)
        self.diag=QtWidgets.QTextEdit(); self.diag.setReadOnly(True); self.diag.setMaximumHeight(108); self.diag.hide()
        self.details_button.toggled.connect(lambda show:(self.diag.setVisible(show),self.details_button.setArrowType(QtCore.Qt.DownArrow if show else QtCore.Qt.RightArrow)))
        feedback.addWidget(self.details_button); feedback.addWidget(self.diag); feedback.addStretch(); row.addLayout(feedback,1)
        root.addLayout(row,1)
        self.speed=QtWidgets.QSlider(QtCore.Qt.Horizontal); self.speed.setRange(5,100); self.speed.setSingleStep(5); self.speed.setValue(20)
        self.speed_label=QtWidgets.QLabel('最高速度 20% · 平移 10 mm/s · 旋转 2.9°/s')
        root.addWidget(self.speed_label); root.addWidget(self.speed)
        self.speed.valueChanged.connect(self._speed_changed)
        buttons=QtWidgets.QHBoxLayout()
        self.start=QtWidgets.QPushButton('开始控制'); self.end=QtWidgets.QPushButton('结束控制'); self.recover=QtWidgets.QPushButton('恢复控制')
        self.stop=QtWidgets.QPushButton('停止  B',objectName='stop')
        for button in (self.start,self.end,self.recover,self.stop): buttons.addWidget(button)
        root.addLayout(buttons)
        self.start.clicked.connect(self._start); self.end.clicked.connect(self._end); self.recover.clicked.connect(self._recover); self.stop.clicked.connect(self._stop)
        self.footer=QtWidgets.QLabel('软件停止不代替物理急停'); root.addWidget(self.footer)
        self._refresh_controls()
        self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(50)

    def _speed_changed(self,value):
        self.speed_label.setText(f'最高速度 {value}% · 平移 {50*value/100:.0f} mm/s · 旋转 {14.3*value/100:.1f}°/s')
        if self.bridge: self.bridge.set_speed(value/100.)

    def _request(self,name):
        if self.bridge:
            return self.bridge.request(name,lambda ok,msg:self._reply(ok,msg))
        return None
    def _reply(self,ok,message):
        if message: self.hint.setText(message)

    def _start(self):
        if self.bridge: self._request('start')
        else: self.state='ARMED'; self.status.setText('离线演示：等待按住 LB')
        self._refresh_controls()
    def _end(self):
        if self.bridge: self._request('end')
        else: self.state='WAITING'; self.status.setText('离线演示：会话已结束')
        self._refresh_controls()
    def _recover(self):
        if self.bridge: self._request('recover')
        else: self.state='ARMED'; self.status.setText('离线演示：等待按住 LB')
        self._refresh_controls()
    def _stop(self):
        if self.bridge: self._request('stop')
        else: self.state='LOCKED'; self.status.setText('离线演示：停止锁定')
        self._refresh_controls()
    def _refresh_controls(self):
        status=self.bridge.status if self.bridge else {}
        state=status.get('state',self.state)
        self.start.setEnabled(state=='WAITING' and
                              (not self.bridge or (bool(status) and status.get('servo_switched') is True)))
        self.end.setEnabled(state in ('ARMED','ACTIVE','PREPARING','STOPPING','STOP_UNKNOWN'))
        self.recover.setEnabled(state in ('LOCKED','STOP_UNKNOWN') and bool(status.get('stop_confirmed')))
        self.speed.setEnabled(state not in ('ACTIVE','PREPARING','STOPPING'))
        self.stop.setEnabled(state!='WAITING' or bool(self.bridge))

    def _render_feedback(self,status):
        snapshot=status.get('feedback')
        if not snapshot or snapshot.get('age_s',999.)>.25:
            self.xyz.setText('X —   Y —   Z —'); self.pose.setText('RPY —')
            self.feedback_state.setText('反馈中断 / 停稳未知'); return
        x,y,z=snapshot['xyz_m']; rx,ry,rz=snapshot['rpy_rad']
        self.xyz.setText(f'X {x*1000:.1f}   Y {y*1000:.1f}   Z {z*1000:.1f} mm')
        self.pose.setText(f'固定 XYZ RPY {degrees(rx):.1f}° / {degrees(ry):.1f}° / {degrees(rz):.1f}°')
        self.feedback_state.setText(('仿真反馈' if self.mode=='sim' else '真实关节反馈')+f' · age {snapshot["age_s"]*1000:.0f} ms')

    def refresh(self):
        if self.bridge:
            self.bridge.spin(); self.bridge.heartbeat()
            status=self.bridge.status; state=status.get('state','WAITING')
            if (self.mode=='real' and not self.real_fallback_sent and self.bridge.guard_received and
                    time.monotonic()-self.bridge.guard_received>.35 and
                    (state in ('ACTIVE','PREPARING','STOPPING') or self.bridge.guard.get('state')=='running')):
                self.real_fallback_sent=self.bridge.real_stop_fallback()
                self.status.setText('guard 状态中断：已请求官方驱动关闭控制门/保持；停稳仍待真实反馈确认')
            self.state=state; self.status.setText(status.get('reason','等待后端状态'))
            self._render_feedback(status)
            frame=self.bridge.input
            if frame and time.monotonic()-frame['received_at']<=.12:
                self.pad.set_frame(frame); self.device.setText(f'{status.get("device_name") or frame["source"]} · {status.get("connection_type")} · 代数 {frame["generation"]}')
            else:
                self.pad.set_frame(None); self.device.setText(status.get('input_error') or '输入数据超时')
            guard=self.bridge.guard
            self.diag.setPlainText(
                f"mode={status.get('mode')} / guard={guard.get('state','unknown')} / gate_closed={guard.get('gate_closed')}\n"
                f"设备={status.get('device_name')} / identity={status.get('device')} / generation={status.get('generation')} / input_age={status.get('input_age_s')}\n"
                f"连接失联验证={status.get('link_verified')} / 输入错误={status.get('input_error')}\n"
                f"控制状态={state} / 停稳确认={guard.get('stop_confirmed')} / 样本={guard.get('stop_sample_count')}\n"
                f"guard reason={guard.get('reason')}\n{self.bridge.last_reply}")
            self.footer.setText('软件停止不代替物理急停 · 只有新鲜实测关节样本可确认停稳')
            if self.close_requested_at is not None:
                if not self.close_stop_requested:
                    future=self._request('stop')
                    self.close_stop_requested=future is not None
                stop_confirmed=stop_confirmed_for_generation(
                    guard, self.close_stop_generation_required)
                if stop_confirmed:
                    self.allow_close=True; self.close()
                elif time.monotonic()-self.close_requested_at > 8.:
                    self.status.setText('停稳未知；停止请求已发送，正在保留看门狗完成退出处理')
                    self.allow_close=True; self.close()
        elif self.mode=='input' and self.input_adapter:
            frame=self.input_adapter.latest()
            if frame:
                self.last_frame=frame; self.pad.set_frame(frame)
                self.device.setText(frame['device']['name']+' · '+frame['device']['path'])
                self.diag.setPlainText(f"GUID: {frame['guid']}\n连接代数: {frame['generation']}\n摇杆: {frame['axes']}\n十字键: {frame['dpad']}\nLB={frame['lb']} B={frame['b']}\n失联验证: 未验证")
                self.status.setText('只读输入在线；此模式不启动执行链')
            else:
                self.pad.set_frame(None); self.device.setText(self.input_adapter.error or '输入中断')
                self.status.setText('等待手柄；只读模式')
        self._refresh_controls()

    def closeEvent(self,event):
        if self.bridge and not self.allow_close:
            event.ignore()
            if self.close_requested_at is None:
                self.close_requested_at=time.monotonic()
                status=self.bridge.status
                guard=self.bridge.guard
                guard_generation=int(guard.get('stop_generation',0))
                if status.get('state') in ('STOPPING','STOP_UNKNOWN'):
                    self.close_stop_generation_required=int(
                        status.get('stop_generation_required',guard_generation+1))
                elif status.get('state')=='LOCKED' and guard.get('stop_confirmed'):
                    self.close_stop_generation_required=guard_generation
                else:
                    self.close_stop_generation_required=guard_generation+1
                self.status.setText('正在撤销许可并等待独立看门狗的停稳确认…')
                self.close_stop_requested=self._request('stop') is not None
            return
        self.timer.stop()
        if self.input_adapter: self.input_adapter.stop()
        event.accept()
