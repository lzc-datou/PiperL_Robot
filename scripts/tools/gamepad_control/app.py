"""Qt gamepad workspace with fixed actions and a shared desktop theme."""
import time
from math import degrees, isfinite
from PyQt5 import QtCore, QtGui, QtWidgets
from scripts.tools.ui.theme import StatusIndicator, action_icon, apply_theme
from .gamepad_widget import DiagnosticTable, GamepadWidget, MappingCard, ModeButton, SessionStatus
from .core import stop_confirmed_for_generation
from .settings import load_tool_config

HOME_STATES=('HOME_ARMING','HOME_PLANNING','HOMING')


class NumericReading(QtWidgets.QLabel):
    """Fit signed telemetry to the tile without jumping as values change."""
    def __init__(self, name, widest):
        super().__init__('—', objectName='reading')
        self.setAccessibleName(name + ' 反馈')
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        font = QtGui.QFont('DejaVu Sans Mono')
        font.setPixelSize(42)
        self.reference_width = QtGui.QFontMetricsF(font).horizontalAdvance(widest)
        self.reading_size = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        width = self.contentsRect().width()
        size = max(12, min(42, int(42 * width / self.reference_width)))
        if size != self.reading_size:
            self.reading_size = size
            self.setStyleSheet(f'font-size: {size}px;')


class GamepadWindow(QtWidgets.QWidget):
    def __init__(self, mode='demo', input_adapter=None, bridge=None, config=None, device_selector=None):
        super().__init__()
        self.mode, self.input_adapter, self.bridge = mode, input_adapter, bridge
        self.search_text = {'bluetooth': '正在查找蓝牙手柄', 'usb': '正在查找 USB 手柄'}.get(device_selector, '正在查找手柄')
        self.config=load_tool_config() if config is None else config
        self.state='WAITING'; self.last_frame=None; self.close_requested_at=None; self.allow_close=False
        self.close_stop_requested=False
        self.close_stop_generation_required=None
        self.real_fallback_sent=False
        self.reply_error=None
        self.stack_failure = ''
        self.next_diagnostic_update = 0.0
        self.control_mode='single_axis'; self.selected_joint=1
        self.selection_request_pending=False
        if mode == 'demo' and bridge is None:
            self.demo_speeds = dict.fromkeys(('single_axis', 'cartesian'), self.config['default_speed_scale'])
        label={'demo':'离线演示 · 不连接机械臂','input':'输入检查 · 不连接机械臂',
               'sim':'仿真 · 不连接实机','real':'实机 · 手动使能'}[mode]
        self.setWindowTitle('PiPER-L · 手柄控制 · '+label)
        screen=QtWidgets.QApplication.primaryScreen()
        available=screen.availableGeometry().size() if screen else QtCore.QSize(1366,768)
        self.initial_size=QtCore.QSize(min(1280,available.width()-32),
                                      min(700 if mode=='input' else 820,available.height()-32))
        self.setMinimumSize(min(600, self.initial_size.width()), min(460, self.initial_size.height()))
        apply_theme(self)
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(18, 10, 18, 14)
        root.setSpacing(8)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel('PiPER-L    /    手柄控制', objectName='title'))
        header.addStretch()
        self.details_button = QtWidgets.QToolButton()
        self.details_button.setText('输入诊断')
        self.details_button.setCheckable(True)
        self.details_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.details_button.setArrowType(QtCore.Qt.RightArrow)
        self.details_button.toggled.connect(self._toggle_details)
        header.addWidget(self.details_button)
        self.mode_label = QtWidgets.QLabel({'demo':'○  离线预览', 'input':'输入检查',
                                           'sim':'仿真', 'real':'实机'}[mode], objectName='mode')
        self.mode_label.setProperty('real', mode == 'real')
        header.addWidget(self.mode_label)
        root.addLayout(header)
        rule = QtWidgets.QFrame(objectName='rule')
        rule.setFixedHeight(1)
        root.addWidget(rule)

        bar = QtWidgets.QHBoxLayout()
        self.selection_panel = QtWidgets.QWidget()
        tabs = QtWidgets.QHBoxLayout(self.selection_panel)
        tabs.setContentsMargins(0, 0, 0, 0)
        tabs.setSpacing(4)
        self.mode_buttons = {}
        for key, name in (('single_axis', '单轴控制'), ('cartesian', '笛卡尔控制')):
            button = ModeButton(name)
            button.clicked.connect(lambda checked, k=key: self._select_control(k, self.selected_joint))
            tabs.addWidget(button)
            self.mode_buttons[key] = button
        bar.addWidget(self.selection_panel)
        bar.addStretch(1)
        self.device = StatusIndicator('手柄未连接')
        self.device.setWordWrap(True)
        self.feedback_state = StatusIndicator('等待反馈')
        bar.addWidget(self.device)
        bar.addSpacing(12)
        bar.addWidget(self.feedback_state)
        root.addLayout(bar)

        self.body_scroll = QtWidgets.QScrollArea()
        self.body_scroll.setWidgetResizable(True)
        self.body_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content = QtWidgets.QWidget()
        self.body_scroll.setWidget(content)
        body = QtWidgets.QVBoxLayout(content)
        body.setSizeConstraint(QtWidgets.QLayout.SetMinAndMaxSize)
        body.setContentsMargins(1, 1, 1, 1)
        body.setSpacing(10)
        root.addWidget(self.body_scroll, 1)
        self.workspace = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight)
        self.workspace.setSpacing(12)
        self.work_card = self._build_feedback()
        self.mapping = MappingCard()
        self.workspace.addWidget(self.work_card, 3)
        self.workspace.addWidget(self.mapping, 2)
        body.addLayout(self.workspace, 1)

        self.details_panel = QtWidgets.QFrame(objectName='surface')
        self.details_layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight, self.details_panel)
        self.details_layout.setContentsMargins(14, 10, 14, 10)
        self.pad = GamepadWidget()
        self.pad.setFixedHeight(220)
        self.diag = DiagnosticTable(input_only=mode == 'input')
        self.details_layout.addWidget(self.pad, 1)
        self.details_layout.addWidget(self.diag, 2)
        self.details_panel.hide()
        body.addWidget(self.details_panel)

        self.status_area = SessionStatus()
        self.status = self.status_area.title
        self.hint = self.status_area.hint
        self.controls = self._build_session_strip()
        root.addWidget(self.controls)
        self.stop = QtWidgets.QPushButton('停止 · B', objectName='stop')
        self.stop.setMinimumHeight(60)
        self.stop.setIcon(action_icon('stop'))
        self.stop.setIconSize(QtCore.QSize(25, 25))
        self.stop.clicked.connect(self._stop)
        root.addWidget(self.stop)
        if mode == 'input':
            self.work_card.hide()
            self.mapping.hide()
            self.selection_panel.hide()
            self.feedback_state.hide()
            self.controls.hide()
            self.stop.hide()
            root.addWidget(self.status_area)
            self._set_status('等待连接')
            self.details_button.setChecked(True)
            body.setStretch(0, 0)
            body.setStretch(body.indexOf(self.details_panel), 1)
            self.details_layout.setAlignment(self.pad, QtCore.Qt.AlignTop)
            close = QtWidgets.QPushButton('关闭')
            close.clicked.connect(self.close)
            root.addWidget(close)
        elif mode == 'demo' and not bridge:
            self.device.setText('示例手柄')
            self.feedback_state.setText('示例数据')
            for label, value in zip(self.joint_values, (0, 80.214, -68.755, 17.189, 57.296, 0)):
                label.setText(f'{value:.3f}')
            for label, value in zip(self.pose_values, (248.6, -32.4, 306.8, 2.1, -14.3, 35.0)):
                label.setText(f'{value:.1f}')
        self._refresh_controls()
        self._render_selection({})
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFocus()
        self.resize(self.initial_size)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)

    def _build_feedback(self):
        card = QtWidgets.QFrame(objectName='surface')
        layout = QtWidgets.QVBoxLayout(card)
        layout.setContentsMargins(16, 12, 16, 14)
        heading = QtWidgets.QHBoxLayout()
        self.feedback_title = QtWidgets.QLabel(objectName='sectionTitle')
        self.frame_label = QtWidgets.QLabel(objectName='caption')
        heading.addWidget(self.feedback_title)
        heading.addStretch()
        heading.addWidget(self.frame_label)
        layout.addLayout(heading)
        self.motion_panel = QtWidgets.QStackedWidget()
        layout.addWidget(self.motion_panel, 1)
        self.joint_buttons = []
        self.joint_values = []
        self.pose_values = []
        for single in (True, False):
            page = QtWidgets.QWidget()
            self.motion_panel.addWidget(page)
            grid = QtWidgets.QGridLayout(page)
            grid.setContentsMargins(0, 0, 0, 0)
            grid.setSpacing(8)
            if single:
                self.joint_panel = page
                names = [f'J{i}' for i in range(1, 7)]
            else:
                names = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
                grid.addWidget(QtWidgets.QLabel('位置（mm）', objectName='caption'), 0, 0, 1, 3)
                grid.addWidget(QtWidgets.QLabel('姿态（°）', objectName='caption'), 2, 0, 1, 3)
            for index, name in enumerate(names):
                if single:
                    tile = QtWidgets.QPushButton(objectName='jointTile')
                    tile.setCheckable(True)
                    tile.setAccessibleName('选择关节 ' + name)
                    tile.clicked.connect(lambda checked, n=index+1: self._select_control('single_axis', n))
                    self.joint_buttons.append(tile)
                else:
                    tile = QtWidgets.QFrame(objectName='readout')
                tile.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
                tile.setMinimumHeight(82)
                content = QtWidgets.QVBoxLayout(tile)
                content.setContentsMargins(10, 6, 10, 8)
                content.setSpacing(2)
                top = QtWidgets.QHBoxLayout()
                title = QtWidgets.QLabel(name, objectName='axisName')
                top.addWidget(title)
                top.addStretch()
                content.addLayout(top)
                value = NumericReading(name, '-180.000' if single else '-1000.0')
                content.addWidget(value, 1)
                if single:
                    mark = QtWidgets.QLabel('✓', objectName='tileMark')
                    top.addWidget(mark)
                    mark.hide()
                    for child in (title, value, mark):
                        child.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
                    tile.toggled.connect(lambda checked, labels=(title, value, mark):
                                         self._mark_selected(checked, labels))
                    self.joint_values.append(value)
                else:
                    self.pose_values.append(value)
                row = index // 3 if single else 1 + 2 * (index // 3)
                grid.addWidget(tile, row, index % 3)
                grid.setRowStretch(row, 1)
                grid.setColumnStretch(index % 3, 1)
        return card

    @staticmethod
    def _mark_selected(checked, labels):
        for label in labels:
            label.setProperty('selected', checked)
            label.style().unpolish(label)
            label.style().polish(label)
        labels[-1].setVisible(checked)

    def _build_session_strip(self):
        strip = QtWidgets.QFrame(objectName='strip')
        layout = QtWidgets.QBoxLayout(QtWidgets.QBoxLayout.LeftToRight, strip)
        self.session_layout = layout
        layout.setContentsMargins(14, 8, 14, 8)
        layout.setSpacing(14)
        speed_row = QtWidgets.QHBoxLayout()
        self.speed_caption = QtWidgets.QLabel('速度比例', objectName='caption')
        self.speed_label = QtWidgets.QLabel(objectName='speedValue')
        speed_row.addWidget(self.speed_caption)
        speed_row.addWidget(self.speed_label)
        self.jog_slider = QtWidgets.QWidget()
        policy = self.jog_slider.sizePolicy()
        policy.setRetainSizeWhenHidden(True)
        self.jog_slider.setSizePolicy(policy)
        slider = QtWidgets.QVBoxLayout(self.jog_slider)
        slider.setContentsMargins(0, 0, 0, 0)
        slider.setSpacing(0)
        self.speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed.setRange(5, 100)
        self.speed.setSingleStep(5)
        self.speed.setValue(round(self.config['default_speed_scale'] * 100))
        self.speed.setAccessibleName('当前控制模式速度比例')
        slider.addWidget(self.speed)
        bounds = QtWidgets.QHBoxLayout()
        bounds.addWidget(QtWidgets.QLabel('5%', objectName='caption'))
        bounds.addStretch()
        bounds.addWidget(QtWidgets.QLabel('100%', objectName='caption'))
        slider.addLayout(bounds)
        speed_row.addWidget(self.jog_slider, 1)
        layout.addLayout(speed_row, 4)
        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(self.status_area, 1)
        self.start = QtWidgets.QPushButton('开始控制', objectName='primary')
        self.end = QtWidgets.QPushButton('结束控制')
        self.recover = QtWidgets.QPushButton('恢复控制', objectName='primary')
        for button, callback in ((self.start, self._start), (self.end, self._end),
                                 (self.recover, self._recover)):
            button.setFixedWidth(124)
            button.setMinimumHeight(44)
            button.clicked.connect(callback)
            actions.addWidget(button)
        layout.addLayout(actions, 5)
        self.speed.valueChanged.connect(self._speed_changed)
        return strip

    def resizeEvent(self, event):
        super().resizeEvent(event)
        width = event.size().width()
        row, column = QtWidgets.QBoxLayout.LeftToRight, QtWidgets.QBoxLayout.TopToBottom
        self.workspace.setDirection(column if width < 900 else row)
        self.details_layout.setDirection(column if width < 760 else row)
        self.session_layout.setDirection(column if width < 760 else row)

    def _toggle_details(self, show):
        self.details_panel.setVisible(show)
        self.details_button.setArrowType(QtCore.Qt.DownArrow if show else QtCore.Qt.RightArrow)
        self.layout().activate()
        if show:
            self.next_diagnostic_update = 0.0
            QtCore.QTimer.singleShot(0,lambda:self.body_scroll.ensureWidgetVisible(self.details_panel))

    def _set_status(self, text, tone='normal', hint=''):
        if self.stack_failure and self.close_requested_at is None:
            self.status_area.set_message('后端已退出', self.stack_failure, 'danger')
        else:
            self.status_area.set_message(text, hint, tone)

    def report_stack_exit(self, returncode, log):
        self.stack_failure = f'退出码 {returncode}。请结束控制并检查日志：{log}'
        self._set_status('后端已退出')

    def _refresh_diagnostics(self, frame):
        now = time.monotonic()
        if not self.details_panel.isVisible() or now < self.next_diagnostic_update:
            return
        self.next_diagnostic_update = now + 0.2
        self.pad.set_control(None if self.mode == 'input' else self.control_mode, self.selected_joint)
        self.pad.set_frame(frame)
        values = {}
        if frame:
            values.update({
                '摇杆': '  '.join(f'{v:+.3f}' for v in frame['axes']),
                '方向键': '  '.join(str(v) for v in frame['dpad']),
                'LB / B / Y': '   '.join(f'{key}={bool(frame.get(field))}'
                                       for key, field in (('LB', 'lb'), ('B', 'b'), ('Y', 'home'))),
            })
        if self.mode == 'input':
            raw = frame or {}
            device = raw.get('device') or {}
            values.update({'设备': device.get('name'), '连接方式': device.get('connection'), '蓝牙报告来源': device.get('report_source'),
                           '设备路径': device.get('path'),
                           'GUID': raw.get('guid'), '连接代数': raw.get('generation'),
                           '连接失联验证': raw.get('link_verified'),
                           '输入错误': getattr(self.input_adapter, 'error', None)})
        elif self.bridge:
            status, guard = self.bridge.status, self.bridge.guard
            age = status.get('input_age_s')
            progress = status.get('home_hold_progress')
            joints = (status.get('feedback') or {}).get('joints_rad')
            values.update({
                '运行环境': status.get('mode'), '设备': status.get('device_name'),
                '连接方式': frame.get('connection_type') if frame else status.get('connection_type'),
                '设备标识': status.get('device'), '连接代数': status.get('generation'),
                '输入来源': frame.get('source') if frame else None,
                '输入延迟（秒）': f'{age:.3f}' if age is not None and isfinite(age) else None,
                '连接失联验证': status.get('link_verified'), '输入错误': status.get('input_error'),
                '控制状态': status.get('state'), '控制方式': status.get('control_mode'),
                '当前关节': status.get('selected_joint'), '选择版本': status.get('selection_revision'),
                '方向受限': status.get('joint_bound_limited', False), 'Guard 状态': guard.get('state'),
                '运动门关闭': guard.get('gate_closed'), '停稳确认': guard.get('stop_confirmed'),
                '回零确认进度': f'{progress:.0%}' if progress is not None and isfinite(progress) else None,
                '关节角（rad）': '  '.join(f'{v:.5f}' if v is not None and isfinite(v) else '—'
                                        for v in joints) if joints else None,
                'Guard 原因': guard.get('reason'), '最近回复': self.bridge.last_reply,
            })
        else:
            values.update({'运行环境': self.mode, '控制状态': self.state,
                           '控制方式': self.control_mode, '当前关节': self.selected_joint})
        self.diag.display(values)

    def _speed_changed(self,value):
        if self.bridge and not self._input_released():
            self._render_selection(self.bridge.status); return
        self._render_speed(value)
        if self.bridge:
            self.bridge.set_speed(value / 100.)
        elif self.mode == 'demo':
            self.demo_speeds[self.control_mode] = value / 100.

    def _render_speed(self,value):
        status=self.bridge.status if self.bridge else {}
        homing = status.get('state') in HOME_STATES
        self.jog_slider.setVisible(not homing)
        if homing:
            self.speed_caption.setText('回零速度')
            self.speed_label.setText('10%')
            self.speed_label.setToolTip('回零使用独立规划比例；两模式速度滑条不参与。')
            return
        limits=status.get('speed_limits',{})
        linear=limits.get('linear_m_s',self.config['linear_speed_max'])*1000*value/100
        angular=degrees(limits.get('angular_rad_s',self.config['angular_speed_max']))*value/100
        joint=degrees(limits.get('joint_rad_s',self.config['joint_speed']))*value/100
        self.speed_caption.setText('速度比例')
        self.speed_label.setText(f'{value}%')
        tip=(f'J{self.selected_joint} 软件指令上限 {joint:.1f}°/s。' if self.control_mode=='single_axis' else
             f'平移上限 {linear:g} mm/s，旋转上限 {angular:.1f}°/s，方向键使用 {status.get("dpad_scale",self.config["dpad_scale"]):.0%} 旋转比例。')
        self.speed_label.setToolTip(tip+' 不是实际运动速度。松开 LB/B/Y 后调节。')

    def _render_selection(self,status):
        control_mode=status.get('control_mode',self.control_mode)
        if control_mode in ('single_axis','cartesian'):
            self.control_mode=control_mode
        joint=status.get('selected_joint',self.selected_joint)
        if isinstance(joint,int) and not isinstance(joint,bool) and 1<=joint<=6:
            self.selected_joint=joint
        single=self.control_mode=='single_axis'
        for mode,button in self.mode_buttons.items():
            button.setChecked(mode==self.control_mode)
        for index,button in enumerate(self.joint_buttons,1):
            button.setChecked(index==self.selected_joint)
        self.frame_label.setText('单位 °' if single else '基座坐标系 · base_link')
        self.feedback_title.setText('关节角度' if single else '末端位姿')
        self.mapping.set_mode(single)
        self.motion_panel.setCurrentIndex(0 if single else 1)
        scale=status.get('speed_scale')
        if self.mode == 'demo' and self.bridge is None:
            scale = self.demo_speeds[self.control_mode]
        if scale is not None and not self.speed.isSliderDown():
            self.speed.blockSignals(True); self.speed.setValue(round(float(scale)*100)); self.speed.blockSignals(False)
        self._render_speed(self.speed.value())

    def _select_control(self,control_mode,joint_index):
        if control_mode==self.control_mode and joint_index==self.selected_joint:
            self._render_selection(self.bridge.status if self.bridge else {})
            return
        if self.bridge:
            self.reply_error=None; self.selection_request_pending=True
            future=self.bridge.select_control(control_mode,joint_index,self._selection_reply)
            if future is None: self.selection_request_pending=False
        else:
            self.control_mode=control_mode; self.selected_joint=joint_index
        # Checked buttons always follow the latest backend snapshot.
        self._render_selection(self.bridge.status if self.bridge else {})
        self._refresh_controls()

    def _selection_reply(self,ok,message):
        self.selection_request_pending=False
        self._reply(ok,message)

    def _input_connected(self):
        if not self.bridge: return True
        frame=self.bridge.input
        return bool(frame and frame.get('connected',True) and
                    time.monotonic()-frame.get('received_at',0.)<=self.config['input_timeout_s'])

    def _input_released(self,neutral=False):
        if not self.bridge: return True
        frame=self.bridge.input
        if not self._input_connected() or frame.get('lb') or frame.get('b') or frame.get('home',False):
            return False
        return not neutral or (all(abs(value)<=self.config['deadzone'] for value in frame['axes']) and
                               not any(frame['dpad']))

    def _request(self,name):
        self.reply_error=None
        if self.bridge:
            return self.bridge.request(name,lambda ok,msg:self._reply(ok,msg))
        return None
    def _reply(self,ok,message):
        if not ok and message:
            self.reply_error=(self.bridge.status.get('state','WAITING'), message)
            self._set_status(self.status.text(), self.status.property('tone'), message)

    def _start(self):
        if self.bridge: self._request('start')
        else: self.state='ARMED'; self._set_status('按住 LB 操作')
        self._refresh_controls()
    def _end(self):
        if self.bridge: self._request('end')
        else: self.state='WAITING'; self._set_status('会话已结束')
        self._refresh_controls()
    def _recover(self):
        if self.bridge: self._request('recover')
        else: self.state='ARMED'; self._set_status('按住 LB 操作')
        self._refresh_controls()
    def _stop(self):
        if self.bridge: self._request('stop')
        else: self.state='LOCKED'; self._set_status('已锁定', 'danger', '可点击“恢复控制”。')
        self._refresh_controls()
    def _refresh_controls(self):
        if self.mode == 'input':
            for button in (self.start, self.end, self.recover, self.stop):
                button.setEnabled(False)
            self.speed.setEnabled(False)
            return
        status=self.bridge.status if self.bridge else {}
        state=status.get('state',self.state)
        self.start.setVisible(state=='WAITING')
        self.recover.setVisible(state in ('LOCKED','STOP_UNKNOWN'))
        self.end.setVisible(state not in ('WAITING','LOCKED','STOP_UNKNOWN'))
        self.start.setEnabled(state=='WAITING' and
                              (not self.bridge or (status.get('servo_switched') is True and self._input_connected())))
        self.end.setText('结束控制')
        self.end.setEnabled(state in ('ARMED','ACTIVE','PREPARING','STOPPING','STOP_UNKNOWN')+HOME_STATES)
        stopped = (self.mode == 'demo' and self.bridge is None) or bool(status.get('stop_confirmed'))
        self.recover.setEnabled(state in ('LOCKED','STOP_UNKNOWN') and stopped and self._input_released(neutral=True))
        self.speed.setEnabled(state in ('WAITING','ARMED') and self._input_released())
        choose=(state in ('WAITING','ARMED') and self._input_released(neutral=True) and
                (not self.bridge or bool(status.get('selection_ready'))) and not self.selection_request_pending)
        for button in (*self.mode_buttons.values(),*self.joint_buttons):
            button.setEnabled(choose)
            button.setToolTip('停稳后，松开 LB/B/Y 并让摇杆、方向键回中，再选择控制方式或关节。')
        self.stop.setEnabled(state!='WAITING' or bool(self.bridge))
        if self.close_requested_at is not None:
            for widget in (self.start,self.end,self.recover,self.speed): widget.setEnabled(False)
            for button in (*self.mode_buttons.values(),*self.joint_buttons): button.setEnabled(False)

    def _render_feedback(self,status):
        snapshot=status.get('feedback')
        age=max(snapshot.get('age_s',999.), time.time()-snapshot.get('source_stamp',0.)) if snapshot else float('inf')
        if not snapshot or age>.25:
            for value in (*self.joint_values,*self.pose_values): value.setText('—')
            self.feedback_state.set_online(False)
            self.feedback_state.setToolTip('未收到新鲜反馈')
            self.feedback_state.setText('反馈中断'); return
        xyz=snapshot['xyz_m']; rpy=snapshot['rpy_rad']
        for label,value in zip(self.pose_values,(*[v*1000 for v in xyz],*[degrees(v) for v in rpy])):
            label.setText(f'{value:.1f}' if isfinite(value) else '—')
        joints=snapshot.get('joints_rad',[])
        for index,label in enumerate(self.joint_values):
            value=joints[index] if len(joints)==6 else None
            label.setText(f'{degrees(value):.3f}' if value is not None and isfinite(value) else '—')
        self.feedback_state.set_online(True)
        self.feedback_state.setText('反馈在线')
        self.feedback_state.setToolTip(f'关节反馈距今 {age*1000:.0f} ms；TCP 由当前模型正运动学计算。')

    def _render_control_status(self, status):
        state=status.get('state','WAITING')
        titles={'WAITING':'等待开始','ARMED':'按住 LB 操作','PREPARING':'正在准备',
                'ACTIVE':'控制中','STOPPING':'正在停止','LOCKED':'已锁定','STOP_UNKNOWN':'停稳未知',
                'HOME_ARMING':'正在确认回零','HOME_PLANNING':'正在规划回零','HOMING':'回零中'}
        tone='active' if state in ('ACTIVE','HOMING') else 'danger' if state in ('LOCKED','STOP_UNKNOWN') else 'normal'
        title=titles.get(state,'等待状态'); hint=''
        if state=='WAITING' and not status.get('servo_switched'):
            title='正在准备'
        elif state=='ARMED':
            if not status.get('neutral_seen'):
                title, hint = '等待回中', '松开 LB/B/Y，摇杆和方向键回中。'
            if status.get('reason','').startswith(('回零', '已回零', '保持 Y', '请先松开 LB/Y')):
                hint=status['reason']
            if status.get('reason','').startswith('看门狗暂不可准备'):
                hint=status['reason']; tone='warning'
        elif state=='ACTIVE' and status.get('joint_bound_limited'):
            title,tone='方向受限','warning'; hint='反向操作可退回。'
        elif state=='HOME_ARMING':
            progress=float(status.get('home_hold_progress',0.))
            progress=max(0.,min(1.,progress)) if isfinite(progress) else 0.
            title=f'回零确认 {progress:.0%}'
        elif state in ('LOCKED','STOP_UNKNOWN','STOPPING'):
            hint=status.get('reason','')
        if state in ('LOCKED', 'STOP_UNKNOWN'):
            if not status.get('stop_confirmed'):
                recovery = '等待停稳确认。'
            elif not self._input_released(neutral=True):
                recovery = '恢复前请连接手柄，松键并回中。'
            else:
                recovery = '可点击“恢复控制”。'
            hint = recovery + ('锁定原因：' + hint if hint else '')
        if not hint and state in ('WAITING', 'ARMED'):
            if not self._input_released(neutral=True):
                hint = '切换模式前，确认手柄已连接、松键并回中。'
            elif not status.get('selection_ready'):
                hint = '等待后台确认可切换模式。'
        if state in HOME_STATES:
            hint='保持 Y + LB · 松开取消 · 回零 10%'
        if self.reply_error is not None:
            if self.reply_error[0]==state: hint=self.reply_error[1]
            else: self.reply_error=None
        if self.selection_request_pending:
            hint='正在确认控制选择。'
        self._set_status(title, tone, hint)

    def refresh(self):
        diagnostic_frame = None
        if self.bridge:
            self.bridge.spin(); self.bridge.heartbeat()
            status=self.bridge.status; state=status.get('state','WAITING')
            self.state=state; self._render_selection(status)
            guard_lost = (self.mode=='real' and self.bridge.guard_received and
                    time.monotonic()-self.bridge.guard_received>.35 and
                    (state in ('ACTIVE','PREPARING','STOPPING')+HOME_STATES or self.bridge.guard.get('state')=='running'))
            if guard_lost:
                if not self.real_fallback_sent: self.real_fallback_sent=self.bridge.real_stop_fallback()
            self._render_feedback(status)
            frame=self.bridge.input
            if self._input_connected():
                diagnostic_frame = frame
                self.device.set_online(frame['source'] != 'fixture')
                name=frame.get('device_name') or status.get('device_name') or '手柄'
                prefix = '蓝牙已连接 · ' if frame.get('connection_type') == '蓝牙' else '已连接 · '
                self.device.setText('模拟输入 · 非实体手柄' if frame['source']=='fixture' else prefix + name)
                self.device.setToolTip(str(status.get('device') or name))
            else:
                self.device.set_online(False)
                self.device.setText(self.search_text if frame is None and self.mode == 'real' else '手柄未连接或输入中断')
                self.device.setToolTip(status.get('input_error') or '未收到新鲜手柄输入')
            guard=self.bridge.guard
            if self.close_requested_at is not None:
                self._set_status('正在退出', hint='正在停止机械臂，等待停稳确认。')
                if not self.close_stop_requested:
                    future=self._request('stop')
                    self.close_stop_requested=future is not None
                stop_confirmed=stop_confirmed_for_generation(
                    guard, self.close_stop_generation_required)
                if stop_confirmed:
                    self.allow_close=True; self.close()
                elif time.monotonic()-self.close_requested_at > 8.:
                    self._set_status('停稳未知','danger')
                    self.allow_close=True; self.close()
            elif guard_lost:
                self._set_status('控制连接中断', 'danger', '已请求停止，等待真实反馈确认停稳。'
                                 if self.real_fallback_sent else '正在请求停止，请检查机械臂状态。')
            else:
                self._render_control_status(status)
        elif self.mode=='input' and self.input_adapter:
            frame=self.input_adapter.latest()
            self.device.set_online(bool(frame))
            if frame:
                self.last_frame = diagnostic_frame = frame
                prefix = '蓝牙已连接 · ' if frame['device'].get('connection') == '蓝牙' else '已连接 · '
                self.device.setText(prefix + frame['device']['name'])
                self.device.setToolTip(frame['device']['path'])
                if self.input_adapter.error:
                    self._set_status('手柄输入提示', 'warning', self.input_adapter.error)
                else:
                    self._set_status('手柄已连接', 'active')
            else:
                self.device.setText('手柄未连接' if self.last_frame else self.search_text)
                self._set_status('等待连接', hint=self.input_adapter.error or '连接并开启手柄，程序会自动识别。')
        self._refresh_diagnostics(diagnostic_frame)
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
                self._set_status('正在退出', hint='正在停止机械臂，等待停稳确认。')
                self.close_stop_requested=self._request('stop') is not None
            return
        self.timer.stop()
        if self.input_adapter: self.input_adapter.stop()
        event.accept()
