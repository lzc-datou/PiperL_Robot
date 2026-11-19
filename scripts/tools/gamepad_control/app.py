"""Qt gamepad workspace with fixed actions and a shared desktop theme."""
import time
from math import degrees, isfinite
from PyQt5 import QtCore, QtWidgets
from scripts.tools.ui.theme import apply_theme
from .gamepad_widget import GamepadWidget
from .core import stop_confirmed_for_generation
from .settings import load_tool_config

HOME_STATES=('HOME_ARMING','HOME_PLANNING','HOMING')


class GamepadWindow(QtWidgets.QWidget):
    def __init__(self, mode='demo', input_adapter=None, bridge=None, config=None):
        super().__init__()
        self.mode, self.input_adapter, self.bridge = mode, input_adapter, bridge
        self.config=load_tool_config() if config is None else config
        self.state='WAITING'; self.last_frame=None; self.close_requested_at=None; self.allow_close=False
        self.close_stop_requested=False
        self.close_stop_generation_required=None
        self.real_fallback_sent=False
        self.reply_error=None
        self.control_mode='single_axis'; self.selected_joint=1
        self.selection_request_pending=False
        label={'demo':'离线演示 · 不连接机械臂','input':'输入检查 · 不连接机械臂',
               'sim':'仿真 · 不连接实机','real':'实机 · 手动使能'}[mode]
        self.setWindowTitle('PiPER-L · 手柄控制 · '+label)
        screen=QtWidgets.QApplication.primaryScreen()
        available=screen.availableGeometry().size() if screen else QtCore.QSize(1366,768)
        self.initial_size=QtCore.QSize(min(1280,available.width()-32),
                                      min(700 if mode=='input' else 820,available.height()-32))
        self.setMinimumSize(min(940,self.initial_size.width()),min(600,self.initial_size.height()))
        apply_theme(self)
        root=QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(22,18,22,18); root.setSpacing(12)
        header=QtWidgets.QHBoxLayout(); header.setSpacing(14)
        header.addWidget(QtWidgets.QLabel('PiPER-L 手柄控制',objectName='title'))
        self.mode_label=QtWidgets.QLabel({'demo':'离线演示','input':'输入检查','sim':'仿真','real':'实机'}[mode],objectName='mode')
        header.addWidget(self.mode_label); header.addStretch()
        self.selection_panel=QtWidgets.QWidget(); modes=QtWidgets.QHBoxLayout(self.selection_panel)
        modes.setContentsMargins(0,0,0,0); modes.setSpacing(10); self.mode_buttons={}
        for key,text in (('single_axis','单轴控制'),('cartesian','笛卡尔控制')):
            button=QtWidgets.QPushButton(text); button.setCheckable(True)
            button.setMinimumWidth(160); button.setProperty('density','compact')
            button.clicked.connect(lambda checked=False, value=key:self._select_control(value,self.selected_joint))
            modes.addWidget(button); self.mode_buttons[key]=button
        header.addWidget(self.selection_panel)
        self.details_button=QtWidgets.QToolButton(); self.details_button.setText('输入诊断'); self.details_button.setCheckable(True)
        self.details_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon); self.details_button.setArrowType(QtCore.Qt.RightArrow)
        self.details_button.toggled.connect(self._toggle_details); header.addWidget(self.details_button)
        root.addLayout(header)

        self.status_bar=QtWidgets.QFrame(objectName='statusBar')
        status_layout=QtWidgets.QHBoxLayout(self.status_bar); status_layout.setContentsMargins(18,10,18,10); status_layout.setSpacing(22)
        self.status=QtWidgets.QLabel('等待开始',objectName='status'); self.status.setProperty('emphasis','high'); self.status.setWordWrap(True)
        status_layout.addWidget(self.status,1)
        self.device=QtWidgets.QLabel('手柄未连接',objectName='connectionText'); self.device.setWordWrap(True)
        self.feedback_state=QtWidgets.QLabel('等待反馈',objectName='connectionText')
        status_layout.addWidget(self.device,1); status_layout.addWidget(self.feedback_state)
        root.addWidget(self.status_bar)
        self.hint=QtWidgets.QLabel(objectName='notice'); self.hint.setWordWrap(True)
        self.hint.setTextFormat(QtCore.Qt.PlainText); self.hint.hide()

        self.body_scroll=QtWidgets.QScrollArea(); self.body_scroll.setWidgetResizable(True)
        self.body_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content=QtWidgets.QWidget(); self.body_scroll.setWidget(content)
        body=QtWidgets.QVBoxLayout(content); body.setContentsMargins(0,0,4,0); body.setSpacing(12); body.addWidget(self.hint)
        root.addWidget(self.body_scroll,1)
        workspace=QtWidgets.QHBoxLayout(); workspace.setSpacing(16)
        self.work_card=QtWidgets.QFrame(objectName='surface')
        work=QtWidgets.QVBoxLayout(self.work_card); work.setContentsMargins(20,16,20,16); work.setSpacing(12)
        self.feedback_title=QtWidgets.QLabel('关节实时状态',objectName='sectionTitle'); work.addWidget(self.feedback_title)
        self.motion_panel=QtWidgets.QStackedWidget()
        self.joint_feedback=QtWidgets.QWidget(); single=QtWidgets.QVBoxLayout(self.joint_feedback)
        single.setContentsMargins(0,0,0,0); single.setSpacing(0)
        headings=QtWidgets.QFrame(objectName='tableHead'); heading_row=QtWidgets.QHBoxLayout(headings)
        heading_row.setContentsMargins(18,8,18,8)
        heading_row.addWidget(QtWidgets.QLabel('关节')); angle_title=QtWidgets.QLabel('位置（°）'); angle_title.setAlignment(QtCore.Qt.AlignRight)
        heading_row.addWidget(angle_title,1); single.addWidget(headings)
        self.joint_rows=[]; self.joint_values=[]
        for index in range(1,7):
            row=QtWidgets.QFrame(objectName='jointRow'); values=QtWidgets.QHBoxLayout(row)
            values.setContentsMargins(18,3,18,3); values.setSpacing(16)
            name=QtWidgets.QLabel(f'J{index}',objectName='axisName'); values.addWidget(name)
            value=QtWidgets.QLabel('—',objectName='jointReading'); value.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)
            values.addWidget(value,1); single.addWidget(row,1)
            self.joint_rows.append(row); self.joint_values.append(value)
        self.motion_panel.addWidget(self.joint_feedback)
        self.pose_panel=QtWidgets.QWidget(); pose_grid=QtWidgets.QGridLayout(self.pose_panel)
        pose_grid.setContentsMargins(0,0,0,0); pose_grid.setHorizontalSpacing(10); pose_grid.setVerticalSpacing(12)
        self.pose_values=[]
        for index,(name,unit) in enumerate((('X','mm'),('Y','mm'),('Z','mm'),('Roll','°'),('Pitch','°'),('Yaw','°'))):
            cell=QtWidgets.QFrame(objectName='readout'); cell_layout=QtWidgets.QVBoxLayout(cell)
            cell_layout.setContentsMargins(10,12,10,12); cell_layout.setSpacing(8)
            cell_layout.addWidget(QtWidgets.QLabel(f'{name}  {unit}',objectName='axisName'))
            value=QtWidgets.QLabel('—',objectName='poseReading'); value.setAlignment(QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
            cell_layout.addWidget(value,1); self.pose_values.append(value)
            pose_grid.addWidget(cell,index//3,index%3)
        for column in range(3): pose_grid.setColumnStretch(column,1)
        for row in range(2): pose_grid.setRowStretch(row,1)
        self.motion_panel.addWidget(self.pose_panel); work.addWidget(self.motion_panel,1)
        workspace.addWidget(self.work_card,1)

        self.control_card=QtWidgets.QFrame(objectName='surface')
        control=QtWidgets.QVBoxLayout(self.control_card); control.setContentsMargins(20,16,20,16); control.setSpacing(14)
        control.addWidget(QtWidgets.QLabel('运动控制',objectName='sectionTitle'))
        self.joint_panel=QtWidgets.QWidget(); joint_layout=QtWidgets.QVBoxLayout(self.joint_panel)
        joint_layout.setContentsMargins(0,0,0,0); joint_layout.setSpacing(8)
        joint_layout.addWidget(QtWidgets.QLabel('选择关节'))
        joints=QtWidgets.QHBoxLayout(); joints.setSpacing(8); self.joint_buttons=[]
        for index in range(1,7):
            button=QtWidgets.QPushButton(f'J{index}'); button.setCheckable(True); button.setProperty('density','compact')
            button.setAccessibleName(f'选择关节 J{index}'); button.setMinimumWidth(0)
            button.clicked.connect(lambda checked=False, value=index:self._select_control('single_axis',value))
            joints.addWidget(button,1); self.joint_buttons.append(button)
        joint_layout.addLayout(joints); control.addWidget(self.joint_panel)
        self.selected_axis=QtWidgets.QLabel('当前 J1',objectName='selectedAxis'); control.addWidget(self.selected_axis)
        self.cartesian_frame=QtWidgets.QLabel('基座坐标系 · base_link',objectName='coordinateFrame'); control.addWidget(self.cartesian_frame)
        control.addStretch(1)
        self.speed_box=QtWidgets.QWidget(); speed_layout=QtWidgets.QVBoxLayout(self.speed_box)
        speed_layout.setContentsMargins(0,0,0,0); speed_layout.setSpacing(10)
        self.speed_label=QtWidgets.QLabel(objectName='speedValue')
        self.speed=QtWidgets.QSlider(QtCore.Qt.Horizontal); self.speed.setRange(5,100); self.speed.setSingleStep(5)
        self.speed.setValue(round(self.config['default_speed_scale']*100)); self.speed.setAccessibleName('当前控制模式速度比例')
        speed_layout.addWidget(self.speed_label); speed_layout.addWidget(self.speed); control.addWidget(self.speed_box)
        self.controls=QtWidgets.QWidget(); buttons=QtWidgets.QVBoxLayout(self.controls)
        buttons.setContentsMargins(0,0,0,0); buttons.setSpacing(0)
        self.start=QtWidgets.QPushButton('开始控制',objectName='primary'); self.end=QtWidgets.QPushButton('结束控制')
        self.recover=QtWidgets.QPushButton('恢复控制',objectName='primary'); self.stop=QtWidgets.QPushButton('停止 · B',objectName='stop')
        for button in (self.start,self.end,self.recover):
            button.setCursor(QtCore.Qt.PointingHandCursor); buttons.addWidget(button)
        self.stop.setCursor(QtCore.Qt.PointingHandCursor); self.stop.setMinimumHeight(76)
        control.addWidget(self.controls); workspace.addWidget(self.control_card,1); body.addLayout(workspace,1)
        self.speed.valueChanged.connect(self._speed_changed)
        self.start.clicked.connect(self._start); self.end.clicked.connect(self._end)
        self.recover.clicked.connect(self._recover); self.stop.clicked.connect(self._stop)

        self.details_panel=QtWidgets.QFrame(objectName='surface')
        details=QtWidgets.QHBoxLayout(self.details_panel); details.setContentsMargins(16,12,16,12); details.setSpacing(16)
        self.pad=GamepadWidget(); self.pad.setPalette(self.palette()); self.pad.setMinimumWidth(300); self.pad.setFixedHeight(220)
        details.addWidget(self.pad,1)
        self.diag=QtWidgets.QTextEdit(); self.diag.setReadOnly(True); self.diag.setMinimumHeight(220)
        details.addWidget(self.diag,2); self.details_panel.hide(); body.addWidget(self.details_panel)
        root.addWidget(self.stop)
        self.close_button=QtWidgets.QPushButton('关闭'); self.close_button.clicked.connect(self.close)
        self.close_button.setVisible(mode=='input'); root.addWidget(self.close_button)
        if mode=='input':
            self.status.setText('等待连接'); self.feedback_state.hide(); self.work_card.hide(); self.control_card.hide()
            self.selection_panel.hide(); self.controls.hide(); self.stop.hide()
            self.details_button.setText('输入诊断'); self.details_button.setChecked(True)
            body.setStretch(1,0); body.addStretch(1)
        self._refresh_controls(); self._render_selection({}); self.resize(self.initial_size)
        self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(50)

    def _toggle_details(self, show):
        self.details_panel.setVisible(show)
        self.details_button.setArrowType(QtCore.Qt.DownArrow if show else QtCore.Qt.RightArrow)
        self.layout().activate()
        if show:
            QtCore.QTimer.singleShot(0,lambda:self.body_scroll.ensureWidgetVisible(self.details_panel))

    def _set_status(self, text, tone='normal'):
        self.status.setText(text)
        if self.status.property('tone') != tone:
            self.status.setProperty('tone', tone)
            self.status.style().unpolish(self.status); self.status.style().polish(self.status)

    def _speed_changed(self,value):
        if self.bridge and not self._input_released():
            self._render_selection(self.bridge.status); return
        self._render_speed(value)
        if self.bridge: self.bridge.set_speed(value/100.)

    def _render_speed(self,value):
        status=self.bridge.status if self.bridge else {}
        if status.get('state') in HOME_STATES:
            self.speed_label.setText('回零速度 10%')
            self.speed_label.setToolTip('回零使用独立规划比例；两模式速度滑条不参与。')
            return
        limits=status.get('speed_limits',{})
        linear=limits.get('linear_m_s',self.config['linear_speed_max'])*1000*value/100
        angular=degrees(limits.get('angular_rad_s',self.config['angular_speed_max']))*value/100
        joint=degrees(limits.get('joint_rad_s',self.config['joint_speed']))*value/100
        self.speed_label.setText(f'速度 {value}%')
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
        for index,row in enumerate(self.joint_rows,1):
            selected=index==self.selected_joint
            if row.property('selected')!=selected:
                row.setProperty('selected',selected); row.style().unpolish(row); row.style().polish(row)
        self.joint_panel.setVisible(single)
        self.selected_axis.setVisible(single); self.selected_axis.setText(f'当前 J{self.selected_joint}')
        self.cartesian_frame.setVisible(not single)
        self.feedback_title.setText('关节实时状态' if single else '末端实时状态')
        self.motion_panel.setCurrentIndex(0 if single else 1)
        self.pad.set_control(None if self.mode=='input' else self.control_mode,self.selected_joint)
        scale=status.get('speed_scale')
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

    def _input_released(self,neutral=False):
        if not self.bridge: return True
        frame=self.bridge.input
        if (not frame or not frame.get('connected',True) or
                time.monotonic()-frame.get('received_at',0.)>.20 or
                frame.get('lb') or frame.get('b') or frame.get('home',False)):
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
            self.hint.setText(message)

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
                              (not self.bridge or (bool(status) and status.get('servo_switched') is True)))
        self.end.setText('结束控制')
        self.end.setEnabled(state in ('ARMED','ACTIVE','PREPARING','STOPPING','STOP_UNKNOWN')+HOME_STATES)
        self.recover.setEnabled(state in ('LOCKED','STOP_UNKNOWN') and
                                bool(status.get('stop_confirmed')) and self._input_released(neutral=True))
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
            self.feedback_state.setText('反馈中断'); return
        xyz=snapshot['xyz_m']; rpy=snapshot['rpy_rad']
        for label,value in zip(self.pose_values,(*[v*1000 for v in xyz],*[degrees(v) for v in rpy])):
            label.setText(f'{value:.1f}' if isfinite(value) else '—')
        joints=snapshot.get('joints_rad',[])
        for index,label in enumerate(self.joint_values):
            value=joints[index] if len(joints)==6 else None
            label.setText(f'{degrees(value):.3f}' if value is not None and isfinite(value) else '—')
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
            if not status.get('neutral_seen'): hint='松开 LB/B/Y，让摇杆和方向键回中。'
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
        if state in HOME_STATES:
            hint='保持 Y + LB · 松开取消 · 回零 10%'
        if self.reply_error is not None:
            if self.reply_error[0]==state: hint=self.reply_error[1]
            else: self.reply_error=None
        if self.selection_request_pending:
            hint='正在确认控制选择。'
        self._set_status(title,tone); self.hint.setText(hint)

    def refresh(self):
        if self.bridge:
            self.bridge.spin(); self.bridge.heartbeat()
            status=self.bridge.status; state=status.get('state','WAITING')
            self.state=state; self._render_selection(status); self._render_control_status(status)
            if (self.mode=='real' and self.bridge.guard_received and
                    time.monotonic()-self.bridge.guard_received>.35 and
                    (state in ('ACTIVE','PREPARING','STOPPING')+HOME_STATES or self.bridge.guard.get('state')=='running')):
                if not self.real_fallback_sent: self.real_fallback_sent=self.bridge.real_stop_fallback()
                self._set_status('控制连接中断','danger')
                self.hint.setText('已请求停止，等待真实反馈确认停稳。' if self.real_fallback_sent else '正在请求停止，请检查机械臂状态。')
            self._render_feedback(status)
            frame=self.bridge.input
            if frame and frame.get('connected',True) and time.monotonic()-frame['received_at']<=.20:
                self.pad.set_frame(frame)
                name=frame.get('device_name') or status.get('device_name') or '手柄'
                self.device.setText('模拟输入 · 非实体手柄' if frame['source']=='fixture' else '已连接 · '+name)
                self.device.setToolTip(str(status.get('device') or name))
            else:
                self.pad.set_frame(None); self.device.setText('手柄未连接或输入中断')
                self.device.setToolTip(status.get('input_error') or '未收到新鲜手柄输入')
            guard=self.bridge.guard
            self.diag.setPlainText(
                f"mode={status.get('mode')} / guard={guard.get('state','unknown')} / gate_closed={guard.get('gate_closed')}\n"
                f"设备={status.get('device_name')} / identity={status.get('device')} / generation={status.get('generation')} / input_age={status.get('input_age_s')}\n"
                f"连接失联验证={status.get('link_verified')} / 输入错误={status.get('input_error')}\n"
                f"控制状态={state} / 方向受限={status.get('joint_bound_limited',False)} / 停稳确认={guard.get('stop_confirmed')}\n"
                f"控制方式={status.get('control_mode')} / 关节={status.get('selected_joint')} / 选择版本={status.get('selection_revision')}\n"
                f"Y={bool(frame and frame.get('home',False))} / 回零按住进度={status.get('home_hold_progress')}\n"
                f"关节角 rad={(status.get('feedback') or {}).get('joints_rad')}\n"
                f"guard reason={guard.get('reason')}\n{self.bridge.last_reply}")
            if self.close_requested_at is not None:
                self._set_status('正在退出')
                self.hint.setText('正在停止机械臂，等待停稳确认。')
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
        elif self.mode=='input' and self.input_adapter:
            frame=self.input_adapter.latest()
            if frame:
                self.last_frame=frame; self.pad.set_frame(frame)
                self.device.setText('已连接 · '+frame['device']['name'])
                self.device.setToolTip(frame['device']['path'])
                self.diag.setPlainText(f"GUID: {frame['guid']}\n连接代数: {frame['generation']}\n摇杆: {frame['axes']}\n十字键: {frame['dpad']}\nLB={frame['lb']} B={frame['b']} Y={bool(frame.get('home',False))}\n失联验证: 未验证")
                self._set_status('手柄已连接','active')
                self.hint.clear()
            else:
                self.pad.set_frame(None); self.device.setText('手柄未连接')
                self._set_status('等待连接')
                self.hint.setText(self.input_adapter.error or '连接并开启手柄，程序会自动识别。')
        self.hint.setVisible(bool(self.hint.text()))
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
                self._set_status('正在退出')
                self.hint.setText('正在停止机械臂，等待停稳确认。')
                self.close_stop_requested=self._request('stop') is not None
            return
        self.timer.stop()
        if self.input_adapter: self.input_adapter.stop()
        event.accept()
