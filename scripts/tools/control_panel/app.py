"""Readable CAN connection and preparation panel with a fixed stop action."""
import queue
from PyQt5 import QtCore, QtWidgets
from scripts.tools.ui.theme import apply_theme


class Panel(QtWidgets.QWidget):
    def __init__(self, backend, settings=None):
        super().__init__()
        self.backend = backend
        self.settings = settings
        self.port_signature = None
        self.last_message = '正在检测 CAN 端口'
        self.setObjectName('panel')
        self.setWindowTitle('PiPER-L · CAN 控制'+(' · 演示' if backend.demo else ''))
        screen = QtWidgets.QApplication.primaryScreen()
        available = screen.availableGeometry() if screen else QtCore.QRect(0, 0, 948, 768)
        width = max(1, min(900, available.width()-48))
        height = max(1, min(720, available.height()-48))
        self.setMinimumSize(min(680, width), min(560, height))
        self.resize(width, height)
        apply_theme(self)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(24, 16, 24, 16)
        layout.setSpacing(14)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel('CAN 控制', objectName='title'))
        header.addStretch()
        header.addWidget(QtWidgets.QLabel('PiPER-L'+(' · 离线演示' if backend.demo else ''), objectName='subtitle'))
        layout.addLayout(header)

        self.body_scroll = QtWidgets.QScrollArea()
        self.body_scroll.setWidgetResizable(True)
        self.body_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.body_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        self.body_scroll.setWidget(content)
        body = QtWidgets.QVBoxLayout(content)
        body.setContentsMargins(0, 0, 6, 0)
        body.setSpacing(14)
        layout.addWidget(self.body_scroll, 1)

        connection = QtWidgets.QFrame(objectName='surface')
        connection_row = QtWidgets.QHBoxLayout(connection)
        connection_row.setContentsMargins(16, 12, 16, 12)
        connection_row.setSpacing(10)
        self.ports = QtWidgets.QComboBox()
        self.ports.setAccessibleName('CAN 端口')
        self.ports.setMinimumWidth(0)
        self.ports.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.ports.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.ports.setMinimumContentsLength(12)
        self.ports.setMinimumHeight(76)
        self.ports.setToolTip('选择实际 CAN 适配器；连接使用 1 Mbps，不会自动使能机械臂')
        self.ports.activated.connect(self.select_port)
        self.connect_button = self.button('CAN 连接', 'connect', lambda: backend.submit('connect'))
        self.connect_button.setProperty('role', 'primary')
        self.connect_button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        connection_row.addWidget(self.ports, 1)
        connection_row.addWidget(self.connect_button)
        body.addWidget(connection)

        status_frame = QtWidgets.QFrame(objectName='surface')
        status_layout = QtWidgets.QVBoxLayout(status_frame)
        status_layout.setContentsMargins(16, 12, 16, 12)
        status_layout.setSpacing(6)
        self.status_heading = QtWidgets.QLabel('等待连接', objectName='status')
        self.status_heading.setProperty('state', 'neutral')
        self.status_heading.setProperty('emphasis', 'medium')
        self.status = QtWidgets.QLabel(self.last_message, objectName='body')
        self.status.setWordWrap(True)
        self.status.setTextFormat(QtCore.Qt.PlainText)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        status_layout.addWidget(self.status_heading)
        status_layout.addWidget(self.status)
        body.addWidget(status_frame)

        operation_frame = QtWidgets.QFrame(objectName='surface')
        operations = QtWidgets.QGridLayout(operation_frame)
        operations.setContentsMargins(16, 16, 16, 16)
        operations.setSpacing(12)
        operations.setColumnStretch(0, 1)
        operations.setColumnStretch(1, 1)
        operations.setRowStretch(0, 1)
        operations.setRowStretch(1, 1)
        self.enable = self.button('机械臂使能', 'enable', lambda: backend.submit('enable'))
        self.enable.setProperty('role', 'primary')
        self.stop = self.button('阻尼急停', 'stop', backend.emergency_stop)
        self.stop.setProperty('role', 'danger')
        self.stop.setMinimumHeight(80)
        self.home = self.button('机械臂回零', 'home', lambda: backend.submit('home'))
        self.reset = self.button('重置', 'reset', lambda: backend.submit('reset'))
        self.home.setProperty('role', 'primary')
        self.reset.setProperty('role', 'primary')
        for button in (self.enable, self.home, self.reset):
            button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        operations.addWidget(self.enable, 0, 0, 1, 2)
        operations.addWidget(self.home, 1, 0)
        operations.addWidget(self.reset, 1, 1)
        body.addWidget(operation_frame, 1)
        layout.addWidget(self.stop)
        self.home.setToolTip(f'先确认活动空间。六轴目标角度 0°，普通 MOVE J，速度比例 {backend.home_speed}%；不修改零点标定')
        self.enable.setToolTip('恢复 CAN / MOVE J 模式并确认六轴使能；急停后请先重置')
        self.stop.setToolTip('阻尼急停可能随负载移动，不代替物理急停。发送 150#0100000000000000')
        self.reset.setToolTip('先支撑机械臂。发送 150#0200000000000000；可能解除保持；等待约 1 秒后再使能')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(80)
        self.refresh()

    @staticmethod
    def button(text, name, callback):
        button = QtWidgets.QPushButton(text, objectName=name)
        button.setMinimumHeight(76)
        button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        button.setCursor(QtCore.Qt.PointingHandCursor)
        button.clicked.connect(callback)
        return button

    def select_port(self, index):
        channel = self.ports.itemData(index)
        self.backend.select(channel)
        if self.settings is not None and channel:
            self.settings.setValue('last_port', channel)
        self.refresh()

    def refresh(self):
        state = self.backend.snapshot()
        signature = (tuple(state['interfaces'].items()), state['channel'])
        if signature != self.port_signature:
            self.port_signature = signature
            self.ports.blockSignals(True)
            self.ports.clear()
            self.ports.addItem('请选择 CAN 端口', None)
            for name, info in state['interfaces'].items():
                up, bitrate, health = info[:3]
                rate = f'{bitrate / 1000000:g} Mbps' if bitrate else '速率未设置'
                self.ports.addItem(f'{name}  ·  {"UP" if up else "DOWN"}  ·  {rate}'
                                   + (f'  ·  {health}' if up and health != 'ERROR-ACTIVE' else ''), name)
            if state['channel'] and state['channel'] not in state['interfaces']:
                self.ports.addItem(f'{state["channel"]}  ·  不存在', state['channel'])
            self.ports.setCurrentIndex(max(0, self.ports.findData(state['channel'])))
            self.ports.blockSignals(False)
        self.connect_button.setEnabled(state['can_connect'] and not state['connected'] and not state['busy'])
        self.connect_button.setText('CAN 已连接' if state['connected'] else 'CAN 连接')
        self.enable.setEnabled(state['connected'] and not state['busy'])
        self.reset.setEnabled(state['connected'] and not state['busy'])
        self.home.setEnabled(state['connected'] and not state['busy'])
        self.stop.setEnabled(state['connected'])
        while True:
            try:
                message = self.backend.events.get_nowait()
            except queue.Empty:
                break
            self.last_message = message
            self.status.setText(message)
            self.status.setToolTip(message)
            print(message, flush=True)
        if any(word in self.last_message for word in ('失败', '异常', '超时', 'BUS-OFF', '不存在')):
            heading, tone = '需要检查', 'danger'
        elif state['busy']:
            heading, tone = '正在处理', 'warning'
        elif state['connected']:
            heading, tone = 'CAN 已连接', 'ok'
        else:
            heading, tone = '等待 CAN 连接', 'neutral'
        self.status_heading.setText(heading)
        if self.status_heading.property('state') != tone:
            self.status_heading.setProperty('state', tone)
            self.status_heading.style().unpolish(self.status_heading)
            self.status_heading.style().polish(self.status_heading)

    def closeEvent(self, event):
        self.timer.stop()
        self.backend.close()
        event.accept()
