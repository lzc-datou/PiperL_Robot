"""Readable CAN connection and preparation panel with a fixed stop action."""
import queue
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets
from scripts.tools.ui.theme import InstrumentSurface, StatusIndicator, action_icon, apply_theme


class ArmSketch(QtWidgets.QWidget):
    """Approved PiPER-L model drawing. Its pose is illustrative."""
    def __init__(self):
        super().__init__()
        asset = Path(__file__).resolve().parents[3] / 'outputs/generated_images/control_panel/piper_l_sketch.png'
        self.image = QtGui.QPixmap(str(asset)).copy(124, 28, 754, 835)
        self.setAccessibleName('PiPER-L 本体静态线稿')
        self.setToolTip('PiPER-L 本体 · 静态结构示意')
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)

    def sizeHint(self):
        return QtCore.QSize(340, 280)

    def paintEvent(self, event):
        if self.image.isNull():
            return
        bounds = self.rect().adjusted(8, 7, -8, -7)
        size = self.image.size().scaled(bounds.size(), QtCore.Qt.KeepAspectRatio)
        target = QtCore.QRect(QtCore.QPoint(), size)
        target.moveCenter(bounds.center())
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        painter.drawPixmap(target, self.image, self.image.rect())


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
        self.setMinimumSize(min(520, width), min(480, height))
        self.resize(width, height)
        apply_theme(self)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 12, 20, 16)
        layout.setSpacing(16)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel('PiPER-L    /    CAN 控制', objectName='title'))
        header.addStretch()
        header.addWidget(QtWidgets.QLabel('○  离线预览' if backend.demo else '实机 · SocketCAN', objectName='mode'))
        layout.addLayout(header)
        rule = QtWidgets.QFrame(objectName='rule')
        rule.setFixedHeight(1)
        layout.addWidget(rule)

        self.body_scroll = QtWidgets.QScrollArea()
        self.body_scroll.setWidgetResizable(True)
        self.body_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        content = QtWidgets.QWidget()
        self.body_scroll.setWidget(content)
        body = QtWidgets.QVBoxLayout(content)
        body.setSizeConstraint(QtWidgets.QLayout.SetMinAndMaxSize)
        body.setContentsMargins(1, 1, 1, 1)
        body.setSpacing(18)
        layout.addWidget(self.body_scroll, 1)

        connection = QtWidgets.QHBoxLayout()
        connection.setSpacing(12)
        connection.addWidget(QtWidgets.QLabel('CAN 端口'))
        self.ports = QtWidgets.QComboBox()
        self.ports.setAccessibleName('CAN 端口')
        self.ports.setMinimumContentsLength(5)
        self.ports.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.ports.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.ports.activated.connect(self.select_port)
        self.ports.setMaximumWidth(240)
        connection.addWidget(self.ports, 2)
        self.connection_state = StatusIndicator('待选择')
        self.rate = QtWidgets.QLabel('—', objectName='caption')
        connection.addWidget(self.connection_state)
        connection.addWidget(self.rate)
        connection.addStretch(1)
        self.connect_button = self.button('连接', 'connect', lambda: backend.submit('connect'))
        self.connect_button.setAccessibleName('CAN 连接')
        connection.addWidget(self.connect_button)
        body.addLayout(connection)

        hero = InstrumentSurface()
        hero_layout = QtWidgets.QHBoxLayout(hero)
        hero_layout.setContentsMargins(25, 20, 16, 20)
        hero_layout.setSpacing(10)
        info = QtWidgets.QVBoxLayout()
        info.setSpacing(10)
        info.addStretch()
        self.status_heading = QtWidgets.QLabel('最近操作', objectName='eyebrow')
        self.status = QtWidgets.QLabel(self.last_message, objectName='lastEvent')
        self.status.setTextFormat(QtCore.Qt.PlainText)
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        info.addWidget(self.status_heading)
        info.addWidget(self.status)
        info.addSpacing(8)
        self.enable = self.button('机械臂使能', 'enable', lambda: backend.submit('enable'))
        self.enable.setMinimumSize(174, 56)
        info.addWidget(self.enable, 0, QtCore.Qt.AlignLeft)
        info.addStretch()
        hero_layout.addLayout(info, 6)
        self.art = ArmSketch()
        hero_layout.addWidget(self.art, 5)
        body.addWidget(hero, 1)

        operations = QtWidgets.QHBoxLayout()
        operations.setSpacing(14)
        self.home = self.button('机械臂回零', 'home', lambda: backend.submit('home'))
        self.reset = self.button('重置', 'reset', lambda: backend.submit('reset'))
        for button in (self.home, self.reset):
            button.setIcon(action_icon(button.objectName()))
            button.setIconSize(QtCore.QSize(22, 22))
            button.setMinimumHeight(62)
            operations.addWidget(button, 1)
        body.addLayout(operations)
        self.stop = self.button('阻尼急停', 'stop', backend.emergency_stop)
        self.stop.setIcon(action_icon('stop'))
        self.stop.setIconSize(QtCore.QSize(28, 28))
        self.stop.setMinimumHeight(66)
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
        button.setMinimumHeight(44)
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
                index = self.ports.count()
                self.ports.addItem(name, name)
                self.ports.setItemData(index, f'{name} · {"UP" if up else "DOWN"} · {rate} · {health}',
                                       QtCore.Qt.ToolTipRole)
            if state['channel'] and state['channel'] not in state['interfaces']:
                self.ports.addItem(state['channel'] + ' · 不存在', state['channel'])
            self.ports.setCurrentIndex(max(0, self.ports.findData(state['channel'])))
            self.ports.blockSignals(False)
            self.ports.setToolTip(self.ports.currentData(QtCore.Qt.ToolTipRole) or '选择实际 CAN 端口')
        interface = state['interfaces'].get(state['channel'])
        link = '未连接' if state['channel'] else '待选择'
        if interface and not state['connected']:
            link = '未启用' if not interface[0] else (interface[2] or link)
            if link == 'ERROR-ACTIVE':
                link = '未连接'
        self.connection_state.setText('已连接' if state['connected'] else link)
        self.connection_state.set_online(state['connected'])
        bitrate = interface[1] if interface else None
        self.rate.setText(f'{bitrate / 1000000:g} Mbps' if bitrate else '—')
        self.connect_button.setEnabled(state['can_connect'] and not state['connected'] and not state['busy'])
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
        else:
            heading, tone = '最近操作', 'neutral'
        self.art.setVisible(self.width() >= 650 and tone != 'danger')
        self.status_heading.setText(heading)
        if self.status_heading.property('state') != tone:
            self.status_heading.setProperty('state', tone)
            self.status_heading.style().unpolish(self.status_heading)
            self.status_heading.style().polish(self.status_heading)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.art.setVisible(event.size().width() >= 650 and self.status_heading.property('state') != 'danger')

    def closeEvent(self, event):
        self.timer.stop()
        self.backend.close()
        event.accept()
