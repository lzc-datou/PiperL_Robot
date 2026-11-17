"""Large five-button CAN remote with interface selection."""
import queue
from PyQt5 import QtCore, QtWidgets


class Panel(QtWidgets.QWidget):
    def __init__(self, backend, settings=None):
        super().__init__()
        self.backend = backend
        self.settings = settings
        self.port_signature = None
        self.setObjectName('panel')
        self.setWindowTitle('PiPER-L · CAN 控制'+(' · 演示' if backend.demo else ''))
        self.resize(640, 560)
        self.setMinimumSize(600, 560)
        self.setStyleSheet('''
            QWidget#panel { background: #f8f9fb; }
            QLabel { color: #243244; background: transparent; }
            QLabel#title { font-size: 26px; font-weight: 600; }
            QLabel#subtitle { font-size: 16px; color: #778291; }
            QPushButton { background: #ffffff; color: #243244;
                border: 1px solid #dce2e9; border-radius: 12px;
                font-size: 24px; font-weight: 500; }
            QPushButton:hover { background: #edf2f8; border-color: #b8c7db; }
            QPushButton:pressed { background: #dde7f3; }
            QPushButton:disabled { color: #a8b1bd; background: #f2f4f7; border-color: #e5e9ee; }
            QPushButton#connect { color: white; background: #2e5680; border: none; }
            QPushButton#connect:hover { background: #24486e; }
            QPushButton#connect:disabled { background: #e8eef4; color: #647e98; }
            QPushButton#stop { color: white; background: #be3c46; border: none; }
            QPushButton#stop:hover { background: #a92d37; }
            QPushButton#stop:pressed { background: #93232d; }
            QPushButton#stop:disabled { color: #c2979b; background: #f3e9eb; }
            QComboBox { font-size: 18px; padding: 7px 12px; background: white;
                border: 1px solid #dce2e9; border-radius: 8px; color: #243244; }
            QComboBox QAbstractItemView { font-size: 18px; }
            QLabel#status { font-size: 18px; color: #52657a; }
            QLabel#hint { font-size: 14px; color: #778291; }
        ''')
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(24, 18, 24, 18)
        layout.setSpacing(0)
        layout.addWidget(QtWidgets.QLabel('PiPER-L', objectName='title'))
        subtitle = QtWidgets.QLabel('CAN 控制  /  1 Mbps'+('  ·  离线演示' if backend.demo else ''), objectName='subtitle')
        layout.addSpacing(2)
        layout.addWidget(subtitle)
        layout.addSpacing(8)
        self.ports = QtWidgets.QComboBox()
        self.ports.setAccessibleName('CAN 端口')
        self.ports.activated.connect(self.select_port)
        layout.addWidget(self.ports)
        layout.addSpacing(10)
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(12)
        self.connect_button = self.button('CAN 连接', 'connect', lambda: backend.submit('connect'))
        self.enable = self.button('机械臂使能', 'enable', lambda: backend.submit('enable'))
        self.stop = self.button('阻尼急停', 'stop', backend.emergency_stop)
        self.home = self.button('机械臂回零', 'home', lambda: backend.submit('home'))
        self.reset = self.button('重置', 'reset', lambda: backend.submit('reset'))
        for row, buttons in enumerate(((self.connect_button, self.enable), (self.home, self.reset))):
            for column, button in enumerate(buttons):
                grid.addWidget(button, row, column)
        grid.addWidget(self.stop, 2, 0, 1, 2)
        layout.addLayout(grid, 1)
        layout.addSpacing(12)
        self.status = QtWidgets.QLabel('正在检测 CAN 端口', objectName='status')
        self.status.setMinimumHeight(44)
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch()
        layout.addWidget(QtWidgets.QLabel('回零前确认活动空间；重置前支撑机械臂。', objectName='hint'))
        self.home.setToolTip(f'六轴目标角度 0°，普通 MOVE J，速度比例 {backend.home_speed}%；不修改零点标定')
        self.enable.setToolTip('恢复 CAN / MOVE J 模式并确认六轴使能；急停后请先重置')
        self.stop.setToolTip('发送 150#0100000000000000')
        self.reset.setToolTip('发送 150#0200000000000000；可能解除保持；等待约 1 秒后再使能')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(80)
        self.refresh()

    @staticmethod
    def button(text, name, callback):
        button = QtWidgets.QPushButton(text, objectName=name)
        button.setMinimumHeight(100)
        button.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
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
            self.status.setText(message[:100])
            self.status.setToolTip(message)
            print(message, flush=True)

    def closeEvent(self, event):
        self.timer.stop()
        self.backend.close()
        event.accept()
