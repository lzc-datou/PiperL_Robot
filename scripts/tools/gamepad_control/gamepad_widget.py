from PyQt5 import QtCore, QtGui, QtWidgets
from scripts.tools.ui.theme import InstrumentSurface, LineDrawing
from .core import CONTROL_MODE_LABELS


class GamepadWidget(QtWidgets.QWidget):
    """Read-only stick instruments and explicit LB/B/Y button states."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.frame = None
        self.control_mode = None
        self.selected_axis = None
        self.setMinimumSize(300, 220)

    def set_frame(self, frame):
        self.frame = frame
        self.update()

    def set_control(self, control_mode, axis_name=None):
        self.control_mode, self.selected_axis = control_mode, axis_name
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        palette = self.palette()
        ink, line = palette.color(QtGui.QPalette.Text), palette.color(QtGui.QPalette.Mid)
        accent = palette.color(QtGui.QPalette.Link)
        font = self.font(); font.setPixelSize(18); p.setFont(font)
        axes = self.frame['axes'] if self.frame else (0.,)*4
        mode = CONTROL_MODE_LABELS.get(self.control_mode)
        title = '手柄原始输入'
        if mode:
            title = ' · '.join(filter(None, (mode, self.selected_axis, '原始输入')))
        if self.frame is None:
            title = '等待新鲜输入'
        p.setPen(ink)
        p.drawText(QtCore.QRect(0, 0, self.width(), 28), QtCore.Qt.AlignCenter, title)
        for index, name in enumerate(('左摇杆', '右摇杆')):
            x, y = int(self.width() * (.25 if index == 0 else .75)), 91
            p.setPen(QtGui.QPen(line, 1.5))
            p.setBrush(palette.color(QtGui.QPalette.AlternateBase))
            p.drawEllipse(QtCore.QPoint(x,y), 35, 35)
            p.drawLine(x-35,y,x+35,y); p.drawLine(x,y-35,x,y+35)
            ax, ay = axes[index*2:index*2+2]
            p.setPen(QtCore.Qt.NoPen); p.setBrush(accent)
            p.drawEllipse(QtCore.QPointF(x+ax*25,y+ay*25), 7, 7)
            p.setPen(ink)
            p.drawText(QtCore.QRect(x-75,31,150,25),QtCore.Qt.AlignCenter,name)
            values=f'{ax:+.2f}  /  {ay:+.2f}' if self.frame else '—  /  —'
            p.drawText(QtCore.QRect(x-75,132,150,28),QtCore.Qt.AlignCenter,values)
        width = (self.width()-16)/3
        for index, (key, name) in enumerate((('lb','LB'),('b','B'),('home','Y'))):
            down = bool(self.frame and self.frame.get(key, False))
            rect = QtCore.QRectF(index*(width+8),177,width,34)
            color = palette.color(QtGui.QPalette.BrightText) if key == 'b' else accent
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(color if down else palette.color(QtGui.QPalette.AlternateBase))
            p.drawRoundedRect(rect,5,5)
            p.setPen(palette.color(QtGui.QPalette.HighlightedText) if down else ink)
            state = '按下' if down else '松开' if self.frame else '—'
            p.drawText(rect,QtCore.Qt.AlignCenter,f'{name} {state}')
        p.end()


class MappingCard(InstrumentSurface):
    """Mode-specific reference diagram. Live button states remain in diagnostics."""
    _DRAWING = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 360 198">
      <g stroke-linecap="round" stroke-linejoin="round" stroke-width="2" stroke="#ecf1ed">
        <path fill="#847fec" d="M54 40q3-24 16-26 17-4 44-1l9 10z"/>
        <path fill="#26302d" d="m238 23 8-11q23-2 42 3 15 4 17 26z"/>
        <path fill="#26302d" d="M61 33q27-12 55-10l22 11h83l23-11q33-2 57 10
          19 14 30 49 14 43 18 76 5 31-13 31-14 3-34-18l-26-25q-13-12-32-12H118
          q-19 0-31 12l-28 26q-19 21-33 17-15-2-13-25 2-41 19-86 12-33 29-45z"/>
        <g fill="none"><circle cx="86" cy="65" r="23"/><circle cx="86" cy="65" r="17" stroke="#928dff" stroke-width="3"/>
          <circle cx="231" cy="101" r="21"/><circle cx="231" cy="101" r="15.5" stroke="RIGHT" stroke-width="2.7"/>
          <circle cx="280" cy="44" r="8"/><circle cx="262" cy="61" r="8"/><circle cx="298" cy="61" r="8"/>
          <circle cx="280" cy="79" r="8"/><circle cx="154" cy="66" r="4"/><circle cx="186" cy="66" r="4"/>
        </g>
        <path fill="#7670d7" d="M121 91h12v12h12v12h-12v12h-12v-12h-12v-12h12z"/>
      </g>ARROWS</svg>"""
    _ARROWS = ('<path d="M86 36v-12m-4 4 4-4 4 4M86 95v12m-4-4 4 4 4-4" '
               'fill="none" stroke="#8bdac4" stroke-width="2" stroke-linecap="round"/>'
               '<path d="m124.5 98 2.5-2.5 2.5 2.5 M124.5 120l2.5 2.5 2.5-2.5 '
               'M116 106.5l-2.5 2.5 2.5 2.5 M138 106.5l2.5 2.5-2.5 2.5" '
               'fill="none" stroke="#f4f3ff" stroke-width="1.45" stroke-linecap="round" stroke-linejoin="round"/>')

    @classmethod
    def _svg(cls, single):
        return cls._DRAWING.replace('RIGHT', '#d4dcd7' if single else '#928dff').replace(
            'ARROWS', cls._ARROWS if single else '')

    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(18, 13, 18, 14)
        layout.setSpacing(8)
        header = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel('手柄映射', objectName='instrumentTitle')
        header.addWidget(self.title)
        header.addStretch()
        self.permission = QtWidgets.QLabel('按住 LB 允许运动', objectName='onDark')
        header.addWidget(self.permission)
        layout.addLayout(header)
        self.diagram = LineDrawing(self._svg(True), '手柄按键映射示意')
        self.diagram.setMinimumHeight(122)
        layout.addWidget(self.diagram, 1)
        separator = QtWidgets.QFrame(objectName='mapRule')
        separator.setFixedHeight(1)
        layout.addWidget(separator)
        self.key = QtWidgets.QStackedWidget()
        for entries in (
            (('左摇杆', '上推  正向点动', '下拉  负向点动'),
             ('方向键', '左右  同行选轴', '上下  同列选轴'),
             ('选轴条件', '停稳、松开 LB/B/Y，摇杆回中')),
            (('左摇杆', '上下 移 X', '左右 移 Y'),
             ('右摇杆', '上下 移 Z', '左右 绕 Z'),
             ('方向键', '左右 绕 X', '上下 绕 Y')),
        ):
            page = QtWidgets.QWidget()
            grid = QtWidgets.QGridLayout(page)
            grid.setContentsMargins(1, 0, 1, 0)
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(4)
            for row, fields in enumerate(entries):
                for column, description in enumerate(fields):
                    span = 2 if len(fields) == 2 and column == 1 else 1
                    grid.addWidget(QtWidgets.QLabel(description, objectName='inputKey'), row, column, 1, span)
            self.key.addWidget(page)
        single_grid = self.key.widget(0).layout()
        single_grid.setHorizontalSpacing(12)
        for column, stretch in enumerate((2, 3, 3)):
            single_grid.setColumnStretch(column, stretch)
        layout.addWidget(self.key)
        self.single = None
        self.set_mode(True)

    def showEvent(self, event):
        super().showEvent(event)
        # Measure both legends after the window's stylesheet has reached their labels.
        self.key.setFixedHeight(self.key.sizeHint().height() + 4)

    def set_mode(self, single, axis=None):
        self.permission.setText(f'{axis} · 按住 LB 允许运动' if axis else '按住 LB 允许运动')
        if self.single == single:
            return
        self.single = single
        page = self.key.widget(0 if single else 1)
        self.key.setCurrentWidget(page)
        self.diagram.load(self._svg(single).encode())
        self.diagram.renderer().setAspectRatioMode(QtCore.Qt.KeepAspectRatio)


class ModeButton(QtWidgets.QPushButton):
    """The confirmed control mode owns the checked state."""
    def __init__(self, name):
        super().__init__(name, objectName='modeTab')
        self.setCheckable(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFixedWidth(132)
        self.setCursor(QtCore.Qt.PointingHandCursor)

    def nextCheckState(self):
        # The click requests a switch; the backend snapshot selects the button.
        pass

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QtCore.QEvent.EnabledChange:
            self.setCursor(QtCore.Qt.PointingHandCursor if self.isEnabled() else QtCore.Qt.ArrowCursor)


class StatusLine(QtWidgets.QLabel):
    """Paint one line while keeping the full text available to assistive technology."""
    def __init__(self, name):
        super().__init__(objectName=name)
        self.setTextFormat(QtCore.Qt.PlainText)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)

    def paintEvent(self, event):
        rect = self.contentsRect()
        summary = ' '.join(self.text().splitlines())
        text = self.fontMetrics().elidedText(summary, QtCore.Qt.ElideRight, rect.width())
        painter = QtGui.QPainter(self)
        painter.setPen(self.palette().color(self.foregroundRole()))
        painter.drawText(rect, QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, text)


class SessionStatus(QtWidgets.QPushButton):
    def __init__(self):
        super().__init__(objectName='sessionStatus')
        self.setAccessibleName('查看完整会话状态')
        self.setToolTip('查看完整状态说明')
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        layout = QtWidgets.QGridLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(0)
        self.title = StatusLine('status')
        self.hint = StatusLine('notice')
        layout.addWidget(self.title, 0, 0)
        layout.addWidget(self.hint, 1, 0)
        layout.setRowStretch(0, 1)
        layout.setRowStretch(1, 1)
        arrow = QtWidgets.QLabel('›', objectName='caption')
        arrow.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        layout.addWidget(arrow, 0, 1, 2, 1)
        layout.setColumnStretch(0, 1)
        self.dialog = None
        self.clicked.connect(self.show_details)
        self.set_message('等待开始')

    def set_message(self, title, hint='', tone='normal'):
        if (self.title.text(), self.hint.text(), self.title.property('tone')) == (title, hint, tone):
            return
        self.title.setText(title)
        self.hint.setText(hint)
        if self.title.property('tone') != tone:
            self.title.setProperty('tone', tone)
            self.title.style().unpolish(self.title)
            self.title.style().polish(self.title)
        self.setAccessibleDescription(self.full_text())
        if self.dialog and self.dialog.isVisible():
            self.detail_text.setPlainText(self.full_text())

    def full_text(self):
        return '\n\n'.join(filter(None, (self.title.text(), self.hint.text())))

    def show_details(self):
        if self.dialog is None:
            self.dialog = QtWidgets.QDialog(self, QtCore.Qt.Tool)
            self.dialog.setWindowTitle('手柄控制 · 状态详情')
            layout = QtWidgets.QVBoxLayout(self.dialog)
            self.detail_text = QtWidgets.QPlainTextEdit()
            self.detail_text.setReadOnly(True)
            layout.addWidget(self.detail_text)
            close = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
            close.rejected.connect(self.dialog.close)
            layout.addWidget(close)
            self.dialog.resize(480, 210)
        self.detail_text.setPlainText(self.full_text())
        self.dialog.show()
        self.dialog.raise_()


class DiagnosticTable(QtWidgets.QTableWidget):
    """Fixed-size rows and persistent items keep the reading position stable."""
    def __init__(self, input_only):
        if input_only:
            fields = ('设备', '连接方式', '蓝牙报告来源', '设备路径', 'GUID', '连接代数', '摇杆', '方向键', 'LB / B / Y',
                      '连接失联验证', '输入错误')
        else:
            fields = ('运行环境', '设备', '连接方式', '设备标识', '连接代数', '输入来源', '输入延迟（秒）',
                      '摇杆', '方向键', 'LB / B / Y', '连接失联验证', '输入错误',
                      '控制状态', '控制方式', '当前选轴', '选择版本', '方向受限',
                      'Guard 状态', '运动门关闭', '停稳确认', '回零确认进度',
                      '关节角（rad）', 'Guard 原因', '最近回复')
        super().__init__(len(fields), 2)
        self.setObjectName('diagnostics')
        self.setAccessibleName('输入诊断')
        self.setHorizontalHeaderLabels(('字段', '当前值'))
        self.horizontalHeader().setDefaultAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        self.setColumnWidth(0, 144)
        self.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.verticalHeader().hide()
        self.verticalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        self.verticalHeader().setDefaultSectionSize(32)
        self.setShowGrid(False)
        self.setWordWrap(False)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.setVerticalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self.setMinimumHeight(220)
        self.cells = {}
        for row, name in enumerate(fields):
            self.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
            value = QtWidgets.QTableWidgetItem('—')
            self.setItem(row, 1, value)
            self.cells[name] = value

    def display(self, values):
        for name, cell in self.cells.items():
            value = values.get(name)
            text = str(value) if value is not None and value != '' else '—'
            if cell.text() != text:
                cell.setText(text)
                cell.setToolTip(text)
