from PyQt5 import QtCore, QtGui, QtWidgets


class GamepadWidget(QtWidgets.QWidget):
    """Read-only stick instruments and explicit LB/B/Y button states."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.frame = None
        self.control_mode = None
        self.selected_joint = 1
        self.setMinimumSize(300, 220)

    def set_frame(self, frame):
        self.frame = frame
        self.update()

    def set_control(self, control_mode, joint_index=1):
        self.control_mode, self.selected_joint = control_mode, joint_index
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        palette = self.palette()
        ink, line = palette.color(QtGui.QPalette.Text), palette.color(QtGui.QPalette.Mid)
        accent = palette.color(QtGui.QPalette.Link)
        font = self.font(); font.setPixelSize(18); p.setFont(font)
        axes = self.frame['axes'] if self.frame else (0.,)*4
        title = ('等待新鲜输入' if self.frame is None else
                 f'单轴 J{self.selected_joint} · 原始输入' if self.control_mode == 'single_axis' else
                 '笛卡尔 · 原始输入' if self.control_mode == 'cartesian' else '手柄原始输入')
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
