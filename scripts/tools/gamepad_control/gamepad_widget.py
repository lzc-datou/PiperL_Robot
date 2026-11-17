from PyQt5 import QtCore, QtGui, QtWidgets


class GamepadWidget(QtWidgets.QWidget):
    """Read-only painter for the latest raw input frame."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.frame = None
        self.setMinimumSize(260, 170)

    def set_frame(self, frame):
        self.frame = frame
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#f6f8fb'))
        pen = QtGui.QPen(QtGui.QColor('#8794a4'), 2)
        p.setPen(pen)
        p.setBrush(QtGui.QColor('#e7ebf0'))
        r = self.rect().adjusted(18, 22, -18, -16)
        p.drawRoundedRect(r, 48, 48)
        p.setPen(QtGui.QColor('#52657a'))
        p.drawText(QtCore.QRect(18, 0, self.width()-36, 22), QtCore.Qt.AlignCenter, '冰原狼 2 · 输入示意')
        axes = self.frame['axes'] if self.frame else (0.,)*4
        for cx, cy, ax, ay in ((.34,.58,axes[0],axes[1]),(.66,.58,axes[2],axes[3])):
            x, y = int(self.width()*cx), int(self.height()*cy)
            p.setBrush(QtGui.QColor('white')); p.drawEllipse(x-28,y-28,56,56)
            p.setBrush(QtGui.QColor('#3d6c9e')); p.drawEllipse(x+int(ax*17)-9,y+int(ay*17)-9,18,18)
        lb = bool(self.frame and self.frame['lb'])
        p.setPen(QtCore.Qt.NoPen); p.setBrush(QtGui.QColor('#d9534f' if lb else '#b8c2ce'))
        p.drawRoundedRect(48, 29, 58, 12, 5, 5)
        p.setPen(QtGui.QColor('#243244')); p.drawText(48, 53, 'LB')
        p.setBrush(QtGui.QColor('#d9534f' if self.frame and self.frame['b'] else '#b8c2ce'))
        p.drawEllipse(self.width()-71, 43, 24, 24); p.setPen(QtGui.QColor('#243244'))
        p.drawText(self.width()-64, 60, 'B')
        p.end()
