"""Visual components shared by the standalone PiPER-L workstations."""
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtSvg, QtWidgets


def stylesheet(compact=False):
    source = (Path(__file__).parent / 'theme.qss').read_text(encoding='utf-8')
    return source.replace('@BODY@', '16' if compact else '19').replace(
        '@ASSET@', Path(__file__).parent.as_posix())


def apply_theme(widget, compact=False):
    palette = widget.palette()
    for role, color in (
        (QtGui.QPalette.Window, '#f5f5f2'), (QtGui.QPalette.Base, '#ffffff'),
        (QtGui.QPalette.WindowText, '#222b2a'), (QtGui.QPalette.Text, '#222b2a'),
        (QtGui.QPalette.AlternateBase, '#eeefed'), (QtGui.QPalette.Highlight, '#5152cc'),
        (QtGui.QPalette.Link, '#5152cc'), (QtGui.QPalette.HighlightedText, '#ffffff'),
        (QtGui.QPalette.Mid, '#abb3b0'), (QtGui.QPalette.BrightText, '#bd2a3c'),
    ):
        palette.setColor(role, QtGui.QColor(color))
    widget.setPalette(palette)
    widget.setAutoFillBackground(True)
    widget.setStyleSheet(stylesheet(compact))


class InstrumentSurface(QtWidgets.QFrame):
    """Dark drafting surface shared by the hardware illustrations."""
    def __init__(self):
        super().__init__(objectName='instrument')
        tile = QtGui.QPixmap(19, 19)
        tile.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(tile)
        painter.setPen(QtGui.QColor('#414c49'))
        painter.drawPoint(9, 9)
        painter.end()
        self.grid = QtGui.QBrush(tile)

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect().adjusted(6, 6, -6, -6), self.grid)


class LineDrawing(QtSvg.QSvgWidget):
    """A scalable static hardware diagram, independent of live telemetry."""
    def __init__(self, svg, description):
        super().__init__()
        self.load(svg.encode())
        self.renderer().setAspectRatioMode(QtCore.Qt.KeepAspectRatio)
        self.setAccessibleName(description)
        self.setToolTip(description)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)


class StatusIndicator(QtWidgets.QLabel):
    def __init__(self, text):
        super().__init__(text, objectName='indicator')
        self.online = False
        self.setTextFormat(QtCore.Qt.PlainText)
        self.setMargin(0)
        self.setIndent(0)
        self.setContentsMargins(17, 0, 0, 0)

    def set_online(self, online):
        if self.online != online:
            self.online = online
            self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor('#258566' if self.online else '#858b88'))
        painter.drawEllipse(QtCore.QPointF(6, self.height() / 2), 4.5, 4.5)


def action_icon(action):
    paths = {
        'stop': '<path d="M8 2h8l6 6v8l-6 6H8l-6-6V8z"/><rect x="8" y="8" width="8" height="8" rx="1" fill="white" stroke="none"/>',
        'home': '<path d="m8 5-6 6 6 6M3 11h12a5 5 0 0 1 0 10h-4"/>',
        'reset': '<path d="M20 9a8.4 8.4 0 1 0 0 7M20 3v6h-6"/>',
    }
    color = 'white' if action == 'stop' else '#35403f'
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">'
           f'<g fill="none" stroke="{color}" stroke-width="1.9" stroke-linecap="round" '
           f'stroke-linejoin="round">{paths[action]}</g></svg>')
    pixmap = QtGui.QPixmap(72, 72)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    QtSvg.QSvgRenderer(svg.encode()).render(painter)
    painter.end()
    return QtGui.QIcon(pixmap)
