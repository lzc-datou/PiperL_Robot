"""Presentation tokens for the standalone robot workstations."""
from pathlib import Path
from PyQt5 import QtGui


def stylesheet(compact=False):
    values = {'BODY': 18 if compact else 22, 'SMALL': 16 if compact else 20,
              'TITLE': 26 if compact else 32, 'METRIC': 32 if compact else 40,
              'BUTTON': 22 if compact else 26, 'SECTION': 22 if compact else 26,
              'CONTROL': 44 if compact else 60, 'PADDING': 6 if compact else 8}
    source = (Path(__file__).parent / 'theme.qss').read_text(encoding='utf-8')
    for name, value in values.items():
        source = source.replace('@'+name+'@', str(value))
    return source.replace('@ASSET@', Path(__file__).parent.as_posix())


def apply_theme(widget, compact=False):
    palette = widget.palette()
    palette.setColor(QtGui.QPalette.Window, QtGui.QColor('#eef4fb'))
    palette.setColor(QtGui.QPalette.Base, QtGui.QColor('#ffffff'))
    palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor('#142a3a'))
    palette.setColor(QtGui.QPalette.Text, QtGui.QColor('#142a3a'))
    palette.setColor(QtGui.QPalette.AlternateBase, QtGui.QColor('#edf2f6'))
    palette.setColor(QtGui.QPalette.Highlight, QtGui.QColor('#103b70'))
    palette.setColor(QtGui.QPalette.Link, QtGui.QColor('#103b70'))
    palette.setColor(QtGui.QPalette.HighlightedText, QtGui.QColor('#ffffff'))
    palette.setColor(QtGui.QPalette.Mid, QtGui.QColor('#bacbd8'))
    palette.setColor(QtGui.QPalette.BrightText, QtGui.QColor('#d5222b'))
    widget.setPalette(palette)
    widget.setAutoFillBackground(True)
    font = QtGui.QFont('Noto Sans CJK SC')
    font.setPixelSize(17 if compact else 22)
    widget.setFont(font)
    widget.setStyleSheet(stylesheet(compact))
