"""Offscreen five-button UI test; demo mode cannot open a real socket."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import time
import unittest
from unittest.mock import patch
from PyQt5 import QtCore, QtWidgets, QtTest
from scripts.tools.control_panel.app import Panel
from scripts.tools.control_panel.backend import Backend


class PanelUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.app.setQuitOnLastWindowClosed(False)

    def until(self,predicate):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            self.app.processEvents()
            if predicate(): return
            time.sleep(0.01)
        self.fail('GUI condition timed out')

    def test_five_buttons_auto_connection_and_direct_actions(self):
        with patch('socket.socket',side_effect=AssertionError('real socket forbidden')):
            backend=Backend('can0',demo=True)
            panel=Panel(backend)
            panel.show()
            try:
                self.assertFalse(panel.art.image.isNull())
                self.assertTrue(panel.art.image.hasAlphaChannel())
                self.assertEqual(len(panel.findChildren(QtWidgets.QPushButton)),5)
                self.assertFalse(panel.findChildren(QtWidgets.QTableWidget))
                self.assertFalse(panel.findChildren(QtWidgets.QTabWidget))
                self.until(lambda: panel.reset.isEnabled())
                # Reset is a direct command, including before the first stop.
                for button,frame in ((panel.reset,'重置指令已发送'),(panel.enable,'模拟确认'),(panel.stop,'150#01'),(panel.home,'回零指令已发送')):
                    QtTest.QTest.mouseClick(button,QtCore.Qt.LeftButton)
                    self.until(lambda: frame in panel.status.text())
                    self.until(lambda: not backend.busy)
                    panel.refresh()
                    self.assertEqual(len(panel.findChildren(QtWidgets.QPushButton)),5)
                self.until(lambda: panel.enable.isEnabled() and panel.reset.isEnabled())
                screen = self.app.primaryScreen().availableGeometry()
                self.assertLessEqual(panel.width(), screen.width())
                self.assertLessEqual(panel.height(), screen.height())
                panel.resize(800, 680)
                self.app.processEvents()
                for button in panel.findChildren(QtWidgets.QPushButton):
                    self.assertGreaterEqual(button.height(), 44)
                    rect = QtCore.QRect(button.mapTo(panel, QtCore.QPoint()), button.size())
                    self.assertTrue(panel.rect().contains(rect), (button.text(), rect))
                self.assertFalse(panel.body_scroll.isAncestorOf(panel.stop))
            finally:
                panel.close()
                self.app.processEvents()

    def test_dropdown_selection_manual_activation_and_saved_preference(self):
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory, patch('socket.socket', side_effect=AssertionError('real socket forbidden')):
            settings = QtCore.QSettings(directory+'/panel.ini', QtCore.QSettings.IniFormat)
            backend = Backend(demo=True)
            panel = Panel(backend, settings)
            panel.show()
            try:
                self.until(lambda: panel.ports.count() == 3)
                self.assertFalse(panel.connect_button.isEnabled())
                self.assertFalse(panel.home.isEnabled())
                index = panel.ports.findData('can1')
                panel.ports.setCurrentIndex(index)
                panel.ports.activated.emit(index)
                self.until(lambda: panel.connect_button.isEnabled())
                self.assertEqual(settings.value('last_port'), 'can1')
                QtTest.QTest.mouseClick(panel.connect_button, QtCore.Qt.LeftButton)
                self.until(lambda: panel.home.isEnabled())
                index = panel.ports.findData('can0')
                panel.ports.setCurrentIndex(index)
                panel.ports.activated.emit(index)
                self.until(lambda: panel.home.isEnabled() and backend.channel == 'can0')
                self.assertEqual(settings.value('last_port'), 'can0')
                self.assertEqual(panel.connection_state.text(), '已连接')
                self.assertEqual(panel.rate.text(), '1 Mbps')
                self.assertFalse(panel.connect_button.isEnabled())
            finally:
                panel.close()
                self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
