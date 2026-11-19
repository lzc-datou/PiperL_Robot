"""Exercise real disposable process groups, including a SIGKILLed Qt owner."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import Mock, patch

from scripts.task.common.process_cleanup import live_group_members
from scripts.tools.gamepad_control import run
from scripts.tools.gamepad_control.stack_supervisor import start_stack, stop_stack


ROOT = Path(__file__).resolve().parents[1]
TREE = r'''
import json, os, signal, subprocess, sys, time
from pathlib import Path
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
Path(sys.argv[1]).write_text(json.dumps({'pid': os.getpid(), 'child': child.pid}))
while True: time.sleep(.05)
'''


@unittest.skipUnless(sys.platform == 'linux', 'Linux process groups and /proc')
class GamepadStackCleanupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.logs = Path(self.temp.name)
        self.processes = []
        self.groups = set()
        # Reap only fixture descendants; do not leave orphans to container init.
        self.libc = ctypes.CDLL(None, use_errno=True)
        self.previous = ctypes.c_int()
        self.assertEqual(self.libc.prctl(37, ctypes.byref(self.previous), 0, 0, 0), 0)
        self.assertEqual(self.libc.prctl(36, 1, 0, 0, 0), 0)

    def tearDown(self):
        for pgid in self.groups:
            try: os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError: pass
        for process in self.processes:
            if process.poll() is None: process.kill()
            process.wait(timeout=3)
        for pgid in self.groups:
            try:
                while True: os.waitpid(-pgid, 0)
            except ChildProcessError:
                pass
        self.libc.prctl(36, self.previous.value, 0, 0, 0)
        self.temp.cleanup()

    def read_json(self, path, timeout=8.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            try: return json.loads(path.read_text())
            except (FileNotFoundError, json.JSONDecodeError): time.sleep(.02)
        self.fail(f'No complete report: {path}')

    def launch(self):
        with (self.logs/'stack.log').open('w') as logfile:
            process, lifeline = start_stack(
                [sys.executable, '-c', TREE, str(self.logs/'ready.json')], logfile, self.logs)
        self.processes.append(process)
        self.groups.add(process.pid)
        return process, lifeline

    def test_normal_close_cleans_stack_and_leaves_unrelated_group_alive(self):
        process, lifeline = self.launch()
        tree = self.read_json(self.logs/'ready.json')
        self.groups.add(tree['pid'])
        unrelated = subprocess.Popen([sys.executable, '-c', 'import time;time.sleep(60)'], start_new_session=True)
        self.processes.append(unrelated)
        self.groups.add(unrelated.pid)
        report = stop_stack(process, lifeline, self.logs)
        self.assertEqual(report['remaining_pids'], [])
        self.assertEqual(report['errors'], [])
        self.assertEqual(report['reason'], 'ui_shutdown')
        self.assertFalse(live_group_members(tree['pid']))
        self.assertIsNone(unrelated.poll())

    def test_immediate_shutdown_during_supervisor_startup_leaves_no_stack(self):
        process, lifeline = self.launch()
        report = stop_stack(process, lifeline, self.logs)
        self.groups.add(report['pid'])
        self.assertFalse(report['remaining_pids'])
        self.assertFalse(live_group_members(report['pid']))

    def test_sigkill_owner_still_reclaims_the_whole_stack(self):
        owner_code = r'''
import sys, time
from pathlib import Path
from scripts.tools.gamepad_control.stack_supervisor import start_stack
logs = Path(sys.argv[1])
with (logs/'stack.log').open('w') as logfile:
    process, lifeline = start_stack([sys.executable, '-c', sys.argv[2], str(logs/'ready.json')], logfile, logs)
while True: time.sleep(.1)
'''
        owner = subprocess.Popen([sys.executable, '-c', owner_code, str(self.logs), TREE], cwd=ROOT, start_new_session=True)
        self.processes.append(owner)
        self.groups.add(owner.pid)
        meta = self.read_json(self.logs/'stack_process.json')
        self.groups.update((meta['pid'], meta['supervisor_pid']))
        self.read_json(self.logs/'ready.json')
        owner.kill()
        owner.wait(timeout=3)
        report = self.read_json(self.logs/'cleanup.json')
        self.assertEqual(report['reason'], 'ui_process_exited')
        self.assertFalse(report['remaining_pids'])
        self.assertFalse(live_group_members(meta['pid']))

    def test_qt_initialization_failure_never_starts_a_stack(self):
        from PyQt5 import QtWidgets
        handles = [Mock()]
        args = argparse.Namespace(mode='sim', input_source='fixture', device=None,
                                  verified_link=False, verified_mapping=False)
        with patch.object(run, '_acquire', return_value=handles), \
                patch.object(QtWidgets, 'QApplication', side_effect=RuntimeError('Qt startup failed')), \
                patch('scripts.tools.gamepad_control.stack_supervisor.start_stack') as spawn:
            with self.assertRaisesRegex(RuntimeError, 'Qt startup failed'):
                run._run_stack(args, argparse.ArgumentParser())
        spawn.assert_not_called()
        handles[0].close.assert_called_once()

    def test_event_loop_exception_cleans_stack_even_if_bridge_cleanup_raises(self):
        from PyQt5 import QtCore, QtWidgets
        args = argparse.Namespace(mode='sim', input_source='fixture', device=None,
                                  verified_link=False, verified_mapping=False)
        bridge = Mock()
        bridge.destroy_node.side_effect = RuntimeError('bridge cleanup failed')
        fake_ros = Mock()
        bridge_module = types.ModuleType('scripts.tools.gamepad_control.ros_bridge')
        bridge_module.init_bridge = lambda: bridge
        runs = []
        def spawn(command, logfile, logs):
            process, lifeline = start_stack(
                [sys.executable, '-c', TREE, str(logs/'ready.json')], logfile, logs)
            self.processes.append(process)
            self.groups.add(process.pid)
            meta = self.read_json(logs/'stack_process.json')
            self.groups.add(meta['pid'])
            self.read_json(logs/'ready.json')
            runs.append(logs)
            return process, lifeline
        with patch.object(run, 'ROOT', self.logs), patch.object(run, '_acquire', return_value=[]), \
                patch.object(run, '_preflight_ros', return_value=fake_ros), \
                patch.object(QtWidgets, 'QApplication'), patch.object(QtCore, 'QTimer'), \
                patch('scripts.tools.gamepad_control.app.GamepadWindow'), \
                patch.dict(sys.modules, {'scripts.tools.gamepad_control.ros_bridge': bridge_module}), \
                patch('scripts.tools.gamepad_control.stack_supervisor.start_stack', side_effect=spawn), \
                patch.object(run, '_exec_window', side_effect=RuntimeError('window failed')):
            with self.assertRaisesRegex(RuntimeError, 'bridge cleanup failed'):
                run._run_stack(args, argparse.ArgumentParser())
        report = self.read_json(runs[0]/'cleanup.json')
        self.assertFalse(report['remaining_pids'])
        self.assertFalse(live_group_members(report['pid']))
        fake_ros.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
