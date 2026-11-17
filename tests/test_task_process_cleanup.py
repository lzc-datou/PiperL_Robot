"""Real disposable process trees only: no ROS nodes, robot driver or CAN access."""
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from scripts.task.common.process_cleanup import live_group_members,stop_process_group


@unittest.skipUnless(sys.platform=='linux','Linux process groups and /proc')
class ProcessCleanupTests(unittest.TestCase):
    def setUp(self):
        # Reap orphaned fixture children here instead of leaving them to container init.
        self.libc=ctypes.CDLL(None,use_errno=True)
        self.previous=ctypes.c_int()
        self.assertEqual(self.libc.prctl(37,ctypes.byref(self.previous),0,0,0),0)
        self.assertEqual(self.libc.prctl(36,1,0,0,0),0)
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.processes=[]
        self.children=[]

    def tearDown(self):
        for process in self.processes:
            try: os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError: pass
            process.wait(timeout=3)
        for pid in self.children:
            try: os.waitpid(pid,0)
            except ChildProcessError: pass
        self.libc.prctl(36,self.previous.value,0,0,0)
        self.temp.cleanup()

    def tree(self,early_exit=False):
        ready=self.root/f'ready{len(self.processes)}'
        code='''
import json,os,signal,sys,time
from pathlib import Path
ready=Path(sys.argv[1])
child=os.fork()
if child==0:
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    ready.write_text(json.dumps({'child':os.getpid()}))
    while True: time.sleep(1)
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
while not ready.exists(): time.sleep(.01)
if sys.argv[2]=='early': sys.exit(0)
while True: time.sleep(1)
'''
        process=subprocess.Popen([sys.executable,'-c',code,str(ready),'early' if early_exit else 'wait'],start_new_session=True)
        self.processes.append(process)
        deadline=time.monotonic()+3
        while not ready.exists():
            if time.monotonic()>deadline: self.fail('Fixture did not become ready')
            time.sleep(.01)
        # The write may still be in progress when the file first appears.
        while True:
            try: child=json.loads(ready.read_text())['child'];break
            except json.JSONDecodeError:
                if time.monotonic()>deadline: raise
                time.sleep(.01)
        self.children.append(child)
        return process,child

    def test_leader_exit_does_not_skip_stubborn_child_and_external_group_survives(self):
        process,child=self.tree()
        external=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],start_new_session=True)
        self.processes.append(external)
        report=stop_process_group(process,(.1,.1,1))
        self.assertEqual(report['signals'],['SIGINT','SIGTERM','SIGKILL'])
        self.assertEqual(report['returncode'],0)
        self.assertEqual(report['remaining_pids'],[])
        self.assertEqual(report['errors'],[])
        self.assertEqual(live_group_members(process.pid),[])
        self.assertIsNone(external.poll())

    def test_already_exited_launcher_still_cleans_child(self):
        process,child=self.tree(early_exit=True)
        process.wait(timeout=3)
        self.assertIn(child,live_group_members(process.pid))
        report=stop_process_group(process,(.05,.05,1))
        self.assertEqual(report['signals'][-1],'SIGKILL')
        self.assertEqual(report['remaining_pids'],[])

    def test_exited_empty_group_needs_no_signal(self):
        process=subprocess.Popen([sys.executable,'-c','pass'],start_new_session=True)
        self.processes.append(process)
        process.wait(timeout=3)
        report=stop_process_group(process,(.05,.05,.1))
        self.assertEqual(report['signals'],[])
        self.assertEqual(report['remaining_pids'],[])


if __name__=='__main__': unittest.main()
