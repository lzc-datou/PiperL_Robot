"""Compile and exercise the actual movement buttons without connecting to a robot."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

class MotionControlsUITests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++') and shutil.which('moc') and shutil.which('pkg-config'), 'Qt tools required')
    def test_five_rows_dispatch_and_execution_gates(self):
        root=Path(__file__).resolve().parents[1]
        panel=root/'scripts/task/multi_point/rviz_panel'
        flags=subprocess.run(['pkg-config','--cflags','--libs','Qt5Widgets'],capture_output=True,text=True)
        if flags.returncode: self.skipTest('Qt5Widgets required')
        with tempfile.TemporaryDirectory() as temp:
            moc=Path(temp)/'moc.cpp';binary=Path(temp)/'editor'
            subprocess.run(['moc',str(panel/'motion_controls.hpp'),'-o',str(moc)],check=True,capture_output=True)
            subprocess.run(['g++','-std=c++17','-fPIC','-I',str(root),str(root/'tests/qt/test_motion_controls.cpp'),
                str(panel/'motion_controls.cpp'),str(moc),'-o',str(binary),*shlex.split(flags.stdout)],
                check=True,capture_output=True,text=True,timeout=60)
            output=root/'outputs/validation/motion_controls.png';output.parent.mkdir(parents=True,exist_ok=True)
            result=subprocess.run([str(binary),str(output)],env=dict(os.environ,QT_QPA_PLATFORM='offscreen'),
                                  capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
