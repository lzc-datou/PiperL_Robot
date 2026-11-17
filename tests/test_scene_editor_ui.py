"""Compile and exercise the actual scene form without connecting to a robot."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

class SceneEditorUITests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++') and shutil.which('moc') and shutil.which('pkg-config'), 'Qt tools required')
    def test_numeric_draft_signal_units_and_busy_state(self):
        root=Path(__file__).resolve().parents[1]
        panel=root/'scripts/task/multi_point/rviz_panel'
        flags=subprocess.run(['pkg-config','--cflags','--libs','Qt5Widgets'],capture_output=True,text=True)
        if flags.returncode: self.skipTest('Qt5Widgets required')
        with tempfile.TemporaryDirectory() as temp:
            moc=Path(temp)/'moc.cpp';binary=Path(temp)/'editor'
            subprocess.run(['moc',str(panel/'scene_editor.hpp'),'-o',str(moc)],check=True,capture_output=True)
            subprocess.run(['g++','-std=c++17','-fPIC','-I',str(root),str(root/'tests/qt/test_scene_editor.cpp'),
                str(panel/'scene_editor.cpp'),str(moc),'-o',str(binary),*shlex.split(flags.stdout)],
                check=True,capture_output=True,text=True,timeout=60)
            output=root/'outputs/validation/scene_editor.png';output.parent.mkdir(parents=True,exist_ok=True)
            result=subprocess.run([str(binary),str(output)],env=dict(os.environ,QT_QPA_PLATFORM='offscreen'),
                                  capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
