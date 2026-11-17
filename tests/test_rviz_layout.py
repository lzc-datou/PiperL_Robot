"""Exercise the panel's actual Qt docking helper without RViz/ROS/hardware."""
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest


class RvizLayoutTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++') and shutil.which('pkg-config'),'Requires Qt build tools')
    def test_default_left_dock_geometry_and_planning_tab(self):
        flags=subprocess.run(['pkg-config','--cflags','--libs','Qt5Widgets'],capture_output=True,text=True)
        if flags.returncode: self.skipTest('Requires Qt5Widgets development files')
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            binary=str(Path(temp)/'layout_test')
            subprocess.run(['g++','-std=c++17','-fPIC','-I',str(root),
                str(root/'tests/qt/test_default_layout.cpp'),'-o',binary,*shlex.split(flags.stdout)],
                check=True,capture_output=True,text=True,timeout=60)
            result=subprocess.run([binary],env=dict(os.environ,QT_QPA_PLATFORM='offscreen'),
                                  capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
