"""Check scene editor properties against the installed RViz plugin, without ROS nodes."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


@unittest.skipUnless(os.environ.get('AMENT_PREFIX_PATH'),'Use ROS runtime wrapper')
class SceneMarkerDisplayTests(unittest.TestCase):
    def test_installed_interactive_marker_namespace_property(self):
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)
            (source/'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.16)
project(scene_marker_property_test)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(check_properties "{root}/tests/qt/test_scene_marker_display.cpp")
target_include_directories(check_properties PRIVATE "{root}")
ament_target_dependencies(check_properties rviz_common)
target_link_libraries(check_properties Qt5::Widgets)
''')
            import sys
            for command in (['cmake','-S',temp,'-B',temp+'/build','-DPython3_EXECUTABLE='+sys.executable],
                            ['cmake','--build',temp+'/build','-j2'],[temp+'/build/check_properties']):
                result=subprocess.run(command,capture_output=True,text=True,timeout=60,
                    env=dict(os.environ,QT_QPA_PLATFORM='offscreen'))
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
