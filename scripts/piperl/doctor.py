"""Read-only environment verification; never constructs a robot or CAN socket."""
import importlib, importlib.metadata, json, os, subprocess, sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix
root = Path(__file__).resolve().parents[2]
assert Path(sys.prefix) == root/'.venv-piperl', sys.prefix
assert sys.version_info[:2] == (3,12)
assert os.environ['ROS_DISTRO'] == 'jazzy'
assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
for variable in ['PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH']:
    for value in os.environ.get(variable, '').split(':'):
        assert 'Jasmine' not in value and 'conda' not in value.lower(), (variable,value)
for name in ['rclpy','moveit_msgs','control_msgs','yaml','numpy','scipy','can','pyAgxArm']:
    module = importlib.import_module(name)
    location = Path(module.__file__).resolve()
    if name == 'pyAgxArm': assert location.is_relative_to(root/'.venv-piperl'), location
    print(f'{name}: {location}')
for name in ['agx_arm_ctrl','agx_arm_description','agx_arm_msgs','agx_arm_moveit']:
    location = Path(get_package_prefix(name))
    assert location.is_relative_to(root/'piperl_ws/install'), location
    print(f'{name}: {location}')
subprocess.run([sys.executable, str(root/'scripts/piperl/verify_sources.py')], check=True)
print(f"Environment OK | ROS domain {os.environ['ROS_DOMAIN_ID']} | no hardware accessed")
