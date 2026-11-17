"""Translate baseline settings into official launch arguments; no custom control."""
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.piperl.deployment import load_driver_config
mode = sys.argv[1]
if mode not in ('driver', 'moveit'): raise SystemExit('Expected driver or moveit')
settings = load_driver_config()
name = 'start_single_agx_arm_moveit.launch.py' if mode == 'moveit' else 'start_single_agx_arm.launch.py'
arguments = [f'{key}:={str(value).lower() if isinstance(value, bool) else value}' for key, value in settings.items()]
arguments += ['follow:=true', 'auto_control_gate:=false'] if mode == 'moveit' else ['control_enabled:=true']
# Explicit CLI overrides replace the matching default once, rather than duplicate it.
extra = sys.argv[2:]
overrides = {value.split(':=', 1)[0] for value in extra if ':=' in value}
arguments = [value for value in arguments if value.split(':=', 1)[0] not in overrides]
command = [sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', 'agx_arm_ctrl', name, *arguments, *extra]
print('Official launch:', ' '.join(command[2:]), flush=True)
os.execv(sys.executable, command)
