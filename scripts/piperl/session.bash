#!/usr/bin/env bash
set -eo pipefail
VENV="$PIPERL_ROOT/.venv-piperl"
[[ -x "$VENV/bin/python" ]] || { echo 'Run scripts/piperl/bootstrap.sh first.' >&2; exit 1; }
"$VENV/bin/python" -I -c 'import pathlib,sys; assert sys.version_info[:2] == (3,12); assert pathlib.Path(sys.prefix)==pathlib.Path(sys.argv[1]), "Recreate relocated venv"' "$VENV"
if [[ "$PIPERL_ENV_MODE" != python ]]; then source /opt/ros/jazzy/setup.bash; fi
# Optional project-local official Servo binary, without modifying system ROS/APT.
PIPERL_SERVO_PREFIX="$PIPERL_ROOT/.deps/servo/opt/ros/jazzy"
if [[ "$PIPERL_ENV_MODE" != python && -d "$PIPERL_SERVO_PREFIX/share/moveit_servo" ]]; then
  export AMENT_PREFIX_PATH="$PIPERL_SERVO_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$PIPERL_SERVO_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
source "$VENV/bin/activate"
[[ "$VIRTUAL_ENV" == "$VENV" ]] || { echo 'Recreate relocated venv.' >&2; exit 1; }
export PIPER_PYTHON="$VENV/bin/python"
if [[ "$PIPERL_ENV_MODE" == runtime ]]; then source "$PIPERL_ROOT/piperl_ws/install/local_setup.bash"; fi
export ROS_LOG_DIR="$PIPERL_ROOT/outputs/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$PIPERL_ROOT"
if [[ $# == 0 ]]; then exec /bin/bash --noprofile --norc -i; fi
if [[ "$1" == ros2 ]]; then shift; exec "$PIPER_PYTHON" /opt/ros/jazzy/bin/ros2 "$@"; fi
exec "$@"
