#!/usr/bin/env bash
# 由 run.sh 在项目 ROS 环境内调用，只增加此工具的独立依赖。
set -eo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$TOOL_DIR/../../.." && pwd)"
PREFIX="$PROJECT_ROOT/.deps/handeye_prefix"
WORKSPACE="$PROJECT_ROOT/.deps/handeye_ws"
if [[ -f "$PREFIX/opt/ros/jazzy/local_setup.bash" ]]; then
  source "$PREFIX/opt/ros/jazzy/local_setup.bash"
else
  export PYTHONPATH="$PREFIX/opt/ros/jazzy/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
export PYTHONPATH="$PREFIX/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$PREFIX/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ -f "$WORKSPACE/install/local_setup.bash" ]]; then
  source "$WORKSPACE/install/local_setup.bash"
fi
if [[ "${1:-}" == __build ]]; then
  cd "$WORKSPACE"
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  exec colcon build --symlink-install --executor sequential \
    --packages-up-to aruco_ros handeye_calibration_ros realsense2_camera realsense2_description \
    --cmake-clean-cache \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    "-Drealsense2_DIR=$PREFIX/usr/lib/x86_64-linux-gnu/cmake/realsense2"
fi
exec python "$TOOL_DIR/cli.py" "$@"
