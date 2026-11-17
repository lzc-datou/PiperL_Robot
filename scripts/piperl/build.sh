#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$ROOT/scripts/piperl/run.sh" --build bash -c '
  cd "$PIPERL_ROOT/piperl_ws"
  exec python -m colcon build --symlink-install --base-paths src "$PIPERL_ROOT/scripts/task/multi_point/rviz_panel" "$PIPERL_ROOT/scripts/task/servo_tracking" --parallel-workers 2 \
    --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
'
