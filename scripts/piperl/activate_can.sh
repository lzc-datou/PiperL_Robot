#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CHANNEL="$(/usr/bin/python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["can_port"])' "$ROOT/config/baseline.json")"
[[ $# -le 1 ]] || { echo 'Usage: activate_can.sh [USB_BUS_ADDRESS]'; exit 2; }
exec bash "$ROOT/piperl_ws/src/agx_arm_ros/scripts/can_activate.sh" "$CHANNEL" 1000000 "$@"
