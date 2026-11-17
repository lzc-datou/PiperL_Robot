#!/usr/bin/env bash
set -euo pipefail
PIPERL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE=runtime
case "${1:-}" in
  --build) MODE=build; shift ;;
  --python-only) MODE=python; shift ;;
esac
exec /usr/bin/env -i \
  HOME="$HOME" USER="${USER:-lzc}" LOGNAME="${LOGNAME:-lzc}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_NUMERIC=C TERM="${TERM:-xterm}" \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}" \
  PYTHONNOUSERSITE=1 ROS_DOMAIN_ID="${PIPERL_ROS_DOMAIN_ID:-43}" \
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PIPERL_ROOT="$PIPERL_ROOT" PIPERL_ENV_MODE="$MODE" \
  /bin/bash --noprofile --norc "$PIPERL_ROOT/scripts/piperl/session.bash" "$@"
