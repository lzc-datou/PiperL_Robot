#!/usr/bin/env bash
set -euo pipefail
TOOL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$TOOL_DIR/../../.." && pwd)"
exec bash "$PROJECT_ROOT/scripts/piperl/run.sh" bash "$TOOL_DIR/session.sh" "$@"
