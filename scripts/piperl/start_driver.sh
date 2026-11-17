#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$ROOT/scripts/piperl/run.sh" python "$ROOT/scripts/piperl/launch_official.py" driver "$@"
