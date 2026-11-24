#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
exec bash "$ROOT/scripts/piperl/run.sh" python -m scripts.tools.d405_depth_accuracy.run "$@"
