#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="${1:---offline}"
[[ $# -le 1 && ( "$MODE" == --offline || "$MODE" == --online ) ]] || { echo 'Usage: bootstrap.sh [--offline|--online]'; exit 2; }
cd "$ROOT"
/usr/bin/env -i HOME="$HOME" PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 /usr/bin/python3 scripts/piperl/verify_sources.py
if [[ ! -d .venv-piperl ]]; then
  /usr/bin/env -i HOME="$HOME" PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages .venv-piperl
fi
mkdir -p external/wheels
if [[ "$MODE" == --online ]]; then
  bash scripts/piperl/run.sh --python-only python -m pip download --only-binary=:all: -r requirements.lock -d external/wheels
fi
bash scripts/piperl/run.sh --python-only python -m pip install --no-index --no-build-isolation --find-links external/wheels -r requirements.lock ./external/pyAgxArm
bash scripts/piperl/build.sh
