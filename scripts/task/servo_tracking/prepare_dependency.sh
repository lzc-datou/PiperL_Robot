#!/usr/bin/env bash
set -euo pipefail
PIPERL_TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
PIPERL_DOWNLOAD_DIR="$(mktemp -d /tmp/piper-servo-dependency.XXXXXX)"
trap 'rm -rf -- "$PIPERL_DOWNLOAD_DIR"' EXIT
cd "$PIPERL_DOWNLOAD_DIR"
apt-get download ros-jazzy-moveit-servo=2.12.4-1noble.20261006.040508
printf '%s\n' '3abc60814f191fc69dca87688c4cc71ac2e63c1f4d86fb6cb1f22a189a15cb2f  ros-jazzy-moveit-servo_2.12.4-1noble.20261006.040508_amd64.deb' | sha256sum -c -
mkdir -p "$PIPERL_TASK_ROOT/.deps/servo"
dpkg-deb -x ros-jazzy-moveit-servo_2.12.4-1noble.20261006.040508_amd64.deb "$PIPERL_TASK_ROOT/.deps/servo"
