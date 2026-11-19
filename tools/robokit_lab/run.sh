#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
# ROS setup scripts reference optional unset variables.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-/tmp/robokit-cache}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/robokit-matplotlib}"
exec "$ROOT/.venv-robokit/bin/python" -u -m tools.robokit_lab.check "$@"
