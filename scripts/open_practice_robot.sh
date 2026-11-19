#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LD_LIBRARY_PATH="$task_root/tools/openscad-local/usr/lib:$task_root/tools/openscad-local/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$task_root/tools/openscad-local/usr/bin/openscad" "$task_root/practice/aria_handoffs/robot.scad" "$@"
