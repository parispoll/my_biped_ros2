#!/usr/bin/env bash
set -e
panel_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$panel_workspace/install/setup.bash" ]]; then
  echo 'ROS Jazzy and a built workspace are required. See docs/guides/FORCE_PANEL.md.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$panel_workspace/install/setup.bash"
cd "$panel_workspace"
exec python3 scripts/force_panel.py "$@"
