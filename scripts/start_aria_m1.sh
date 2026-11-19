#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export GZ_PARTITION="${GZ_PARTITION:-aria_m1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/aria-m1-logs}"
cmake -S "$task_root/practice/aria_handoffs/simulation" -B "$task_root/practice/aria_handoffs/simulation/build" >/dev/null
cmake --build "$task_root/practice/aria_handoffs/simulation/build" -j2
if [[ "${1:-}" == "--build-only" ]]; then exit 0; fi
exec ros2 launch "$task_root/practice/aria_handoffs/launch/simulation.launch.py" "$@"
