#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export GZ_PARTITION="${GZ_PARTITION:-aria_m2}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/aria-m2-logs}"
task_ws="$task_root/practice/aria_handoffs/m2_ws"
colcon build --base-paths "$task_ws/src" --build-base "$task_ws/build" --install-base "$task_ws/install" --packages-select aria_interfaces --event-handlers console_cohesion-
source "$task_ws/install/setup.bash"
cmake -S "$task_root/practice/aria_handoffs/simulation" -B "$task_root/practice/aria_handoffs/simulation/build" >/dev/null
cmake --build "$task_root/practice/aria_handoffs/simulation/build" -j2
if [[ "${1:-}" == "--build-only" ]]; then exit 0; fi
exec ros2 launch "$task_root/practice/aria_handoffs/launch/simulation.launch.py" supported_actuation:=true "$@"
