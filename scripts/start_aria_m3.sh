#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-45}"
export GZ_PARTITION="${GZ_PARTITION:-aria_m3}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/aria-m3-logs}"
exec bash "$task_root/scripts/start_aria_m2.sh" contacts:=true support_height:=0.4213 "$@"
