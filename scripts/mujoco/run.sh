#!/usr/bin/env bash
# No ROS processes are required; source only makes Xacro available to the importer.
set -euo pipefail
cd "$(dirname "$0")/../.."
export BIPED_ACTUATOR_PROFILE=robotis_p54
export PYTHONPATH="$PWD:$PWD/src/my_robot_description:/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec .runtime/mujoco-venv/bin/python -m tools.mujoco_biped.simulate "$@"
