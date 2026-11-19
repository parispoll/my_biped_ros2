#!/usr/bin/env bash
# Reproduce the simulator dependencies without sudo or system Python changes.
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 -m venv .runtime/mujoco-venv
.runtime/mujoco-venv/bin/pip install -r tools/mujoco_biped/requirements-lock.txt
