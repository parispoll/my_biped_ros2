#!/usr/bin/env bash
# Build an isolated official library for legacy controller_manager binaries.
set -euo pipefail
compat_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
compat_dir="$compat_workspace/.runtime/diagnostics-4.2.6"
mkdir -p "$compat_dir"
touch "$compat_workspace/.runtime/COLCON_IGNORE"
if [[ ! -d "$compat_dir/source/.git" ]]; then
  git clone --depth 1 --branch 4.2.6 https://github.com/ros/diagnostics.git "$compat_dir/source"
fi
source /opt/ros/jazzy/setup.bash
cmake -S "$compat_dir/source/diagnostic_updater" -B "$compat_dir/build" -DCMAKE_INSTALL_PREFIX="$compat_dir/install" -DBUILD_TESTING=OFF
cmake --build "$compat_dir/build" -j2
cmake --install "$compat_dir/build"
