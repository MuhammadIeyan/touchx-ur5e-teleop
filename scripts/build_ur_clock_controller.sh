#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cmake -S "$root_dir/ros2/ur5e_clock_fixed_controller" \
  -B "$root_dir/build/ur5e_clock_fixed_controller" \
  -DCMAKE_INSTALL_PREFIX="$root_dir/install/ur5e_clock_fixed_controller" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$root_dir/build/ur5e_clock_fixed_controller" -j2
cmake --install "$root_dir/build/ur5e_clock_fixed_controller"
