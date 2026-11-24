#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$root_dir"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_LOG_DIR="$root_dir/diagnostics/test-ros-logs"
mkdir -p "$ROS_LOG_DIR" "$root_dir/build/tests"
python3 -m pytest -q tests/real_robot tests/touch_x
g++ -std=c++17 tests/touch_x/test_force_safety.cpp -o build/tests/test_force_safety
build/tests/test_force_safety
