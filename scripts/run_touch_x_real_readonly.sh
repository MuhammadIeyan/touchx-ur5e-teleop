#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$root_dir/diagnostics/real-robot/ros-logs"
mkdir -p "$ROS_LOG_DIR"
exec python3 "$root_dir/scripts/touch_x_publisher.py" --real-read-only
