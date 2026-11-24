#!/usr/bin/env bash
# No driver start, no motion publishers; listens to an existing domain-43 driver.
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$root_dir/diagnostics/real-robot/ros-logs"
mkdir -p "$ROS_LOG_DIR"
exec python3 "$root_dir/scripts/real_robot/observe.py" --output \
    "$root_dir/diagnostics/real-robot/observation-$(date +%Y%m%d-%H%M%S).jsonl"
