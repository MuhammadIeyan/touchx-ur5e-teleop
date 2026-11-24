#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# Deliberately do not source the external workspace containing ur_robot_driver.
source /opt/ros/humble/setup.bash
if [[ ! -f "$root_dir/install/sim_contact_feedback/share/sim_contact_feedback/local_setup.bash" ]]; then
    echo 'Build the contact plugin first: scripts/build_sim_feedback.sh' >&2
    exit 2
fi
source "$root_dir/install/sim_contact_feedback/share/sim_contact_feedback/local_setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$root_dir/diagnostics/touch-x/ros-logs"
exec ros2 launch "$root_dir/launch/touch_x_sim.launch.py" "$@"
