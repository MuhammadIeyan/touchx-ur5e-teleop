#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cmake -S "$root_dir/ros2/sim_contact_feedback" -B "$root_dir/build/sim_contact_feedback" -DCMAKE_INSTALL_PREFIX="$root_dir/install/sim_contact_feedback"
# ROS package upgrades may install headers with older preserved timestamps.
# Always rebuild this small plugin so its virtual interface matches the host.
cmake --build "$root_dir/build/sim_contact_feedback" --clean-first -j2
cmake --install "$root_dir/build/sim_contact_feedback"
