#!/usr/bin/env bash
set -eo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cmake -S "$root_dir/ros2/sim_contact_feedback" -B "$root_dir/build/sim_contact_feedback" -DCMAKE_INSTALL_PREFIX="$root_dir/install/sim_contact_feedback"
cmake --build "$root_dir/build/sim_contact_feedback" -j2
cmake --install "$root_dir/build/sim_contact_feedback"
