#!/usr/bin/env bash
# RViz view; MuJoCo still supplies physics. Domain 42 is set by the sim launcher.
# Forces require explicit force_feedback:=true; hold the Touch button to move.
set -euo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$root_dir/scripts/run_touch_x_sim.sh" headless:=true rviz:=true execute:=true "$@"
